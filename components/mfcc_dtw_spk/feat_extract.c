/**
 * @file feat_extract.c
 * @brief MFCC feature extraction: pre-emphasis -> Hamming -> FFT -> Mel -> log -> DCT
 *        16kHz, 512-point FFT, 26 Mel filters, 13 MFCC coefficients (C1..C13)
 */
#include <math.h>
#include <string.h>
#include "feat_extract.h"
#include "user_config.h"

/* ========== 内部常量 ========== */
#define F_LOW       80.0f
#define F_HIGH      8000.0f
#define SR          16000
#define PRE_EMPH    0.97f
#define PI          3.14159265f

/* ========== 静态预计算表（仅初始化一次）========== */
static int   s_init_done;
static float s_hamming[SPK_FRAME_LEN];                  /* 2 KB */
static int   s_mel_bins[SPK_N_MEL + 2];                 /* 28 个 bin 边界 */
static float s_dct_cos[SPK_N_MFCC_BASE][SPK_N_MEL];     /* 13×26 = 1.4 KB */

/* FFT 工作缓冲（逐帧复用，不与外部数据重叠）*/
static float s_fft_re[SPK_FFT_N];                       /* 2 KB */
static float s_fft_im[SPK_FFT_N];                       /* 2 KB */

/* ========== 内部工具函数 ========== */

static float hz_to_mel(float hz)
{
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static float mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

/* 原地 Cooley-Tukey 基-2 DIT FFT，N 必须是 2 的幂 */
static void fft_inplace(float *re, float *im, int N)
{
    /* 位逆序置换 */
    for (int i = 1, j = 0; i < N; i++) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    /* 蝶形运算 */
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -PI / (float)(len >> 1);
        float wR = cosf(ang), wI = sinf(ang);
        for (int i = 0; i < N; i += len) {
            float curR = 1.0f, curI = 0.0f;
            for (int k = 0; k < (len >> 1); k++) {
                float uR = re[i + k];
                float uI = im[i + k];
                float vR = re[i + k + (len >> 1)] * curR - im[i + k + (len >> 1)] * curI;
                float vI = re[i + k + (len >> 1)] * curI + im[i + k + (len >> 1)] * curR;
                re[i + k]            = uR + vR;
                im[i + k]            = uI + vI;
                re[i + k + (len>>1)] = uR - vR;
                im[i + k + (len>>1)] = uI - vI;
                float nR = curR * wR - curI * wI;
                curI     = curR * wI + curI * wR;
                curR     = nR;
            }
        }
    }
}

/* ========== 公开 API ========== */

void feat_init(void)
{
    if (s_init_done) return;

    /* 汉明窗 */
    for (int i = 0; i < SPK_FRAME_LEN; i++) {
        s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * PI * i / (SPK_FRAME_LEN - 1));
    }

    /* Mel 滤波器组 bin 边界：共 N_MEL+2 个端点 */
    float mel_low  = hz_to_mel(F_LOW);
    float mel_high = hz_to_mel(F_HIGH);
    for (int i = 0; i <= SPK_N_MEL + 1; i++) {
        float mel = mel_low + (float)i * (mel_high - mel_low) / (float)(SPK_N_MEL + 1);
        float hz  = mel_to_hz(mel);
        s_mel_bins[i] = (int)floorf((float)(SPK_FFT_N + 1) * hz / (float)SR);
        /* 边界保护 */
        if (s_mel_bins[i] < 0)             s_mel_bins[i] = 0;
        if (s_mel_bins[i] > SPK_N_FFT_BINS - 1) s_mel_bins[i] = SPK_N_FFT_BINS - 1;
    }

    /* DCT-II 系数表：C1..C13
     * mfcc[k] = sum_{m=0}^{N_MEL-1} log_mel[m] * cos(PI*(k+1)*(m+0.5)/N_MEL)
     */
    for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
        for (int m = 0; m < SPK_N_MEL; m++) {
            s_dct_cos[k][m] = cosf(PI * (float)(k + 1) * ((float)m + 0.5f) / (float)SPK_N_MEL);
        }
    }

    s_init_done = 1;
}

int feat_extract_mfcc(const short *pcm, int n_samples,
                      float mfcc_out[][SPK_N_MFCC_BASE],
                      int max_frames, int *out_frames)
{
    if (!s_init_done) feat_init();
    if (!pcm || n_samples < SPK_FRAME_LEN) {
        if (out_frames) *out_frames = 0;
        return -1;
    }

    int n_frames = 0;
    int offset   = 0;

    while ((offset + SPK_FRAME_LEN) <= n_samples && n_frames < max_frames) {

        /* 1. 预加重 + 汉明窗 → 实部；虚部清零 */
        s_fft_re[0] = ((float)pcm[offset] - PRE_EMPH * (offset > 0 ? (float)pcm[offset - 1] : 0.0f))
                      * s_hamming[0];
        for (int i = 1; i < SPK_FRAME_LEN; i++) {
            float cur = (float)pcm[offset + i];
            float pre = (float)pcm[offset + i - 1];
            s_fft_re[i] = (cur - PRE_EMPH * pre) * s_hamming[i];
        }
        memset(s_fft_im, 0, sizeof(float) * SPK_FFT_N);

        /* 2. 512 点 FFT */
        fft_inplace(s_fft_re, s_fft_im, SPK_FFT_N);

        /* 3. 功率谱 |X[k]|^2，取 bin 0..256 */
        float power[SPK_N_FFT_BINS];
        for (int k = 0; k < SPK_N_FFT_BINS; k++) {
            power[k] = s_fft_re[k] * s_fft_re[k] + s_fft_im[k] * s_fft_im[k];
        }

        /* 4. Mel 三角滤波器组：26 个能量值 */
        float mel_energy[SPK_N_MEL];
        for (int m = 0; m < SPK_N_MEL; m++) {
            int lo  = s_mel_bins[m];
            int mid = s_mel_bins[m + 1];
            int hi  = s_mel_bins[m + 2];
            float e = 0.0f;
            int span_lo = mid - lo;
            int span_hi = hi  - mid;
            if (span_lo > 0) {
                for (int k = lo; k < mid; k++)
                    e += (float)(k - lo) / (float)span_lo * power[k];
            }
            if (span_hi > 0) {
                for (int k = mid; k <= hi; k++)
                    e += (float)(hi - k) / (float)span_hi * power[k];
            }
            mel_energy[m] = e;
        }

        /* 5. 对数（自然对数，底限 1e-10 防 log(0)）*/
        for (int m = 0; m < SPK_N_MEL; m++) {
            mel_energy[m] = logf(mel_energy[m] > 1e-10f ? mel_energy[m] : 1e-10f);
        }

        /* 6. DCT-II → C1..C13 */
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            float c = 0.0f;
            for (int m = 0; m < SPK_N_MEL; m++)
                c += mel_energy[m] * s_dct_cos[k][m];
            mfcc_out[n_frames][k] = c;
        }

        n_frames++;
        offset += SPK_FRAME_SHIFT;
    }

    if (out_frames) *out_frames = n_frames;
    return (n_frames > 0) ? 0 : -1;
}