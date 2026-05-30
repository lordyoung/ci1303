#include <math.h>
#include <string.h>
#include "feat_extract.h"
#include "ci_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* Static work buffers — never placed on task stack */
static float s_hamming[SPK_FRAME_LEN];                 /* 512 -> 2KB */
static float s_tw_re[SPK_FFT_N / 2];                   /* 256 -> 1KB twiddle cos */
static float s_tw_im[SPK_FFT_N / 2];                   /* 256 -> 1KB twiddle sin */
static float s_fft_re[SPK_FFT_N];                      /* 512 -> 2KB real work */
static float s_fft_im[SPK_FFT_N];                      /* 512 -> 2KB imag work */
static int   s_mel_lower[SPK_N_MEL];
static int   s_mel_center[SPK_N_MEL];
static int   s_mel_upper[SPK_N_MEL];
static float s_dct_cos[SPK_N_MFCC_BASE][SPK_N_MEL];    /* 13 × 26 */
static float s_power[SPK_N_FFT_BINS];                  /* 256 */
static float s_mel_e[SPK_N_MEL];                       /* 26 */
static int   s_inited;

static float mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

/* In-place iterative radix-2 Cooley-Tukey FFT, length SPK_FFT_N (512).
 * Twiddles precomputed in s_tw_re/s_tw_im. Self-contained — no SDK/ROM FFT. */
static void fft_512(float *re, float *im)
{
    const int n = SPK_FFT_N;

    /* bit-reversal permutation */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    /* butterflies */
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                int idx = k * step;
                float wr = s_tw_re[idx];
                float wi = s_tw_im[idx];
                float xr = re[i + k + half];
                float xi = im[i + k + half];
                float vr = xr * wr - xi * wi;
                float vi = xr * wi + xi * wr;
                float ur = re[i + k];
                float ui = im[i + k];
                re[i + k]        = ur + vr;
                im[i + k]        = ui + vi;
                re[i + k + half] = ur - vr;
                im[i + k + half] = ui - vi;
            }
        }
    }
}

int feat_init(void)
{
    if (s_inited) return 0;

    /* Hamming window */
    for (int i = 0; i < SPK_FRAME_LEN; i++)
        s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (SPK_FRAME_LEN - 1));

    /* FFT twiddle factors: W_n^i = exp(-j*2*pi*i/n) */
    for (int i = 0; i < SPK_FFT_N / 2; i++) {
        float ang = -2.0f * M_PI * i / (float)SPK_FFT_N;
        s_tw_re[i] = cosf(ang);
        s_tw_im[i] = sinf(ang);
    }

    /* Mel filterbank: 26 triangular filters, 80–8000 Hz */
        float mel_lo = 2595.0f * log10f(1.0f + (float)SPK_MEL_LOW_HZ / 700.0f);
    float mel_hi = 2595.0f * log10f(1.0f + 8000.0f / 700.0f);
    int bin_pts[SPK_N_MEL + 2];
    for (int i = 0; i < SPK_N_MEL + 2; i++) {
        float m  = mel_lo + (mel_hi - mel_lo) * i / (SPK_N_MEL + 1);
        float hz = mel_to_hz(m);
        int   b  = (int)(hz * (SPK_FFT_N + 1) / (float)SPK_SAMPLE_RATE);
        if (b < 0)                b = 0;
        if (b >= SPK_N_FFT_BINS)  b = SPK_N_FFT_BINS - 1;
        bin_pts[i] = b;
    }
    for (int m = 0; m < SPK_N_MEL; m++) {
        s_mel_lower[m]  = bin_pts[m];
        s_mel_center[m] = bin_pts[m + 1];
        s_mel_upper[m]  = bin_pts[m + 2];
    }

    /* DCT-II cosine table: coeff n=1..13, Mel bin m=0..25
     *   s_dct_cos[n][m] = cos(π*(n+1)*(m+0.5)/M) */
    for (int n = 0; n < SPK_N_MFCC_BASE; n++)
        for (int m = 0; m < SPK_N_MEL; m++)
            s_dct_cos[n][m] = cosf(M_PI * (n + 1) * (m + 0.5f) / SPK_N_MEL);

    s_inited = 1;
    mprintf("[SPK] feat_init: mel=%d mfcc=%d ",SPK_N_MEL, SPK_N_MFCC_BASE);
    return 0;
}

int feat_extract_mfcc(const short *pcm, int n_samples,
                      float out[][SPK_N_MFCC_BASE], int n_max_frames)
{
    if (!s_inited) return 0;

    int n_frames = 0;
    for (int off = 0;
         off + SPK_FRAME_LEN <= n_samples && n_frames < n_max_frames;
         off += SPK_FRAME_SHIFT, n_frames++) {

        /* Apply Hamming window: int16 PCM -> float, zero imaginary */
                /* 预加重(高通)后加汉明窗：int16 PCM -> float，虚部清零。
         * 预加重 y[n]=x[n]-a*x[n-1] 压低频(风噪)、提高频，是标准MFCC的第一步。 */
        {
            const float a = (float)SPK_PREEMPH_X100 / 100.0f;
            for (int i = 0; i < SPK_FFT_N; i++) {
                int   idx  = off + i;
                float cur  = (float)pcm[idx];
                float prev = (idx > 0) ? (float)pcm[idx - 1] : cur;
                float pe   = cur - a * prev;
                s_fft_re[i] = pe * s_hamming[i];
                s_fft_im[i] = 0.0f;
            }
        }

        fft_512(s_fft_re, s_fft_im);

        /* Power spectrum: |X[k]|^2 for bins 0..255 */
        for (int k = 0; k < SPK_N_FFT_BINS; k++) {
            float re = s_fft_re[k];
            float im = s_fft_im[k];
            s_power[k] = re * re + im * im;
        }

        /* Triangular Mel filterbank → log energy */
        for (int m = 0; m < SPK_N_MEL; m++) {
            int lo  = s_mel_lower[m];
            int mid = s_mel_center[m];
            int hi  = s_mel_upper[m];
            float e = 0.0f;

            if (mid > lo)
                for (int k = lo + 1; k <= mid && k < SPK_N_FFT_BINS; k++)
                    e += s_power[k] * (float)(k - lo) / (float)(mid - lo);
            if (mid < SPK_N_FFT_BINS)
                e += s_power[mid];
            if (hi > mid)
                for (int k = mid + 1; k < hi && k < SPK_N_FFT_BINS; k++)
                    e += s_power[k] * (float)(hi - k) / (float)(hi - mid);

            s_mel_e[m] = logf(e > 1e-6f ? e : 1e-6f);
        }

        /* DCT-II → 13 MFCC coefficients (C1..C13) */
        for (int n = 0; n < SPK_N_MFCC_BASE; n++) {
            float c = 0.0f;
            for (int m = 0; m < SPK_N_MEL; m++)
                c += s_mel_e[m] * s_dct_cos[n][m];
            out[n_frames][n] = c;
        }
    }
    return n_frames;
}