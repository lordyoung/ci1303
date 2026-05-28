#include <math.h>
#include <string.h>
#include "feat_extract.h"
#include "ci_fft.h"
#include "ci_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* Static work buffers — never placed on task stack */
static float s_hamming[SPK_FRAME_LEN];
static int   s_mel_lower[SPK_N_MEL];
static int   s_mel_center[SPK_N_MEL];
static int   s_mel_upper[SPK_N_MEL];
static float s_dct_cos[SPK_N_MFCC_BASE][SPK_N_MEL];   /* 13 × 26 */
static float s_fft_buf[SPK_FFT_N];                     /* 256 complex = 512 floats */
static float s_power[SPK_N_FFT_BINS];                  /* 256 */
static float s_mel_e[SPK_N_MEL];                       /* 26 */
static int   s_inited;

static float mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

int feat_init(void)
{
    if (s_inited) return 0;

    /* Hamming window */
    for (int i = 0; i < SPK_FRAME_LEN; i++)
        s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (SPK_FRAME_LEN - 1));

    /* Mel filterbank: 26 triangular filters, 80–8000 Hz */
    float mel_lo = 2595.0f * log10f(1.0f + 80.0f   / 700.0f);
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

    /* Init SDK FFT (w=512, s=256) */
    ci_software_fft_w512_s256_init();

    s_inited = 1;
    mprintf("[SPK] feat_init done\n");
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

        /* SDK FFT: windowed int16 → 256 interleaved complex floats */
        ci_software_fft_w512_s256(pcm + off, s_hamming, s_fft_buf);

        /* Power spectrum: |X[k]|^2 */
        for (int k = 0; k < SPK_N_FFT_BINS; k++) {
            float re = s_fft_buf[2 * k];
            float im = s_fft_buf[2 * k + 1];
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