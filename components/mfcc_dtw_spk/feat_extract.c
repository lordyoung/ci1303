#include <math.h>
#include <string.h>
#include "feat_extract.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float s_hamming[SPK_FRAME_LEN];
static int   s_mel_bins[SPK_N_MEL + 2];
static float s_dct_cos[SPK_N_MFCC_BASE][SPK_N_MEL];

static float s_fft_re[SPK_FFT_N];
static float s_fft_im[SPK_FFT_N];

static float hz_to_mel(float hz) { return 2595.0f * log10f(1.0f + hz / 700.0f); }
static float mel_to_hz(float mel) { return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f); }

static void fft_radix2(float *re, float *im, int n)
{
    int j = 0;
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wr0 = cosf(ang), wi0 = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float wr = 1.0f, wi = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                float ur = re[i+k],          ui = im[i+k];
                float vr = re[i+k+len/2]*wr - im[i+k+len/2]*wi;
                float vi = re[i+k+len/2]*wi + im[i+k+len/2]*wr;
                re[i+k]         = ur + vr;
                im[i+k]         = ui + vi;
                re[i+k+len/2]   = ur - vr;
                im[i+k+len/2]   = ui - vi;
                float nwr = wr*wr0 - wi*wi0;
                float nwi = wr*wi0 + wi*wr0;
                wr = nwr; wi = nwi;
            }
        }
    }
}

void feat_init(void)
{
    for (int i = 0; i < SPK_FRAME_LEN; i++) {
        s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (SPK_FRAME_LEN - 1));
    }

    float mel_lo = hz_to_mel(80.0f);
    float mel_hi = hz_to_mel(8000.0f);
    for (int i = 0; i < SPK_N_MEL + 2; i++) {
        float mel = mel_lo + (mel_hi - mel_lo) * i / (SPK_N_MEL + 1);
        float hz  = mel_to_hz(mel);
        s_mel_bins[i] = (int)floorf((SPK_FFT_N + 1) * hz / SPK_SAMPLE_RATE);
        if (s_mel_bins[i] >= SPK_N_FFT_BINS) s_mel_bins[i] = SPK_N_FFT_BINS - 1;
    }

    for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
        for (int m = 0; m < SPK_N_MEL; m++) {
            s_dct_cos[k][m] = cosf((float)M_PI * (k + 1) * (m + 0.5f) / SPK_N_MEL);
        }
    }
}

int feat_extract_mfcc(const short *pcm, int n_samples,
                      float mfcc_out[][SPK_N_MFCC_BASE],
                      int max_frames, int *out_frames)
{
    static float power[SPK_N_FFT_BINS];
    static float mel_energy[SPK_N_MEL];

    int n_frames = 0;
    int offset   = 0;

    while ((offset + SPK_FRAME_LEN) <= n_samples && n_frames < max_frames) {
        s_fft_re[0] = (float)pcm[offset] * s_hamming[0];
        s_fft_im[0] = 0.0f;
        for (int i = 1; i < SPK_FRAME_LEN; i++) {
            float s = (float)pcm[offset + i] - 0.97f * (float)pcm[offset + i - 1];
            s_fft_re[i] = s * s_hamming[i];
            s_fft_im[i] = 0.0f;
        }

        fft_radix2(s_fft_re, s_fft_im, SPK_FFT_N);

        for (int i = 0; i < SPK_N_FFT_BINS; i++) {
            power[i] = s_fft_re[i]*s_fft_re[i] + s_fft_im[i]*s_fft_im[i];
        }

        for (int m = 0; m < SPK_N_MEL; m++) {
            int lo  = s_mel_bins[m];
            int ctr = s_mel_bins[m + 1];
            int hi  = s_mel_bins[m + 2];
            float val = 0.0f;
            for (int k = lo; k < ctr && k < SPK_N_FFT_BINS; k++) {
                val += power[k] * (float)(k - lo) / (float)(ctr - lo + 1);
            }
            for (int k = ctr; k <= hi && k < SPK_N_FFT_BINS; k++) {
                val += power[k] * (float)(hi - k + 1) / (float)(hi - ctr + 1);
            }
            mel_energy[m] = logf(val + 1e-10f);
        }

        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            float sum = 0.0f;
            for (int m = 0; m < SPK_N_MEL; m++) {
                sum += mel_energy[m] * s_dct_cos[k][m];
            }
            mfcc_out[n_frames][k] = sum;
        }

        n_frames++;
        offset += SPK_FRAME_SHIFT;
    }

    *out_frames = n_frames;
    return (n_frames > 0) ? 0 : -1;
}