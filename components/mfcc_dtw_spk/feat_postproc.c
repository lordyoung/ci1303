#include <string.h>
#include "feat_postproc.h"

static float s_delta_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];

static int safe_frame(int t, int n) {
    if (t < 0)  return 0;
    if (t >= n) return n - 1;
    return t;
}

static void compute_delta(const float src[][SPK_N_MFCC_BASE], int n_frames,
                           float dst[][SPK_N_MFCC_BASE])
{
    for (int t = 0; t < n_frames; t++) {
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            float d =
                1.0f * (src[safe_frame(t+1, n_frames)][k] - src[safe_frame(t-1, n_frames)][k])
              + 2.0f * (src[safe_frame(t+2, n_frames)][k] - src[safe_frame(t-2, n_frames)][k]);
            dst[t][k] = d / 10.0f;
        }
    }
}

void feat_apply_cmn(float mfcc[][SPK_N_MFCC_BASE], int n_frames)
{
    if (n_frames <= 0) return;
    for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
        float sum = 0.0f;
        for (int t = 0; t < n_frames; t++) sum += mfcc[t][k];
        float mean = sum / n_frames;
        for (int t = 0; t < n_frames; t++) mfcc[t][k] -= mean;
    }
}

int feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                         float feat_out[][SPK_FEAT_DIM])
{
    if (n_frames <= 0 || n_frames > SPK_MAX_TEMPLATE_FRAMES) return 0;

    compute_delta(mfcc, n_frames, s_delta_buf);

    for (int t = 0; t < n_frames; t++) {
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            feat_out[t][k] = mfcc[t][k];
        }
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            feat_out[t][SPK_N_MFCC_BASE + k] = s_delta_buf[t][k];
        }
    }

    float (*delta_ptr)[SPK_N_MFCC_BASE] = s_delta_buf;
    for (int t = 0; t < n_frames; t++) {
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            float d =
                1.0f * (delta_ptr[safe_frame(t+1, n_frames)][k] - delta_ptr[safe_frame(t-1, n_frames)][k])
              + 2.0f * (delta_ptr[safe_frame(t+2, n_frames)][k] - delta_ptr[safe_frame(t-2, n_frames)][k]);
            feat_out[t][2 * SPK_N_MFCC_BASE + k] = d / 10.0f;
        }
    }

    return n_frames;
}