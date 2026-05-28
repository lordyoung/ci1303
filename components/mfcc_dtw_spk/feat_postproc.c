#include "feat_postproc.h"
#include "user_config.h"

/* Intermediate buffer for Δ — static to avoid stack overflow */
static float s_d1[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];

void feat_apply_cmn(float feats[][SPK_N_MFCC_BASE], int n_frames)
{
    for (int d = 0; d < SPK_N_MFCC_BASE; d++) {
        float mean = 0.0f;
        for (int t = 0; t < n_frames; t++) mean += feats[t][d];
        mean /= (float)n_frames;
        for (int t = 0; t < n_frames; t++) feats[t][d] -= mean;
    }
}

/* D=2 central difference for sequence f[0..n-1] at time t, dim d */
static float central_diff(const float (*f)[SPK_N_MFCC_BASE], int t, int n, int d)
{
    int t1p = (t + 1 < n) ? t + 1 : n - 1;
    int t1m = (t - 1 >= 0) ? t - 1 : 0;
    int t2p = (t + 2 < n) ? t + 2 : n - 1;
    int t2m = (t - 2 >= 0) ? t - 2 : 0;
    return (f[t1p][d] - f[t1m][d] + 2.0f * (f[t2p][d] - f[t2m][d])) / 10.0f;
}

void feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                          float out[][SPK_FEAT_DIM])
{
    int cap = (n_frames < SPK_MAX_TEMPLATE_FRAMES) ? n_frames : SPK_MAX_TEMPLATE_FRAMES;

    /* Compute first-order delta into s_d1 */
    for (int t = 0; t < cap; t++)
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            s_d1[t][d] = central_diff(mfcc, t, cap, d);

    /* Pack [mfcc | Δ | ΔΔ] */
    for (int t = 0; t < cap; t++) {
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            out[t][d] = mfcc[t][d];
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            out[t][SPK_N_MFCC_BASE + d] = s_d1[t][d];
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            out[t][2 * SPK_N_MFCC_BASE + d] = central_diff(s_d1, t, cap, d);
    }
}