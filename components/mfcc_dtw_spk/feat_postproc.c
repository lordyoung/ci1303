/**
 * @file feat_postproc.c
 * @brief CMN (Cepstral Mean Normalization) + Delta + Delta-Delta
 *        Output: 39-dim feature vector per frame [MFCC(13) | Δ(13) | ΔΔ(13)]
 */
#include <string.h>
#include "feat_postproc.h"
#include "user_config.h"
#include "ci_log.h"

/* Delta 计算时的临时缓冲（静态，避免占用任务栈）*/
static float s_delta_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];

/* ========== 内部工具 ========== */

/* 安全取帧：超出边界时重复端点（replicate padding）*/
static inline const float *safe_frame(const float mfcc[][SPK_N_MFCC_BASE],
                                       int t, int n_frames)
{
    if (t < 0)        return mfcc[0];
    if (t >= n_frames) return mfcc[n_frames - 1];
    return mfcc[t];
}

/* D=2 中心差分 Delta
 * delta[t][k] = (1*(f[t+1]-f[t-1]) + 2*(f[t+2]-f[t-2])) / 10
 */
static void compute_delta(const float src[][SPK_N_MFCC_BASE], int n_frames,
                           float dst[][SPK_N_MFCC_BASE])
{
    const float denom = 10.0f; /* 2*(1^2 + 2^2) = 10 */
    for (int t = 0; t < n_frames; t++) {
        const float *p1 = safe_frame(src, t + 1, n_frames);
        const float *m1 = safe_frame(src, t - 1, n_frames);
        const float *p2 = safe_frame(src, t + 2, n_frames);
        const float *m2 = safe_frame(src, t - 2, n_frames);
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            dst[t][k] = (1.0f * (p1[k] - m1[k]) + 2.0f * (p2[k] - m2[k])) / denom;
        }
    }
}

/* ========== 公开 API ========== */

void feat_apply_cmn(float mfcc[][SPK_N_MFCC_BASE], int n_frames)
{
    if (n_frames <= 0) return;

    float mean[SPK_N_MFCC_BASE];
    memset(mean, 0, sizeof(mean));

    for (int t = 0; t < n_frames; t++)
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            mean[d] += mfcc[t][d];
    for (int d = 0; d < SPK_N_MFCC_BASE; d++)
        mean[d] /= (float)n_frames;

    for (int t = 0; t < n_frames; t++)
        for (int d = 0; d < SPK_N_MFCC_BASE; d++)
            mfcc[t][d] -= mean[d];

#if SPK_DEBUG
    /* 打印 C01 归一化前均值和归一化后均值（应接近 0）*/
    float check = 0.0f;
    for (int t = 0; t < n_frames; t++) check += mfcc[t][0];
    mprintf("[CMN] C01 mean_before=%d mean_after=%d (should be ~0)\r\n",
            (int)(mean[0] * 1000), (int)(check / n_frames * 1000));
#endif
}

int feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                         float feat_out[][SPK_FEAT_DIM])
{
    if (n_frames <= 0 || n_frames > SPK_MAX_TEMPLATE_FRAMES) return -1;

    /* 计算一阶 Delta，存入 s_delta_buf */
    compute_delta(mfcc, n_frames, s_delta_buf);

    /* 计算二阶 Delta（对 s_delta_buf 再做一次 Delta），直接写入 feat_out 的第三段 */
    /* 先临时用 feat_out 后半部分作为 delta2 存储 */
    for (int t = 0; t < n_frames; t++) {
        const float *dp1 = safe_frame(s_delta_buf, t + 1, n_frames);
        const float *dm1 = safe_frame(s_delta_buf, t - 1, n_frames);
        const float *dp2 = safe_frame(s_delta_buf, t + 2, n_frames);
        const float *dm2 = safe_frame(s_delta_buf, t - 2, n_frames);
        for (int k = 0; k < SPK_N_MFCC_BASE; k++) {
            float dd = (1.0f * (dp1[k] - dm1[k]) + 2.0f * (dp2[k] - dm2[k])) / 10.0f;
            /* 拼接：[MFCC(13) | Delta(13) | DeltaDelta(13)] */
            feat_out[t][k]                        = mfcc[t][k];
            feat_out[t][k + SPK_N_MFCC_BASE]      = s_delta_buf[t][k];
            feat_out[t][k + SPK_N_MFCC_BASE * 2]  = dd;
        }
    }

    return 0;
}