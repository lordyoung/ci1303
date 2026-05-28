/**
 * @file dtw_match.c
 * @brief Dynamic Time Warping with Sakoe-Chiba band, cosine frame distance
 *        Two-row rolling array to save memory.
 */
#include <math.h>
#include <float.h>
#include "dtw_match.h"
#include "user_config.h"

#define DTW_INF   1e10f

/* 滚动行缓冲：预留最大模板长度 */
static float s_row_prev[SPK_MAX_TEMPLATE_FRAMES];
static float s_row_curr[SPK_MAX_TEMPLATE_FRAMES];

/* 余弦距离：返回 1 - cos(a,b)，范围 [0, 2]，0 = 完全相同 */
static float frame_cos_dist(const float *a, const float *b, int dim)
{
    float dot = 0.0f, na = 0.0f, nb = 0.0f;
    for (int k = 0; k < dim; k++) {
        dot += a[k] * b[k];
        na  += a[k] * a[k];
        nb  += b[k] * b[k];
    }
    float denom = sqrtf(na * nb);
    if (denom < 1e-10f) return 1.0f;
    float cos_sim = dot / denom;
    if (cos_sim >  1.0f) cos_sim =  1.0f;
    if (cos_sim < -1.0f) cos_sim = -1.0f;
    return 1.0f - cos_sim;
}

/* Sakoe-Chiba 带约束 DTW，返回归一化路径距离 */
float dtw_distance(const float a[][SPK_FEAT_DIM], int la,
                   const float b[][SPK_FEAT_DIM], int lb,
                   int band_ratio_x100)
{
    if (la <= 0 || lb <= 0) return DTW_INF;
    if (la > SPK_MAX_TEMPLATE_FRAMES || lb > SPK_MAX_TEMPLATE_FRAMES)
        return DTW_INF;

    /* Sakoe-Chiba 带宽 */
    int max_len = (la > lb) ? la : lb;
    int R = max_len * band_ratio_x100 / 100;
    if (R < 1) R = 1;

    /* 初始化 prev 行为 INF */
    for (int j = 0; j < lb; j++) s_row_prev[j] = DTW_INF;

    for (int i = 0; i < la; i++) {
        /* 当前行 j 范围：[max(0,i-R), min(lb-1,i+R)] */
        int j_lo = i - R; if (j_lo < 0)    j_lo = 0;
        int j_hi = i + R; if (j_hi >= lb)  j_hi = lb - 1;

        /* 把不在 band 内的 curr 设为 INF */
        for (int j = 0; j < lb; j++) s_row_curr[j] = DTW_INF;

        for (int j = j_lo; j <= j_hi; j++) {
            float d = frame_cos_dist(a[i], b[j], SPK_FEAT_DIM);

            if (i == 0 && j == 0) {
                s_row_curr[j] = d;
            } else {
                float best = DTW_INF;
                /* 来自 (i-1, j-1) */
                if (i > 0 && j > 0 && s_row_prev[j - 1] < best)
                    best = s_row_prev[j - 1];
                /* 来自 (i-1, j) */
                if (i > 0 && s_row_prev[j] < best)
                    best = s_row_prev[j];
                /* 来自 (i, j-1) */
                if (j > 0 && s_row_curr[j - 1] < best)
                    best = s_row_curr[j - 1];

                s_row_curr[j] = d + best;
            }
        }

        /* 滚动：curr → prev */
        for (int j = 0; j < lb; j++) s_row_prev[j] = s_row_curr[j];
    }

    float total = s_row_prev[lb - 1];
    if (total >= DTW_INF) return DTW_INF;

    /* 归一化：除以路径"对角长度估计"= la + lb */
    return total / (float)(la + lb);
}