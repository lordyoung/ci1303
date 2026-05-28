#include <math.h>
#include <float.h>
#include "dtw_match.h"
#include "user_config.h"

static float s_row_prev[SPK_MAX_TEMPLATE_FRAMES];
static float s_row_curr[SPK_MAX_TEMPLATE_FRAMES];

static float cosine_dist(const float *a, const float *b, int dim)
{
    float dot = 0.0f, na = 0.0f, nb = 0.0f;
    for (int k = 0; k < dim; k++) {
        dot += a[k] * b[k];
        na  += a[k] * a[k];
        nb  += b[k] * b[k];
    }
    float denom = sqrtf(na) * sqrtf(nb);
    if (denom < 1e-10f) return 1.0f;
    float cos_sim = dot / denom;
    if (cos_sim >  1.0f) cos_sim =  1.0f;
    if (cos_sim < -1.0f) cos_sim = -1.0f;
    return 1.0f - cos_sim;
}

float dtw_distance(const float a[][SPK_FEAT_DIM], int la,
                   const float b[][SPK_FEAT_DIM], int lb,
                   int band_ratio_x100)
{
    if (la <= 0 || lb <= 0) return FLT_MAX;

    int max_len = (la > lb) ? la : lb;
    int R = max_len * band_ratio_x100 / 100;
    if (R < 1) R = 1;

    for (int j = 0; j < lb; j++) s_row_prev[j] = FLT_MAX;
    s_row_prev[0] = cosine_dist(a[0], b[0], SPK_FEAT_DIM);

    for (int i = 1; i < la; i++) {
        for (int j = 0; j < lb; j++) s_row_curr[j] = FLT_MAX;

        int j_lo = i - R; if (j_lo < 0)  j_lo = 0;
        int j_hi = i + R; if (j_hi >= lb) j_hi = lb - 1;

        for (int j = j_lo; j <= j_hi; j++) {
            float d = cosine_dist(a[i], b[j], SPK_FEAT_DIM);
            float best = s_row_prev[j];
            if (j > 0 && s_row_curr[j-1] < best) best = s_row_curr[j-1];
            if (j > 0 && s_row_prev[j-1] < best) best = s_row_prev[j-1];
            s_row_curr[j] = d + (best == FLT_MAX ? FLT_MAX : best);
        }

        for (int j = 0; j < lb; j++) s_row_prev[j] = s_row_curr[j];
    }

    float total = s_row_prev[lb - 1];
    if (total == FLT_MAX) return FLT_MAX;
    return total / (float)(la + lb);
}