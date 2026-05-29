#include <math.h>
#include <string.h>
#include "dtw_match.h"
#include "user_config.h"

/* Rolling two-row DP buffers — static to avoid stack overflow */
static float s_prev[SPK_MAX_TEMPLATE_FRAMES];
static float s_curr[SPK_MAX_TEMPLATE_FRAMES];

static float cosine_dist(const float *a, const float *b, int dim)
{
    float dot = 0.0f, na = 0.0f, nb = 0.0f;
    for (int i = 0; i < dim; i++) {
        dot += a[i] * b[i];
        na  += a[i] * a[i];
        nb  += b[i] * b[i];
    }
    float denom = sqrtf(na) * sqrtf(nb);
    if (denom < 1e-8f) return 1.0f;
    float cosv = dot / denom;
    if (cosv >  1.0f) cosv =  1.0f;
    if (cosv < -1.0f) cosv = -1.0f;
    return 1.0f - cosv;
}

int dtw_match(const float *query, int lq,
              const float *ref,   int lr,
              int dim, int band_pct)
{
    if (lq <= 0 || lr <= 0 ||
        lq > SPK_MAX_TEMPLATE_FRAMES ||
        lr > SPK_MAX_TEMPLATE_FRAMES) return -1;

    const float INF = 1e9f;

    /* Sakoe-Chiba band. Two fixes vs. the naive version:
     *  1) the band is centered on the diagonal that connects (0,0) and
     *     (lq-1, lr-1), i.e. j ≈ i * (lr-1)/(lq-1) — not on j = i. Without
     *     this, when lq and lr differ a lot the window drifts off the valid
     *     j range and the path can never reach the corner (=> INF => -1).
     *  2) the band is widened to at least |lq-lr|+2 so consecutive windows
     *     always overlap and the corner stays reachable. */
    int diff = lq - lr; if (diff < 0) diff = -diff;
    int band = (((lq > lr) ? lq : lr) * band_pct + 50) / 100;
    if (band < diff + 2) band = diff + 2;

    float slope = (lq > 1) ? (float)(lr - 1) / (float)(lq - 1) : 0.0f;

    for (int j = 0; j < lr; j++) s_prev[j] = INF;

    /* first query row (i = 0): center = 0, reached by horizontal moves only */
    {
        int j_hi = band; if (j_hi >= lr) j_hi = lr - 1;
        s_prev[0] = cosine_dist(query, ref, dim);
        for (int j = 1; j <= j_hi; j++) {
            float cost = cosine_dist(query, ref + j * dim, dim);
            s_prev[j]  = s_prev[j - 1] + cost;
        }
    }

    /* remaining rows */
    for (int i = 1; i < lq; i++) {
        const float *qi = query + i * dim;
        for (int j = 0; j < lr; j++) s_curr[j] = INF;

        int center = (int)(i * slope + 0.5f);
        int j_lo = center - band; if (j_lo < 0)    j_lo = 0;
        int j_hi = center + band; if (j_hi >= lr)  j_hi = lr - 1;

        for (int j = j_lo; j <= j_hi; j++) {
            float cost = cosine_dist(qi, ref + j * dim, dim);
            float best = s_prev[j];
            if (j > 0 && s_prev[j - 1] < best) best = s_prev[j - 1];
            if (j > 0 && s_curr[j - 1] < best) best = s_curr[j - 1];
            s_curr[j] = best + cost;
        }
        memcpy(s_prev, s_curr, (size_t)(lr * sizeof(float)));
    }

    float dist = s_prev[lr - 1];
    if (dist >= 1e8f) return -1;
    return (int)(dist / (float)(lq + lr) * 1000.0f);
}