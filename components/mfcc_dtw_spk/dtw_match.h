#ifndef _DTW_MATCH_H_
#define _DTW_MATCH_H_

/* DTW with Sakoe-Chiba band.
 * query/ref: row-major float arrays, each row is dim floats.
 * band_pct : half-band as % of ref length (e.g. 20 → ±20%).
 * Returns normalized DTW distance × 1000, or -1 on error. */
int dtw_match(const float *query, int lq,
              const float *ref,   int lr,
              int dim, int band_pct);

#endif