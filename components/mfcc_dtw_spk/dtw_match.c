/**
 * @file dtw_match.c
 * @brief M1 stub. Real implementation in M3.
 */
#include "dtw_match.h"
#include "ci_log.h"

float dtw_distance(const float a[][SPK_FEAT_DIM], int la,
                   const float b[][SPK_FEAT_DIM], int lb,
                   int band_ratio_x100)
{
    (void)a; (void)b; (void)la; (void)lb; (void)band_ratio_x100;
    mprintf("[DTW] stub\r\n");
    return 999.0f;
}