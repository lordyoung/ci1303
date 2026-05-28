#ifndef DTW_MATCH_H
#define DTW_MATCH_H

#include "feat_postproc.h"

#ifdef __cplusplus
extern "C" {
#endif

float dtw_distance(const float a[][SPK_FEAT_DIM], int la,
                   const float b[][SPK_FEAT_DIM], int lb,
                   int band_ratio_x100);

#ifdef __cplusplus
}
#endif

#endif /* DTW_MATCH_H */