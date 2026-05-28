#ifndef _SPK_DTW_MATCH_H_
#define _SPK_DTW_MATCH_H_

#include "feat_postproc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 返回 DTW 归一化距离（值越小越相似） */
float dtw_distance(const float a[][SPK_FEAT_DIM], int la,
                   const float b[][SPK_FEAT_DIM], int lb,
                   int band_ratio_x100);

#ifdef __cplusplus
}
#endif
#endif