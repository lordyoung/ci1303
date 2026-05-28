#ifndef FEAT_POSTPROC_H
#define FEAT_POSTPROC_H

#include "feat_extract.h"
#include "user_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPK_FEAT_DIM  (SPK_N_MFCC_BASE * 3)   /* 39: MFCC | Delta | DeltaDelta */

void feat_apply_cmn(float mfcc[][SPK_N_MFCC_BASE], int n_frames);

int feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                         float feat_out[][SPK_FEAT_DIM]);

#ifdef __cplusplus
}
#endif

#endif /* FEAT_POSTPROC_H */