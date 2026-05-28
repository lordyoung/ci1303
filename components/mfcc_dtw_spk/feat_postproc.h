#ifndef _FEAT_POSTPROC_H_
#define _FEAT_POSTPROC_H_

#include "feat_extract.h"

#define SPK_FEAT_DIM   (SPK_N_MFCC_BASE * 3)   /* 39 = MFCC + Δ + ΔΔ */

/* Subtract per-dimension utterance mean in-place (CMN) */
void feat_apply_cmn(float feats[][SPK_N_MFCC_BASE], int n_frames);

/* Pack [MFCC | Δ | ΔΔ] into out[n_frames][39] */
void feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                          float out[][SPK_FEAT_DIM]);

#endif