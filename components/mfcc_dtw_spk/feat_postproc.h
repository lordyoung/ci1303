#ifndef _SPK_FEAT_POSTPROC_H_
#define _SPK_FEAT_POSTPROC_H_

#include <stdint.h>
#include "feat_extract.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPK_FEAT_DIM   (SPK_N_MFCC_BASE * 3)   /* MFCC + Delta + DeltaDelta = 39 */

void feat_apply_cmn(float mfcc[][SPK_N_MFCC_BASE], int n_frames);
int  feat_pack_with_delta(const float mfcc[][SPK_N_MFCC_BASE], int n_frames,
                          float feat_out[][SPK_FEAT_DIM]);

#ifdef __cplusplus
}
#endif
#endif