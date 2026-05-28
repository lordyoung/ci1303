#ifndef _SPK_TEMPLATE_STORE_H_
#define _SPK_TEMPLATE_STORE_H_

#include "feat_postproc.h"

int spk_tpl_save(const float feats[][SPK_FEAT_DIM], int n_frames);
int spk_tpl_load(float feats[][SPK_FEAT_DIM], int *n_frames_out);
int spk_tpl_exists(void);
int spk_tpl_delete(void);

#endif