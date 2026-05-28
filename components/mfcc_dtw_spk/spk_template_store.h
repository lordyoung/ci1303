#ifndef SPK_TEMPLATE_STORE_H
#define SPK_TEMPLATE_STORE_H

#include <stdint.h>
#include "feat_postproc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPK_TEMPLATE_MAGIC  0x53504B31u  /* 'SPK1' */

int spk_template_load(float feat[][SPK_FEAT_DIM], int *n_frames);
int spk_template_save(const float feat[][SPK_FEAT_DIM], int n_frames);
int spk_template_delete(void);

#ifdef __cplusplus
}
#endif

#endif /* SPK_TEMPLATE_STORE_H */