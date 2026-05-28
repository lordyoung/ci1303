#ifndef _SPK_TEMPLATE_STORE_H_
#define _SPK_TEMPLATE_STORE_H_

#include <stdint.h>
#include "feat_postproc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Flash 耐久度说明：
 *   模板仅在用户主动注册时写入，验证流程只读不写。
 *   单板生命周期内 Flash 擦写次数 << 10000，无耐久度风险。
 *   若将来加入"自适应模板更新"功能，必须改为 RAM 缓存 + 延迟写。
 */

int spk_template_save(const float tmpl[][SPK_FEAT_DIM], int n_frames);
int spk_template_load(float tmpl[][SPK_FEAT_DIM], int max_frames, int *out_frames);
int spk_template_load_meta(int *out_frames);  /* 仅读元数据，用于启动时探测 */
int spk_template_clear(void);

#ifdef __cplusplus
}
#endif
#endif