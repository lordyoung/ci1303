/**
 * @file mfcc_dtw_spk.h
 * @brief MFCC + DTW based speaker recognition for fixed command word
 *        Target command: "小屁开门"
 */
#ifndef _MFCC_DTW_SPK_H_
#define _MFCC_DTW_SPK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 验证结果 ===== */
typedef enum {
    SPK_RESULT_ACCEPT = 0,      /* 通过：目标说话人 */
    SPK_RESULT_REJECT = 1,      /* 拒绝：非目标说话人 */
    SPK_RESULT_NO_TEMPLATE = 2, /* 无模板：未注册 */
    SPK_RESULT_ERROR = 3        /* 错误 */
} spk_result_t;

/* ===== 注册阶段结果 ===== */
typedef enum {
    SPK_ENROLL_PROGRESS = 0,    /* 进行中（已收一条样本） */
    SPK_ENROLL_DONE     = 1,    /* 完成，模板已写入 Flash */
    SPK_ENROLL_FAIL     = 2     /* 失败：样本一致性差 */
} spk_enroll_state_t;

/* ===== 回调类型 ===== */
typedef void (*spk_verify_cb_t)(spk_result_t result, int dtw_dist_x1000);
typedef void (*spk_enroll_cb_t)(spk_enroll_state_t state, int current, int total);

/* ===== 公开 API ===== */
int  spk_init(spk_verify_cb_t verify_cb);
int  spk_start_enroll(spk_enroll_cb_t enroll_cb);
int  spk_verify(uint32_t pcm_base_addr, int voice_start_frame, int valid_frame_len);
int  spk_delete_template(void);

#ifdef __cplusplus
}
#endif
#endif /* _MFCC_DTW_SPK_H_ */