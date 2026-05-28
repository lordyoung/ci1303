#ifndef _SPK_FEAT_EXTRACT_H_
#define _SPK_FEAT_EXTRACT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPK_FFT_N           512
#define SPK_FRAME_LEN       512
#define SPK_FRAME_SHIFT     256
#define SPK_N_MEL           26
#define SPK_N_FFT_BINS      257     /* N/2 + 1 */
#define SPK_N_MFCC_BASE     13      /* C1..C13 */

/* 模块初始化：预计算 Hamming 窗、Mel 滤波器组、DCT 系数表
 * 在 spk_init() 中调用一次即可 */
void feat_init(void);

/* 从 PCM 序列提取 MFCC 帧序列
 *   pcm        : int16 PCM 数据，16kHz 单声道
 *   n_samples  : PCM 采样点总数
 *   mfcc_out   : 输出 [n_frames][SPK_N_MFCC_BASE]，由调用者分配
 *   max_frames : mfcc_out 最多能接受的帧数
 *   out_frames : 实际输出帧数
 * 返回 0 成功，-1 失败（帧数不足）
 */
int feat_extract_mfcc(const short *pcm, int n_samples,
                      float mfcc_out[][SPK_N_MFCC_BASE],
                      int max_frames, int *out_frames);

#ifdef __cplusplus
}
#endif
#endif