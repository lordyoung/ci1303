#ifndef _FEAT_EXTRACT_H_
#define _FEAT_EXTRACT_H_

#include "user_config.h"

#define SPK_SAMPLE_RATE     16000
#define SPK_FFT_N           512
#define SPK_FRAME_LEN       512
#define SPK_FRAME_SHIFT     256
#define SPK_N_MEL           40
#define SPK_N_FFT_BINS      (SPK_FFT_N / 2)   /* 256 complex bins */
#define SPK_N_MFCC_BASE     20

#define SPK_PCM_BUF_SAMPLES (SPK_PCM_BUF_SIZE / sizeof(short))

int feat_init(void);

/* Extract 13-dim MFCC into out[n_max_frames][13].
 * Returns number of frames extracted. */
int feat_extract_mfcc(const short *pcm, int n_samples,
                      float out[][SPK_N_MFCC_BASE], int n_max_frames);

#endif