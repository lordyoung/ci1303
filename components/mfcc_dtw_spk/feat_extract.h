#ifndef FEAT_EXTRACT_H
#define FEAT_EXTRACT_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPK_SAMPLE_RATE     16000
#define SPK_FFT_N           512
#define SPK_FRAME_LEN       512
#define SPK_FRAME_SHIFT     256
#define SPK_N_MEL           26
#define SPK_N_FFT_BINS      257   /* FFT_N/2 + 1 */
#define SPK_N_MFCC_BASE     13    /* C1 .. C13 */

void feat_init(void);

int feat_extract_mfcc(const short *pcm, int n_samples,
                      float mfcc_out[][SPK_N_MFCC_BASE],
                      int max_frames, int *out_frames);

#ifdef __cplusplus
}
#endif

#endif /* FEAT_EXTRACT_H */