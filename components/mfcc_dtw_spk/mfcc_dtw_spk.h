#ifndef _MFCC_DTW_SPK_H_
#define _MFCC_DTW_SPK_H_

#include <stdint.h>

typedef enum {
    SPK_RESULT_ACCEPT      = 0,
    SPK_RESULT_REJECT      = 1,
    SPK_RESULT_NO_TEMPLATE = 2,
    SPK_RESULT_ERROR       = 3,
} spk_result_t;

typedef enum {
    SPK_ENROLL_STATE_RECORDING = 0,
    SPK_ENROLL_STATE_DONE      = 1,
    SPK_ENROLL_STATE_FAILED    = 2,
} spk_enroll_state_t;

typedef void (*spk_verify_cb_t)(spk_result_t result, int dtw_dist_x1000);
typedef void (*spk_enroll_cb_t)(spk_enroll_state_t state, int current, int total);

/* Called once at startup */
int spk_init(spk_verify_cb_t verify_cb);

/* Start enrollment mode; app must say cmd word SPK_ENROLL_TIMES times */
int spk_start_enroll(spk_enroll_cb_t enroll_cb);

/* Erase stored template */
int spk_delete_template(void);

/* Called from vadstart_callback — saves ring buffer position & bounds */
void spk_ringbuf_snapshot(uint32_t vad_start_addr, uint32_t buf_base,
                           uint32_t buf_end, int frm_shift_samples);

/* Called from asr_result_callback — copies PCM from ring buf, signals task */
int spk_process(int n_asr_frames);

#endif