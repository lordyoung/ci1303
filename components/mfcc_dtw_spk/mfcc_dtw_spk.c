#include <string.h>
#include <stdint.h>
#include "mfcc_dtw_spk.h"
#include "feat_extract.h"
#include "feat_postproc.h"
#include "dtw_match.h"
#include "spk_template_store.h"
#include "user_config.h"
#include "ci_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#if USE_MFCC_DTW_SPK

int get_asrtop_asrfrmshift(void)
{
    return 160; /* 16kHz, 10ms frame shift */
}

typedef enum {
    SPK_STATE_IDLE      = 0,
    SPK_STATE_ENROLLING = 1,
    SPK_STATE_READY     = 2
} spk_state_t;

static spk_state_t     s_state      = SPK_STATE_IDLE;
static spk_verify_cb_t s_verify_cb  = NULL;
static spk_enroll_cb_t s_enroll_cb  = NULL;
static int             s_enroll_cnt = 0;

/* ---- static buffers ---- */
static short s_pcm_copy[SPK_PCM_BUF_SIZE / sizeof(short)];  /* 32KB */

static float s_mfcc_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];
static float s_feat_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];

/* s_template serves double duty: enrollment running-average + verification template */
static float s_template[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];
static int   s_template_frames = 0;

/* ---- helpers ---- */

static int process_pcm(const short *pcm, int n_samples,
                       float feat_out[][SPK_FEAT_DIM], int *out_frames)
{
    int n_mfcc = 0;
    if (feat_extract_mfcc(pcm, n_samples, s_mfcc_buf,
                          SPK_MAX_TEMPLATE_FRAMES, &n_mfcc) != 0) {
        mprintf("[SPK] MFCC extraction failed\n");
        return -1;
    }
    feat_apply_cmn(s_mfcc_buf, n_mfcc);
    *out_frames = feat_pack_with_delta(s_mfcc_buf, n_mfcc, feat_out);
    return 0;
}

/* ---- message queue / task ---- */

typedef struct {
    uint32_t pcm_base_addr;
    int      voice_start_frame;
    int      valid_frame_len;
} spk_msg_t;

static QueueHandle_t s_spk_queue = NULL;

static void spk_task(void *arg)
{
    spk_msg_t msg;
    mprintf("[SPK] task started, state=%d\n", (int)s_state);

    for (;;) {
        if (xQueueReceive(s_spk_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        int frame_shift = get_asrtop_asrfrmshift();
        int n_samples   = msg.valid_frame_len * frame_shift;

        if (n_samples > (int)(SPK_PCM_BUF_SIZE / sizeof(short))) {
            n_samples = (int)(SPK_PCM_BUF_SIZE / sizeof(short));
        }

        uint32_t pcm_start = msg.pcm_base_addr
            + (uint32_t)(msg.voice_start_frame * frame_shift * sizeof(short));
        memcpy(s_pcm_copy, (const void *)pcm_start, (size_t)n_samples * sizeof(short));

        if (s_state == SPK_STATE_READY) {
            /* ---------- verification ---------- */
            int n_feat = 0;
            if (process_pcm(s_pcm_copy, n_samples, s_feat_buf, &n_feat) != 0) {
                if (s_verify_cb) s_verify_cb(SPK_RESULT_ERROR, 0);
                continue;
            }
            float dist = dtw_distance(s_feat_buf, n_feat,
                                      s_template, s_template_frames,
                                      SPK_DTW_BAND_RATIO_X100);
            int dist_x1000 = (int)(dist * 1000.0f);
            int thr_x1000  = SPK_DTW_THRESHOLD_X1000;
            mprintf("[SPK] DTW dist=%d thr=%d -> %s\n",
                    dist_x1000, thr_x1000,
                    dist_x1000 < thr_x1000 ? "ACCEPT" : "REJECT");
            spk_result_t result = (dist_x1000 < thr_x1000)
                                  ? SPK_RESULT_ACCEPT : SPK_RESULT_REJECT;
            if (s_verify_cb) s_verify_cb(result, dist_x1000);

        } else if (s_state == SPK_STATE_ENROLLING) {
            /* ---------- enrollment: in-place running average ---------- */
            int n_feat = 0;
            if (process_pcm(s_pcm_copy, n_samples, s_feat_buf, &n_feat) != 0) {
                mprintf("[SPK] Enroll sample %d failed\n", s_enroll_cnt + 1);
                if (s_enroll_cb) s_enroll_cb(SPK_ENROLL_FAIL,
                                              s_enroll_cnt, SPK_ENROLL_TIMES);
                continue;
            }

            if (s_enroll_cnt == 0) {
                /* first sample: copy feat_buf → template */
                if (n_feat > SPK_MAX_TEMPLATE_FRAMES) n_feat = SPK_MAX_TEMPLATE_FRAMES;
                memcpy(s_template, s_feat_buf,
                       (size_t)n_feat * SPK_FEAT_DIM * sizeof(float));
                s_template_frames = n_feat;
            } else {
                /* subsequent samples: remap to template length, update running mean */
                int ref = s_template_frames;
                for (int t = 0; t < ref; t++) {
                    int ti = t * n_feat / ref;
                    if (ti >= n_feat) ti = n_feat - 1;
                    for (int d = 0; d < SPK_FEAT_DIM; d++) {
                        s_template[t][d] =
                            (s_template[t][d] * (float)s_enroll_cnt
                             + s_feat_buf[ti][d])
                            / (float)(s_enroll_cnt + 1);
                    }
                }
            }

            mprintf("[SPK] Enrolled sample %d/%d (%d frames)\n",
                    s_enroll_cnt + 1, SPK_ENROLL_TIMES, n_feat);
            if (s_enroll_cb) s_enroll_cb(SPK_ENROLL_PROGRESS,
                                          s_enroll_cnt + 1, SPK_ENROLL_TIMES);
            s_enroll_cnt++;

            if (s_enroll_cnt >= SPK_ENROLL_TIMES) {
                mprintf("[SPK] Template ready: %d frames (avg of %d samples)\n",
                        s_template_frames, SPK_ENROLL_TIMES);
                if (spk_template_save(s_template, s_template_frames) == 0) {
                    s_state = SPK_STATE_READY;
                    mprintf("[SPK] Enrollment complete, template saved\n");
                    if (s_enroll_cb) s_enroll_cb(SPK_ENROLL_DONE,
                                                  SPK_ENROLL_TIMES, SPK_ENROLL_TIMES);
                } else {
                    mprintf("[SPK] Template save failed, re-enrolling\n");
                    s_enroll_cnt = 0;
                    if (s_enroll_cb) s_enroll_cb(SPK_ENROLL_FAIL,
                                                  0, SPK_ENROLL_TIMES);
                }
            }
        }
    }
}

/* ---- Public API ---- */

int spk_init(spk_verify_cb_t verify_cb)
{
    mprintf("[SPK] spk_init called\n");
    s_verify_cb  = verify_cb;
    s_enroll_cb  = NULL;
    s_enroll_cnt = 0;

    feat_init();
    mprintf("[SPK] feat_init done\n");

    if (spk_template_load(s_template, &s_template_frames) == 0) {
        s_state = SPK_STATE_READY;
        mprintf("[SPK] Template found in Flash, state=READY\n");
    } else {
        s_state = SPK_STATE_ENROLLING;
        mprintf("[SPK] No template, state=ENROLLING (say cmd %d times)\n",
                SPK_ENROLL_TIMES);
    }

    s_spk_queue = xQueueCreate(4, sizeof(spk_msg_t));
    if (!s_spk_queue) {
        mprintf("[SPK] Queue create failed!\n");
        return -1;
    }
    xTaskCreate(spk_task, "spk task", 1024, NULL, 3, NULL);
    mprintf("[SPK] spk task created\n");
    return 0;
}

int spk_start_enroll(spk_enroll_cb_t enroll_cb)
{
    s_enroll_cb  = enroll_cb;
    s_enroll_cnt = 0;
    s_state      = SPK_STATE_ENROLLING;
    mprintf("[SPK] Re-enrollment started\n");
    return 0;
}

int spk_verify(uint32_t pcm_base_addr, int voice_start_frame, int valid_frame_len)
{
    if (!s_spk_queue) return -1;
    if (s_state == SPK_STATE_IDLE) return -1;

    spk_msg_t msg;
    msg.pcm_base_addr     = pcm_base_addr;
    msg.voice_start_frame = voice_start_frame;
    msg.valid_frame_len   = valid_frame_len;

    mprintf("[ASR->SPK] frm_start=%d len=%d\n", voice_start_frame, valid_frame_len);
    if (xQueueSend(s_spk_queue, &msg, 0) != pdTRUE) {
        mprintf("[SPK] Queue full, dropped\n");
        return -1;
    }
    return 0;
}

int spk_delete_template(void)
{
    int ret = spk_template_delete();
    if (ret == 0) {
        s_state      = SPK_STATE_ENROLLING;
        s_enroll_cnt = 0;
        mprintf("[SPK] Template deleted, state=ENROLLING\n");
    }
    return ret;
}

#endif /* USE_MFCC_DTW_SPK */