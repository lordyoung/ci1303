#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mfcc_dtw_spk.h"
#include "feat_extract.h"
#include "feat_postproc.h"
#include "dtw_match.h"
#include "spk_template_store.h"
#include "user_config.h"
#include "ci_log.h"

/* ── types ─────────────────────────────────────────────────────────────────── */

typedef enum { SPK_ST_IDLE=0, SPK_ST_ENROLL, SPK_ST_VERIFY } spk_state_t;
typedef struct { int n_pcm_samples; } spk_msg_t;

/* ── large static buffers (BSS, not stack) ─────────────────────────────────── */

static short s_ring[SPK_PCM_BUF_SAMPLES];     /* capture ring buffer (fed every frame) */
static short s_pcm_copy[SPK_PCM_BUF_SAMPLES]; /* linear snapshot handed to spk_task    */
static float s_mfcc_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];
static float s_feat_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];
static float s_template[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];

/* ── ring buffer write state (written from BNPU frame callback context) ────── */

static volatile uint32_t s_ring_w     = 0;    /* next write index [0, SPK_PCM_BUF_SAMPLES) */
static volatile uint32_t s_ring_count = 0;    /* total samples buffered, saturates at cap  */
static int               s_frm_shift  = 160;  /* ASR FE frame shift in samples             */
static int               s_feed_dbg   = 0;

/* ── state ─────────────────────────────────────────────────────────────────── */

static spk_state_t     s_state           = SPK_ST_IDLE;
static int             s_enroll_cnt      = 0;
static int             s_template_frames = 0;
static spk_verify_cb_t s_verify_cb       = NULL;
static spk_enroll_cb_t s_enroll_cb       = NULL;
static QueueHandle_t   s_queue           = NULL;

/* ── PCM capture: called every audio frame from audio_deal_one_frm_callback ── */

void spk_feed_pcm(const short *pcm, int n)
{
    if (!pcm || n <= 0 || s_state == SPK_ST_IDLE)
        return;

    if (s_feed_dbg < 5) {
        mprintf("[SPK] feed pcm addr=0x%x n=%d\n",
                (unsigned)(uintptr_t)pcm, n);
        s_feed_dbg++;
    }

    /* If a single frame somehow exceeds the buffer, keep only its tail */
    if (n > (int)SPK_PCM_BUF_SAMPLES) {
        pcm += (n - (int)SPK_PCM_BUF_SAMPLES);
        n    = (int)SPK_PCM_BUF_SAMPLES;
    }

    uint32_t w     = s_ring_w;
    int      first = (int)(SPK_PCM_BUF_SAMPLES - w);
    if (first > n) first = n;
    memcpy(&s_ring[w], pcm, (size_t)first * sizeof(short));
    int rem = n - first;
    if (rem > 0)
        memcpy(&s_ring[0], pcm + first, (size_t)rem * sizeof(short));

    w += (uint32_t)n;
    if (w >= SPK_PCM_BUF_SAMPLES) w -= SPK_PCM_BUF_SAMPLES;
    s_ring_w = w;

    uint32_t c = s_ring_count + (uint32_t)n;
    s_ring_count = (c > SPK_PCM_BUF_SAMPLES) ? SPK_PCM_BUF_SAMPLES : c;
}

/* Back-compat shim: old VAD callbacks still call this. We no longer rely on the
 * ASR ring buffer bounds (unavailable in pure command-word mode); we only pick
 * up the frame shift if a valid one is offered. */
void spk_ringbuf_snapshot(uint32_t vad_start, uint32_t buf_base,
                           uint32_t buf_end, int frm_shift)
{
    (void)vad_start; (void)buf_base; (void)buf_end;
    if (frm_shift > 0) s_frm_shift = frm_shift;
}

/* ── utterance processing (runs in spk_task) ───────────────────────────────── */

static void process_utterance(int n_pcm_samples)
{
    /* 1. MFCC */
    int n_feat = feat_extract_mfcc(s_pcm_copy, n_pcm_samples,
                                   s_mfcc_buf, SPK_MAX_TEMPLATE_FRAMES);
    if (n_feat < 4) {
        mprintf("[SPK] too few frames: %d\n", n_feat);
        if (s_state == SPK_ST_ENROLL && s_enroll_cb)
            s_enroll_cb(SPK_ENROLL_STATE_FAILED, s_enroll_cnt, SPK_ENROLL_TIMES);
        return;
    }

    /* 2. CMN + delta → 39-dim */
    feat_apply_cmn(s_mfcc_buf, n_feat);
    feat_pack_with_delta((const float (*)[SPK_N_MFCC_BASE])s_mfcc_buf,
                         n_feat, s_feat_buf);

    if (s_state == SPK_ST_ENROLL) {
        /* Running average into s_template */
        if (s_enroll_cnt == 0) {
            memcpy(s_template, s_feat_buf,
                   (size_t)(n_feat * SPK_FEAT_DIM * sizeof(float)));
            s_template_frames = n_feat;
        } else {
            int ref = s_template_frames;
            for (int t = 0; t < ref; t++) {
                int ti = t * n_feat / ref;
                if (ti >= n_feat) ti = n_feat - 1;
                for (int d = 0; d < SPK_FEAT_DIM; d++) {
                    s_template[t][d] = (s_template[t][d] * (float)s_enroll_cnt
                                        + s_feat_buf[ti][d]) / (float)(s_enroll_cnt + 1);
                }
            }
        }
        s_enroll_cnt++;
        mprintf("[SPK] enrolled %d/%d (feat_frames=%d)\n",
                s_enroll_cnt, SPK_ENROLL_TIMES, n_feat);

        if (s_enroll_cnt < SPK_ENROLL_TIMES) {
            if (s_enroll_cb)
                s_enroll_cb(SPK_ENROLL_STATE_RECORDING, s_enroll_cnt, SPK_ENROLL_TIMES);
        } else {
            spk_tpl_save(s_template, s_template_frames);
            s_state      = SPK_ST_VERIFY;
            s_enroll_cnt = 0;
            if (s_enroll_cb)
                s_enroll_cb(SPK_ENROLL_STATE_DONE, SPK_ENROLL_TIMES, SPK_ENROLL_TIMES);
        }

    } else if (s_state == SPK_ST_VERIFY) {
        /* Load template from flash if not cached */
        if (s_template_frames == 0) {
            if (spk_tpl_load(s_template, &s_template_frames) != 0) {
                mprintf("[SPK] no template\n");
                if (s_verify_cb) s_verify_cb(SPK_RESULT_NO_TEMPLATE, 0);
                return;
            }
        }
        int dist = dtw_match((const float *)s_feat_buf, n_feat,
                             (const float *)s_template, s_template_frames,
                             SPK_FEAT_DIM, SPK_DTW_BAND_RATIO_X100);
        mprintf("[SPK] DTW dist*1000=%d thr=%d -> %s\n",
                dist, SPK_DTW_THRESHOLD_X1000,
                (dist >= 0 && dist <= SPK_DTW_THRESHOLD_X1000) ? "ACCEPT" : "REJECT");

        spk_result_t r = (dist >= 0 && dist <= SPK_DTW_THRESHOLD_X1000)
                         ? SPK_RESULT_ACCEPT : SPK_RESULT_REJECT;
        if (s_verify_cb) s_verify_cb(r, dist);
    }
}

/* ── FreeRTOS task ─────────────────────────────────────────────────────────── */

static void spk_task(void *p)
{
    (void)p;
    spk_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_queue, &msg, portMAX_DELAY) == pdTRUE)
            process_utterance(msg.n_pcm_samples);
    }
}

/* ── public API ────────────────────────────────────────────────────────────── */

int spk_init(spk_verify_cb_t verify_cb)
{
    feat_init();
    s_verify_cb = verify_cb;

    /* Pull the real ASR frame shift from the ASR system if available */
    extern int get_asrtop_asrfrmshift(void);
    int fs = get_asrtop_asrfrmshift();
    if (fs > 0) s_frm_shift = fs;
    mprintf("[SPK] frm_shift=%d\n", s_frm_shift);

    if (spk_tpl_load(s_template, &s_template_frames) == 0)
        mprintf("[SPK] template loaded: %d frames\n", s_template_frames);
    else
        mprintf("[SPK] no stored template\n");

    s_queue = xQueueCreate(4, sizeof(spk_msg_t));
    xTaskCreate(spk_task, "spk", 1024, NULL, 3, NULL);

    /* Start capturing PCM right away */
    s_state = SPK_ST_VERIFY;
    return 0;
}

int spk_start_enroll(spk_enroll_cb_t enroll_cb)
{
    s_enroll_cb       = enroll_cb;
    s_enroll_cnt      = 0;
    s_template_frames = 0;
    s_state           = SPK_ST_ENROLL;
    mprintf("[SPK] enroll started - say cmd word %d times\n", SPK_ENROLL_TIMES);
    return 0;
}

int spk_delete_template(void)
{
    spk_tpl_delete();
    s_template_frames = 0;
    s_state           = SPK_ST_VERIFY;
    mprintf("[SPK] template deleted\n");
    return 0;
}

int spk_process(int n_asr_frames)
{
    if (n_asr_frames <= 0 || s_state == SPK_ST_IDLE || !s_queue) return -1;

    int n_samples = n_asr_frames * s_frm_shift;
    if (n_samples > (int)SPK_PCM_BUF_SAMPLES)
        n_samples = (int)SPK_PCM_BUF_SAMPLES;

    /* Snapshot the current write head and how much is buffered */
    uint32_t w     = s_ring_w;
    uint32_t avail = s_ring_count;
    if ((uint32_t)n_samples > avail)
        n_samples = (int)avail;

    mprintf("[SPK] process frm=%d shift=%d -> %d samples (avail=%u)\n",
            n_asr_frames, s_frm_shift, n_samples, (unsigned)avail);

    if (n_samples < SPK_FRAME_LEN) {
        mprintf("[SPK] not enough pcm captured (%d)\n", n_samples);
        return -1;
    }

    /* Copy the most recent n_samples (in time order) out of the ring buffer */
    int start = (int)w - n_samples;
    if (start < 0) start += (int)SPK_PCM_BUF_SAMPLES;
    int first = (int)SPK_PCM_BUF_SAMPLES - start;
    if (first > n_samples) first = n_samples;
    memcpy(s_pcm_copy, &s_ring[start], (size_t)first * sizeof(short));
    if (n_samples > first)
        memcpy(s_pcm_copy + first, &s_ring[0],
               (size_t)(n_samples - first) * sizeof(short));

    spk_msg_t msg = { n_samples };
    if (xQueueSend(s_queue, &msg, 0) != pdTRUE) {
        mprintf("[SPK] queue full\n");
        return -1;
    }
    return 0;
}