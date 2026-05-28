/**
 * @file mfcc_dtw_spk.c
 * @brief Full implementation — state machine, enrollment, verification.
 *
 * Auto-enrollment policy:
 *   - On boot, if no template in flash, enter ENROLLING state.
 *   - User says "小屁开门" SPK_ENROLL_TIMES (3) times → template saved.
 *   - Subsequent recognitions enter VERIFY state.
 *
 * To re-enroll: call spk_delete_template() then reboot, or call spk_start_enroll().
 */
#include <string.h>
#include <math.h>
#include "mfcc_dtw_spk.h"
#include "feat_extract.h"
#include "feat_postproc.h"
#include "dtw_match.h"
#include "spk_template_store.h"
#include "user_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ci_log.h"

extern int get_asrtop_asrfrmshift(void);

/* ===== 消息 ===== */
typedef struct {
    int n_samples;        /* PCM 副本中有效样本数 */
} spk_msg_t;

/* ===== 上下文 ===== */
typedef enum {
    SPK_STATE_IDLE = 0,
    SPK_STATE_ENROLLING,
    SPK_STATE_VERIFYING
} spk_state_t;

typedef struct {
    spk_state_t       state;
    spk_verify_cb_t   verify_cb;
    int               template_loaded;
    int               template_frm_num;
    int               enroll_count;
} spk_ctx_t;

static spk_ctx_t      g_spk_ctx     = { 0 };
static QueueHandle_t  g_spk_queue   = NULL;
static TaskHandle_t   g_spk_task_h  = NULL;

/* ===== 大缓冲：全部静态，避免栈溢出 ===== */

/* PCM 副本（同步 memcpy 自 ASR 缓冲，避免被覆盖）*/
static short s_pcm_copy[SPK_PCM_BUF_SIZE / sizeof(short)];   /* 48KB */

/* 当前 utterance 的中间结果 */
static float s_mfcc_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_N_MFCC_BASE];
static float s_feat_buf[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];
static int   s_feat_n_frames;

/* 注册时三次样本 */
static float s_enroll_feats[SPK_ENROLL_TIMES][SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];
static int   s_enroll_frm_nums[SPK_ENROLL_TIMES];

/* 已激活的模板（运行时从 Flash 加载或注册后写入）*/
static float s_template[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];

/* ===== 内部函数声明 ===== */
static void spk_task(void *arg);
static void process_enroll_sample(int sample_idx);
static void process_verify(void);
static int  compute_average_template(void);

/* ===== 工具：估算 PCM 能量（用于检测静音）===== */
static int compute_avg_energy(const short *pcm, int n)
{
    if (n <= 0) return 0;
    int64_t sum = 0;
    for (int i = 0; i < n; i++) {
        int32_t v = pcm[i];
        sum += (v * v) / 1000;
    }
    return (int)(sum / n);
}

/* ===== 公开 API ===== */

int spk_init(spk_verify_cb_t verify_cb)
{
    mprintf("[SPK] init start\r\n");

    feat_init();
    mprintf("[SPK] feat_init done\r\n");

    g_spk_ctx.state            = SPK_STATE_IDLE;
    g_spk_ctx.verify_cb        = verify_cb;
    g_spk_ctx.template_loaded  = 0;
    g_spk_ctx.template_frm_num = 0;
    g_spk_ctx.enroll_count     = 0;

    g_spk_queue = xQueueCreate(2, sizeof(spk_msg_t));
    if (g_spk_queue == NULL) {
        mprintf("[SPK] ERROR: queue create failed\r\n");
        return -1;
    }
    mprintf("[SPK] queue created ok\r\n");

    BaseType_t ret = xTaskCreate(spk_task, "spk", 1024, NULL, 3, &g_spk_task_h);
    if (ret != pdPASS) {
        mprintf("[SPK] ERROR: task create failed ret=%d\r\n", (int)ret);
        return -1;
    }
    mprintf("[SPK] task created ok\r\n");

    /* 启动时加载模板 */
    int loaded_frm = 0;
    int load_ret = spk_template_load(s_template, SPK_MAX_TEMPLATE_FRAMES, &loaded_frm);
    if (load_ret == 0 && loaded_frm > 0) {
        g_spk_ctx.template_loaded  = 1;
        g_spk_ctx.template_frm_num = loaded_frm;
        g_spk_ctx.state            = SPK_STATE_VERIFYING;
        mprintf("[SPK] template loaded, frm=%d -> VERIFY mode\r\n", loaded_frm);
    } else {
        g_spk_ctx.state        = SPK_STATE_ENROLLING;
        g_spk_ctx.enroll_count = 0;
        mprintf("[SPK] no template -> ENROLL mode (say command %d times)\r\n",
                SPK_ENROLL_TIMES);
    }

    mprintf("[SPK] init done state=%d\r\n", (int)g_spk_ctx.state);
    return 0;
}

int spk_start_enroll(spk_enroll_cb_t enroll_cb)
{
    (void)enroll_cb;
    g_spk_ctx.state        = SPK_STATE_ENROLLING;
    g_spk_ctx.enroll_count = 0;
    mprintf("[SPK] enroll restarted (say command %d times)\r\n", SPK_ENROLL_TIMES);
    return 0;
}

int spk_delete_template(void)
{
    spk_template_clear();
    g_spk_ctx.template_loaded  = 0;
    g_spk_ctx.template_frm_num = 0;
    g_spk_ctx.state            = SPK_STATE_ENROLLING;
    g_spk_ctx.enroll_count     = 0;
    mprintf("[SPK] template deleted, back to ENROLL mode\r\n");
    return 0;
}

int spk_verify(uint32_t pcm_base_addr, int voice_start_frame, int valid_frame_len)
{
    if (g_spk_queue == NULL) {
        mprintf("[SPK] verify called before init\r\n");
        return -1;
    }
    if (valid_frame_len <= 0 || pcm_base_addr == 0) {
        mprintf("[SPK] verify: invalid args ptr=0x%x frm=%d\r\n",
                pcm_base_addr, valid_frame_len);
        return -1;
    }

    int asr_frm_shift = get_asrtop_asrfrmshift();
    int n_samples_total = valid_frame_len * asr_frm_shift;
    int byte_offset     = voice_start_frame * asr_frm_shift * (int)sizeof(short);
    const short *src    = (const short *)(pcm_base_addr + byte_offset);

    /* 同步 memcpy 到副本，防止 ASR 缓冲被覆盖 */
    int max_samples = (int)(sizeof(s_pcm_copy) / sizeof(short));
    int copy_samples = n_samples_total;
    if (copy_samples > max_samples) {
        mprintf("[SPK] WARN: pcm truncated %d -> %d samples\r\n",
                copy_samples, max_samples);
        copy_samples = max_samples;
    }
    memcpy(s_pcm_copy, src, copy_samples * sizeof(short));

    mprintf("[ASR->SPK] start_frm=%d valid_frm=%d frm_shift=%d samples=%d state=%d\r\n",
            voice_start_frame, valid_frame_len, asr_frm_shift,
            copy_samples, (int)g_spk_ctx.state);

    spk_msg_t msg;
    msg.n_samples = copy_samples;
    if (xQueueSend(g_spk_queue, &msg, 0) != pdPASS) {
        mprintf("[SPK] queue full, drop msg\r\n");
        return -1;
    }
    return 0;
}

/* ===== 内部：SPK 工作任务 ===== */

static void spk_task(void *arg)
{
    (void)arg;
    mprintf("[SPK] task running\r\n");

    spk_msg_t msg;
    for (;;) {
        if (xQueueReceive(g_spk_queue, &msg, portMAX_DELAY) != pdPASS) continue;

        /* 能量检查 */
        int energy = compute_avg_energy(s_pcm_copy, msg.n_samples);
        mprintf("[SPK] pcm energy=%d (>100=voice, <100=silence)\r\n", energy);

        /* 提取 MFCC */
        int mfcc_frm = 0;
        int ret = feat_extract_mfcc(s_pcm_copy, msg.n_samples,
                                     s_mfcc_buf, SPK_MAX_TEMPLATE_FRAMES, &mfcc_frm);
        if (ret != 0 || mfcc_frm < 5) {
            mprintf("[SPK] mfcc extract failed ret=%d frm=%d\r\n", ret, mfcc_frm);
            if (g_spk_ctx.verify_cb) g_spk_ctx.verify_cb(SPK_RESULT_ERROR, 0);
            continue;
        }
        mprintf("[SPK] mfcc extracted frm=%d\r\n", mfcc_frm);

        /* CMN */
        feat_apply_cmn(s_mfcc_buf, mfcc_frm);

        /* Delta + DeltaDelta 拼接 → 39 维 */
        if (feat_pack_with_delta(s_mfcc_buf, mfcc_frm, s_feat_buf) != 0) {
            mprintf("[SPK] feat pack failed\r\n");
            if (g_spk_ctx.verify_cb) g_spk_ctx.verify_cb(SPK_RESULT_ERROR, 0);
            continue;
        }
        s_feat_n_frames = mfcc_frm;
        mprintf("[SPK] feat packed dim=%d frm=%d\r\n", SPK_FEAT_DIM, mfcc_frm);

        /* 分支：注册 vs 验证 */
        if (g_spk_ctx.state == SPK_STATE_ENROLLING) {
            process_enroll_sample(g_spk_ctx.enroll_count);
        } else if (g_spk_ctx.state == SPK_STATE_VERIFYING) {
            process_verify();
        } else {
            mprintf("[SPK] state=IDLE, ignore\r\n");
        }
    }
}

/* ===== 内部：处理注册样本 ===== */

static void process_enroll_sample(int idx)
{
    if (idx < 0 || idx >= SPK_ENROLL_TIMES) {
        mprintf("[SPK] enroll idx out of range %d\r\n", idx);
        return;
    }

    /* 拷贝特征到 s_enroll_feats[idx] */
    memcpy(s_enroll_feats[idx], s_feat_buf,
           sizeof(float) * s_feat_n_frames * SPK_FEAT_DIM);
    s_enroll_frm_nums[idx] = s_feat_n_frames;

    mprintf("[SPK] enroll sample %d/%d stored, frm=%d\r\n",
            idx + 1, SPK_ENROLL_TIMES, s_feat_n_frames);

    /* 一致性检查：第 idx 个样本与第 0 个的 DTW 距离 */
    if (idx > 0) {
        float d = dtw_distance(s_enroll_feats[0], s_enroll_frm_nums[0],
                                s_enroll_feats[idx], s_enroll_frm_nums[idx],
                                SPK_DTW_BAND_RATIO_X100);
        mprintf("[SPK] consistency: sample1 vs sample%d dtw_dist=%d (lower=better)\r\n",
                idx + 1, (int)(d * 1000));
        if (d > 0.8f) {
            mprintf("[SPK] WARN: sample%d looks different, consider re-enrolling\r\n",
                    idx + 1);
        }
    }

    g_spk_ctx.enroll_count++;
    if (g_spk_ctx.enroll_count >= SPK_ENROLL_TIMES) {
        /* 全部收集完毕，计算平均模板并写 Flash */
        mprintf("[SPK] all %d samples collected, computing avg template\r\n",
                SPK_ENROLL_TIMES);
        if (compute_average_template() == 0) {
            spk_template_save(s_template, g_spk_ctx.template_frm_num);

            /* 读回验证 */
            float verify_tmpl[SPK_MAX_TEMPLATE_FRAMES][SPK_FEAT_DIM];
            int verify_frm = 0;
            if (spk_template_load(verify_tmpl, SPK_MAX_TEMPLATE_FRAMES, &verify_frm) == 0
                && verify_frm == g_spk_ctx.template_frm_num) {
                int diff_x1000 = 0;
                for (int t = 0; t < verify_frm; t++)
                    for (int d = 0; d < SPK_FEAT_DIM; d++)
                        diff_x1000 += (int)(fabsf(s_template[t][d] - verify_tmpl[t][d]) * 1000);
                mprintf("[SPK] flash verify: frm_match=YES total_diff_x1000=%d (should=0)\r\n",
                        diff_x1000);
            } else {
                mprintf("[SPK] flash verify: FAILED (frm=%d expected=%d)\r\n",
                        verify_frm, g_spk_ctx.template_frm_num);
            }

            g_spk_ctx.template_loaded = 1;
            g_spk_ctx.state           = SPK_STATE_VERIFYING;
            mprintf("[SPK] === ENROLL DONE -> VERIFY mode ===\r\n");
        } else {
            mprintf("[SPK] avg template failed, restart enrollment\r\n");
            g_spk_ctx.enroll_count = 0;
        }
    }
}

/* ===== 内部：处理验证 ===== */

static void process_verify(void)
{
    if (!g_spk_ctx.template_loaded || g_spk_ctx.template_frm_num <= 0) {
        mprintf("[SPK] no template loaded\r\n");
        if (g_spk_ctx.verify_cb) g_spk_ctx.verify_cb(SPK_RESULT_NO_TEMPLATE, 0);
        return;
    }

    float d = dtw_distance(s_template, g_spk_ctx.template_frm_num,
                            s_feat_buf, s_feat_n_frames,
                            SPK_DTW_BAND_RATIO_X100);
    int dist_x1000 = (int)(d * 1000);
    int thr_x1000  = SPK_DTW_THRESHOLD_X1000;

    mprintf("[SPK] DTW dist=%d thr=%d -> %s\r\n",
            dist_x1000, thr_x1000,
            dist_x1000 < thr_x1000 ? "ACCEPT" : "REJECT");

    if (dist_x1000 < thr_x1000) {
        if (g_spk_ctx.verify_cb) g_spk_ctx.verify_cb(SPK_RESULT_ACCEPT, dist_x1000);
    } else {
        if (g_spk_ctx.verify_cb) g_spk_ctx.verify_cb(SPK_RESULT_REJECT, dist_x1000);
    }
}

/* ===== 内部：DTW 对齐求平均模板 =====
 * 简化策略：以第 0 个样本为参考长度，把第 1、2 个样本按相同帧索引相加再取平均。
 * （比标准 DTW Barycenter Averaging 简单，但效果对 3 个一致性高的样本足够好。）
 */
static int compute_average_template(void)
{
    int ref_frm = s_enroll_frm_nums[0];
    if (ref_frm <= 0) return -1;

    /* 初始化为 sample 0 */
    memcpy(s_template, s_enroll_feats[0],
           sizeof(float) * ref_frm * SPK_FEAT_DIM);

    /* 累加其他样本（按比例索引映射，简单线性插值）*/
    for (int i = 1; i < SPK_ENROLL_TIMES; i++) {
        int src_frm = s_enroll_frm_nums[i];
        if (src_frm <= 0) continue;
        for (int t = 0; t < ref_frm; t++) {
            int src_t = t * src_frm / ref_frm;
            if (src_t >= src_frm) src_t = src_frm - 1;
            for (int d = 0; d < SPK_FEAT_DIM; d++) {
                s_template[t][d] += s_enroll_feats[i][src_t][d];
            }
        }
    }
    /* 求平均 */
    for (int t = 0; t < ref_frm; t++)
        for (int d = 0; d < SPK_FEAT_DIM; d++)
            s_template[t][d] /= (float)SPK_ENROLL_TIMES;

    g_spk_ctx.template_frm_num = ref_frm;
    mprintf("[SPK] avg template computed, frm=%d\r\n", ref_frm);
    return 0;
}