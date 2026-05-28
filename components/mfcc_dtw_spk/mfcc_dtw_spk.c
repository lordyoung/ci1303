/**
 * @file mfcc_dtw_spk.c
 * @brief M1 skeleton — task creation, queue, prints. Stubs only.
 */
#include <math.h>    /* sinf，用于 M2 自检 */
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

/* ===== 内部消息类型 ===== */
typedef enum {
    SPK_MSG_VERIFY = 0,
    SPK_MSG_ENROLL = 1
} spk_msg_type_t;

typedef struct {
    spk_msg_type_t type;
    uint32_t pcm_base_addr;
    int voice_start_frame;
    int valid_frame_len;
} spk_msg_t;

/* ===== 模块上下文 ===== */
typedef enum {
    SPK_STATE_IDLE = 0,
    SPK_STATE_ENROLLING,
    SPK_STATE_VERIFYING
} spk_state_t;

typedef struct {
    spk_state_t state;
    spk_verify_cb_t verify_cb;
    spk_enroll_cb_t enroll_cb;
    int template_loaded;
    int template_frm_num;
    int enroll_count;
} spk_ctx_t;

static spk_ctx_t      g_spk_ctx     = { 0 };
static QueueHandle_t  g_spk_queue   = NULL;
static TaskHandle_t   g_spk_task_h  = NULL;

/* ===== SPK 工作任务 ===== */
static void spk_task(void *arg)
{
    mprintf("[SPK] task running\r\n");                       /* 打印6 */

    spk_msg_t msg;
    for (;;) {
        if (xQueueReceive(g_spk_queue, &msg, portMAX_DELAY) == pdPASS) {
            if (msg.type == SPK_MSG_VERIFY) {
                mprintf("[SPK] verify msg: ptr=0x%x start=%d len=%d\r\n",
                        msg.pcm_base_addr, msg.voice_start_frame, msg.valid_frame_len);
                /* M4 在此填入真正的验证流程 */
                if (g_spk_ctx.verify_cb) {
                    g_spk_ctx.verify_cb(SPK_RESULT_NO_TEMPLATE, 0);
                }
            } else if (msg.type == SPK_MSG_ENROLL) {
                mprintf("[SPK] enroll msg: ptr=0x%x start=%d len=%d\r\n",
                        msg.pcm_base_addr, msg.voice_start_frame, msg.valid_frame_len);
                /* M3 在此填入真正的注册流程 */
            }
        }
    }
}

/* ===== 公开 API 实现 ===== */
int spk_init(spk_verify_cb_t verify_cb)
{
    mprintf("[SPK] init start\r\n");                          /* 打印1 */
        /* 特征提取模块初始化（预计算 Hamming/Mel/DCT 系数表）*/
    feat_init();
    mprintf("[SPK] feat_init done\r\n");                  /* 打印：feat_init 完成 */
    g_spk_ctx.state         = SPK_STATE_IDLE;
    g_spk_ctx.verify_cb     = verify_cb;
    g_spk_ctx.enroll_cb     = NULL;
    g_spk_ctx.template_loaded   = 0;
    g_spk_ctx.template_frm_num  = 0;
    g_spk_ctx.enroll_count  = 0;

    g_spk_queue = xQueueCreate(2, sizeof(spk_msg_t));
    if (g_spk_queue == NULL) {
        mprintf("[SPK] ERROR: queue create failed\r\n");      /* 打印2a */
        return -1;
    }
    mprintf("[SPK] queue created ok\r\n");                    /* 打印2b */

    BaseType_t ret = xTaskCreate(spk_task, "spk", 1024, NULL, 3, &g_spk_task_h);
    if (ret != pdPASS) {
        mprintf("[SPK] ERROR: task create failed ret=%d\r\n", (int)ret); /* 打印3a */
        return -1;
    }
    mprintf("[SPK] task created ok\r\n");                     /* 打印3b */

    int load_ret = spk_template_load_meta(&g_spk_ctx.template_frm_num);
    if (load_ret == 0 && g_spk_ctx.template_frm_num > 0) {
        g_spk_ctx.template_loaded = 1;
        mprintf("[SPK] template loaded from flash, frm=%d\r\n",
                g_spk_ctx.template_frm_num);                  /* 打印4a */
    } else {
        mprintf("[SPK] no template in flash (first boot)\r\n"); /* 打印4b */
    }

    #if SPK_DEBUG
    /* ===== M2 自检：对一帧合成正弦波计算 MFCC，与 PC Python 参考对比 ===== */
    {
        static short  s_test_pcm[SPK_FRAME_LEN];       /* 静态，避免占栈 */
        static float  s_test_mfcc[1][SPK_N_MFCC_BASE];

        for (int i = 0; i < SPK_FRAME_LEN; i++) {
            s_test_pcm[i] = (short)(8000.0f *
                sinf(2.0f * 3.14159265f * 440.0f * (float)i / 16000.0f));
        }
        int test_nfrm = 0;
        feat_extract_mfcc(s_test_pcm, SPK_FRAME_LEN,
                          s_test_mfcc, 1, &test_nfrm);

        mprintf("[SPK] M2 MFCC self-test (440Hz sine, 1 frame):\r\n");
        for (int d = 0; d < SPK_N_MFCC_BASE; d++) {
            mprintf("  C%02d = %d (x0.001)\r\n", d + 1,
                    (int)(s_test_mfcc[0][d] * 1000));
        }
        mprintf("[SPK] compare with Python reference below\r\n");
    }
#endif
    mprintf("[SPK] init done\r\n");                           /* 打印5 */
    return 0;
}

int spk_start_enroll(spk_enroll_cb_t enroll_cb)
{
    mprintf("[SPK] start_enroll called (stub)\r\n");
    g_spk_ctx.enroll_cb    = enroll_cb;
    g_spk_ctx.enroll_count = 0;
    g_spk_ctx.state        = SPK_STATE_ENROLLING;
    return 0;
}

int spk_verify(uint32_t pcm_base_addr, int voice_start_frame, int valid_frame_len)
{
    if (g_spk_queue == NULL) {
        mprintf("[SPK] verify called before init\r\n");
        return -1;
    }
    spk_msg_t msg;
    msg.type              = (g_spk_ctx.state == SPK_STATE_ENROLLING) ? SPK_MSG_ENROLL : SPK_MSG_VERIFY;
    msg.pcm_base_addr     = pcm_base_addr;
    msg.voice_start_frame = voice_start_frame;
    msg.valid_frame_len   = valid_frame_len;
    if (xQueueSend(g_spk_queue, &msg, 0) != pdPASS) {
        mprintf("[SPK] queue full, drop msg\r\n");
        return -1;
    }
    return 0;
}

int spk_delete_template(void)
{
    mprintf("[SPK] delete_template called (stub)\r\n");
    spk_template_clear();
    g_spk_ctx.template_loaded  = 0;
    g_spk_ctx.template_frm_num = 0;
    return 0;
}