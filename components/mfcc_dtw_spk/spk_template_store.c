/**
 * @file spk_template_store.c
 * @brief Save/load average MFCC+Delta template in flash via cinv_item API.
 *        Storage layout:
 *           NVDATA_ID_SPK_TEMPLATE_META : { int magic; int n_frames; }
 *           NVDATA_ID_SPK_TEMPLATE      : float[n_frames][SPK_FEAT_DIM]
 *
 * Flash endurance: write only during user enrollment (~1 erase per use).
 * No adaptive update on verify (read-only) — typical lifetime erases < 100.
 */
#include <string.h>
#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

#define SPK_TEMPLATE_MAGIC   0x53504B31   /* 'SPK1' */

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
} spk_meta_t;

int spk_template_save(const float tmpl[][SPK_FEAT_DIM], int n_frames)
{
    if (n_frames <= 0 || n_frames > SPK_MAX_TEMPLATE_FRAMES) {
        mprintf("[STORE] save: invalid n_frames=%d\r\n", n_frames);
        return -1;
    }

    /* 写 meta */
    spk_meta_t meta;
    meta.magic    = SPK_TEMPLATE_MAGIC;
    meta.n_frames = n_frames;
    cinv_item_ret_t r1 = cinv_item_write(NVDATA_ID_SPK_TEMPLATE_META,
                                          sizeof(spk_meta_t), &meta);
    if (r1 != CINV_OPER_SUCCESS) {
        /* 首次写：item 不存在，需要 cinv_item_init 创建 */
        cinv_item_init(NVDATA_ID_SPK_TEMPLATE_META, sizeof(spk_meta_t), &meta);
    }

    /* 写模板数据 */
    uint16_t data_len = (uint16_t)(n_frames * SPK_FEAT_DIM * sizeof(float));
    cinv_item_ret_t r2 = cinv_item_write(NVDATA_ID_SPK_TEMPLATE,
                                          data_len, (void *)tmpl);
    if (r2 != CINV_OPER_SUCCESS) {
        cinv_item_init(NVDATA_ID_SPK_TEMPLATE, data_len, (void *)tmpl);
    }

    mprintf("[STORE] save: meta_ret=%d data_ret=%d n_frames=%d size=%d\r\n",
            (int)r1, (int)r2, n_frames, (int)data_len);
    return 0;
}

int spk_template_load_meta(int *out_frames)
{
    spk_meta_t meta;
    memset(&meta, 0, sizeof(meta));
    uint16_t real_len = 0;
    cinv_item_ret_t r = cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META,
                                        sizeof(meta), &meta, &real_len);
    if (r != CINV_OPER_SUCCESS || meta.magic != SPK_TEMPLATE_MAGIC) {
        if (out_frames) *out_frames = 0;
        return -1;
    }
    if (out_frames) *out_frames = meta.n_frames;
    return 0;
}

int spk_template_load(float tmpl[][SPK_FEAT_DIM], int max_frames, int *out_frames)
{
    int n_frames = 0;
    if (spk_template_load_meta(&n_frames) != 0 || n_frames <= 0) {
        if (out_frames) *out_frames = 0;
        return -1;
    }
    if (n_frames > max_frames) {
        mprintf("[STORE] load: n_frames=%d > max=%d\r\n", n_frames, max_frames);
        if (out_frames) *out_frames = 0;
        return -1;
    }

    uint16_t want = (uint16_t)(n_frames * SPK_FEAT_DIM * sizeof(float));
    uint16_t real_len = 0;
    cinv_item_ret_t r = cinv_item_read(NVDATA_ID_SPK_TEMPLATE,
                                        want, (void *)tmpl, &real_len);
    if (r != CINV_OPER_SUCCESS || real_len != want) {
        mprintf("[STORE] load: read failed ret=%d real=%d want=%d\r\n",
                (int)r, (int)real_len, (int)want);
        if (out_frames) *out_frames = 0;
        return -1;
    }

    if (out_frames) *out_frames = n_frames;
    mprintf("[STORE] load: ok n_frames=%d size=%d\r\n", n_frames, (int)want);
    return 0;
}

int spk_template_clear(void)
{
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE_META);
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE);
    mprintf("[STORE] cleared\r\n");
    return 0;
}