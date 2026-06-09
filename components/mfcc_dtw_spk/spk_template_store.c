#include <stdint.h>
#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

#define TPL_MAGIC  0x53504B33u   /* 'SPK3' — 升级magic, 使旧格式失效自动触发重注册 */

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
    int32_t  feat_dim;
} tpl_meta_t;

/* 每帧一个NV item: SPK_FEAT_DIM(39)*4 = 156字节 < 240上限
 * 关键: 新item必须先 cinv_item_init 创建, cinv_item_write 对不存在的item返回UNINIT且不创建 */
static int nv_put(uint32_t id, uint16_t len, const void *buf)
{
    cinv_item_ret_t r = cinv_item_write(id, len, (void *)buf);
    if (r == CINV_OPER_SUCCESS) return 0;          /* 已存在 -> 更新成功 */
    if (r == CINV_ITEM_UNINIT) {                    /* 不存在 -> 创建(init会写入数据) */
        r = cinv_item_init(id, len, (void *)buf);
        if (r == CINV_ITEM_UNINIT || r == CINV_OPER_SUCCESS) return 0;
    }
    mprintf("[SPK] NV put FAIL id=0x%x len=%d ret=%d\n", (unsigned)id, (int)len, (int)r);
    return (int)r;                                  /* 非0: 打印真实错误码 */
}

int spk_tpl_save(const float feats[][SPK_FEAT_DIM], int n_frames)
{
    if (n_frames <= 0 || n_frames > SPK_MAX_TEMPLATE_FRAMES) {
        mprintf("[SPK] tpl_save bad n_frames=%d\n", n_frames);
        return -1;
    }

    /* 先清掉旧模板, 避免多次注册残留占满NV */
    spk_tpl_delete();

    uint16_t flen = (uint16_t)(SPK_FEAT_DIM * sizeof(float));   /* 156 */

    /* 1. 逐帧写入 */
    for (int f = 0; f < n_frames; f++) {
        if (nv_put(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)f, flen, feats[f]) != 0) {
            mprintf("[SPK] tpl save FAILED at frame %d\n", f);
            return -1;
        }
    }

    /* 2. 最后写meta作为提交标记(写在最后, 中途掉电则meta无效->自动重注册) */
    tpl_meta_t meta = { TPL_MAGIC, n_frames, SPK_FEAT_DIM };
    if (nv_put(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta) != 0) {
        mprintf("[SPK] tpl save FAILED at meta\n");
        return -1;
    }

    mprintf("[SPK] tpl saved OK: %d frames x %d bytes (total %d)\n",
            n_frames, (int)flen, n_frames * (int)flen);
    return 0;
}

int spk_tpl_load(float feats[][SPK_FEAT_DIM], int *n_frames_out)
{
    tpl_meta_t meta;
    uint16_t   rlen = 0;

    if (cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta, &rlen)
            != CINV_OPER_SUCCESS)
        return -1;
    if (meta.magic != TPL_MAGIC || meta.n_frames <= 0 ||
        meta.n_frames > SPK_MAX_TEMPLATE_FRAMES || meta.feat_dim != SPK_FEAT_DIM)
        return -1;

    uint16_t flen = (uint16_t)(SPK_FEAT_DIM * sizeof(float));
    for (int f = 0; f < meta.n_frames; f++) {
        if (cinv_item_read(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)f,
                           flen, feats[f], &rlen) != CINV_OPER_SUCCESS) {
            mprintf("[SPK] tpl load FAIL at frame %d\n", f);
            return -1;
        }
    }
    *n_frames_out = meta.n_frames;
    mprintf("[SPK] tpl loaded: %d frames\n", meta.n_frames);
    return 0;
}

int spk_tpl_exists(void)
{
    tpl_meta_t meta;
    uint16_t   rlen = 0;
    cinv_item_ret_t r = cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META,
                                       sizeof(meta), &meta, &rlen);
    return (r == CINV_OPER_SUCCESS && meta.magic == TPL_MAGIC && meta.n_frames > 0);
}

int spk_tpl_delete(void)
{
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE_META);
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE);   /* 旧整模板ID(若存在) */
    for (uint32_t f = 0; f < SPK_MAX_TEMPLATE_FRAMES; f++)
        cinv_item_delete(NVDATA_ID_SPK_CHUNK_BASE + f);   /* 删不存在的item无害 */
    return 0;
}