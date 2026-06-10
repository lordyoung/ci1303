#include <stdint.h>
#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

#define TPL_MAGIC  0x53504B34u   /* 'SPK4' — int16量化格式, 升级magic使旧模板失效自动重注册 */

/* CINV_DATA_ITEM_MAX_LEN = 240 (USE_VPR=0: IO buf 256 - 16字节item头 = 240)
 * 编译期自适应: dim=60 -> 2帧/条目(240B); dim=39 -> 3帧/条目(234B) */
#define NV_MAX_ITEM_BYTES  240u
#define INT16_FRAME_BYTES  (SPK_FEAT_DIM * 2u)
#define FRAMES_PER_ITEM    (NV_MAX_ITEM_BYTES / INT16_FRAME_BYTES)

#if (FRAMES_PER_ITEM < 1)
#error "SPK_FEAT_DIM too large: one frame exceeds NV item limit"
#endif

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
    int32_t  feat_dim;
    float    scale;    /* 反量化系数: feat = q * scale */
} tpl_meta_t;

static int nv_put(uint32_t id, uint16_t len, const void *buf)
{
    cinv_item_ret_t r = cinv_item_write(id, len, (void *)buf);
    if (r == CINV_OPER_SUCCESS) return 0;
    if (r == CINV_ITEM_UNINIT) {
        r = cinv_item_init(id, len, (void *)buf);
        if (r == CINV_ITEM_UNINIT || r == CINV_OPER_SUCCESS) return 0;
    }
    mprintf("[SPK] NV put FAIL id=0x%x len=%d ret=%d\n", (unsigned)id, (int)len, (int)r);
    return (int)r;
}

int spk_tpl_save(const float feats[][SPK_FEAT_DIM], int n_frames)
{
    if (n_frames <= 0 || n_frames > SPK_MAX_TEMPLATE_FRAMES) {
        mprintf("[SPK] tpl_save bad n_frames=%d\n", n_frames);
        return -1;
    }

    spk_tpl_delete();   /* 先清旧模板释放NV空间 */

    /* 1. 求全局最大幅度 -> 量化系数 */
    float maxabs = 1e-6f;
    for (int f = 0; f < n_frames; f++)
        for (int d = 0; d < SPK_FEAT_DIM; d++) {
            float a = feats[f][d] < 0 ? -feats[f][d] : feats[f][d];
            if (a > maxabs) maxabs = a;
        }
    float scale = maxabs / 32767.0f;
    float inv   = 32767.0f / maxabs;

    /* 2. 按FRAMES_PER_ITEM帧打包成int16写入 */
    int16_t pack[FRAMES_PER_ITEM * SPK_FEAT_DIM];
    int item = 0;
    for (int f = 0; f < n_frames; f += (int)FRAMES_PER_ITEM, item++) {
        int nf = n_frames - f;
        if (nf > (int)FRAMES_PER_ITEM) nf = (int)FRAMES_PER_ITEM;
        for (int k = 0; k < nf; k++)
            for (int d = 0; d < SPK_FEAT_DIM; d++) {
                float q = feats[f + k][d] * inv;
                if (q >  32767.0f) q =  32767.0f;
                if (q < -32767.0f) q = -32767.0f;
                pack[k * SPK_FEAT_DIM + d] =
                    (int16_t)(q < 0 ? q - 0.5f : q + 0.5f);
            }
        uint16_t blen = (uint16_t)(nf * SPK_FEAT_DIM * sizeof(int16_t));
        if (nv_put(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)item, blen, pack) != 0) {
            mprintf("[SPK] tpl save FAILED at item %d (frame %d)\n", item, f);
            return -1;
        }
    }

    /* 3. 最后写meta作为提交标记(中途掉电meta无效->下次自动重注册) */
    tpl_meta_t meta = { TPL_MAGIC, n_frames, SPK_FEAT_DIM, scale };
    if (nv_put(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta) != 0) {
        mprintf("[SPK] tpl save FAILED at meta\n");
        return -1;
    }

    mprintf("[SPK] tpl saved OK: %d frames in %d items (%d frames/item)\n",
            n_frames, item, (int)FRAMES_PER_ITEM);
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

    int16_t pack[FRAMES_PER_ITEM * SPK_FEAT_DIM];
    int item = 0;
    for (int f = 0; f < meta.n_frames; f += (int)FRAMES_PER_ITEM, item++) {
        int nf = meta.n_frames - f;
        if (nf > (int)FRAMES_PER_ITEM) nf = (int)FRAMES_PER_ITEM;
        uint16_t blen = (uint16_t)(nf * SPK_FEAT_DIM * sizeof(int16_t));
        if (cinv_item_read(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)item,
                           blen, pack, &rlen) != CINV_OPER_SUCCESS) {
            mprintf("[SPK] tpl load FAIL at item %d\n", item);
            return -1;
        }
        for (int k = 0; k < nf; k++)
            for (int d = 0; d < SPK_FEAT_DIM; d++)
                feats[f + k][d] = pack[k * SPK_FEAT_DIM + d] * meta.scale;
    }
    *n_frames_out = meta.n_frames;
    mprintf("[SPK] tpl loaded: %d frames (%d items)\n", meta.n_frames, item);
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
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE);   /* 旧整块ID(若存在) */
    /* 循环到SPK_MAX_TEMPLATE_FRAMES: 同时清掉旧版per-frame格式残留条目 */
    for (uint32_t i = 0; i < SPK_MAX_TEMPLATE_FRAMES; i++)
        cinv_item_delete(NVDATA_ID_SPK_CHUNK_BASE + i);
    return 0;
}