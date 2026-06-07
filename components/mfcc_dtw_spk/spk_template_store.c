#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

#define TPL_MAGIC  0x53504B32u   /* 'SPK2'：升级版本号，旧格式自动失效→重注册一次 */

/* 每帧字节数；必须 ≤ NV 单条目上限(240)，否则写入会静默失败 */
#define SPK_FRAME_BYTES   ((uint16_t)(SPK_FEAT_DIM * sizeof(float)))
#define SPK_NV_ITEM_LIMIT (240)

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
} tpl_meta_t;

/* 返回 1=成功，0=失败 */
static int nv_write_checked(uint32_t id, uint16_t len, void *buf)
{
    cinv_item_ret_t r = cinv_item_write(id, len, buf);
    if (r != CINV_OPER_SUCCESS) {
        cinv_item_delete(id);
        cinv_item_init(id, len, buf);
        r = cinv_item_write(id, len, buf);
    }
    return (r == CINV_OPER_SUCCESS);
}

int spk_tpl_save(const float feats[][SPK_FEAT_DIM], int n_frames)
{
    /* 边界保护：每帧不得超过 NV 单条目上限 */
    if (SPK_FRAME_BYTES > SPK_NV_ITEM_LIMIT) {
        mprintf("[SPK] FATAL: frame_bytes=%u > NV limit=%d, cannot persist!\n",
                (unsigned)SPK_FRAME_BYTES, SPK_NV_ITEM_LIMIT);
        return -1;
    }

    /* 先写帧块，全部成功后再写 meta，避免 meta 有效但块缺失 */
    int ok = 1;
    for (int i = 0; i < n_frames; i++) {
        if (!nv_write_checked(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)i,
                              SPK_FRAME_BYTES, (void *)feats[i])) {
            mprintf("[SPK] tpl save FAILED at frame %d (NV full?)\n", i);
            ok = 0;
            break;
        }
    }
    if (!ok) return -1;

    tpl_meta_t meta = { TPL_MAGIC, (int32_t)n_frames };
    if (!nv_write_checked(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta)) {
        mprintf("[SPK] tpl meta save FAILED\n");
        return -1;
    }

    mprintf("[SPK] tpl saved OK: %d frames x %u bytes\n",
            n_frames, (unsigned)SPK_FRAME_BYTES);
    return 0;
}

int spk_tpl_load(float feats[][SPK_FEAT_DIM], int *n_frames_out)
{
    tpl_meta_t meta;
    uint16_t   rlen = 0;

    if (cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta, &rlen)
            != CINV_OPER_SUCCESS)
        return -1;
    if (meta.magic != TPL_MAGIC || meta.n_frames <= 0)
        return -1;

    int n = (meta.n_frames <= SPK_MAX_TEMPLATE_FRAMES)
            ? meta.n_frames : SPK_MAX_TEMPLATE_FRAMES;

    for (int i = 0; i < n; i++) {
        if (cinv_item_read(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)i,
                           SPK_FRAME_BYTES, feats[i], &rlen) != CINV_OPER_SUCCESS) {
            mprintf("[SPK] tpl load: chunk %d missing\n", i);
            return -1;
        }
    }

    *n_frames_out = n;
    mprintf("[SPK] tpl loaded OK: %d frames\n", n);
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
    tpl_meta_t meta;
    uint16_t   rlen = 0;
    int n = 0;
    if (cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta, &rlen)
            == CINV_OPER_SUCCESS && meta.magic == TPL_MAGIC)
        n = (meta.n_frames <= SPK_MAX_TEMPLATE_FRAMES)
            ? meta.n_frames : SPK_MAX_TEMPLATE_FRAMES;

    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE_META);
    for (int i = 0; i < n; i++)
        cinv_item_delete(NVDATA_ID_SPK_CHUNK_BASE + (uint32_t)i);
    return 0;
}