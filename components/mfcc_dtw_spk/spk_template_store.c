#include <stdint.h>

#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

#define TPL_MAGIC  0x53504B32u   /* 'SPK2' — 升级特征维度后必须重新注册 */

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
} tpl_meta_t;

static void nv_write_safe(uint32_t id, uint16_t len, void *buf)
{
    cinv_item_ret_t r = cinv_item_write(id, len, buf);
    if (r != CINV_OPER_SUCCESS) {
        cinv_item_delete(id);
        cinv_item_init(id, len, buf);
        cinv_item_write(id, len, buf);
    }
}

int spk_tpl_save(const float feats[][SPK_FEAT_DIM], int n_frames)
{
    tpl_meta_t meta   = { TPL_MAGIC, (int32_t)n_frames };
    uint16_t   dlen   = (uint16_t)(n_frames * SPK_FEAT_DIM * sizeof(float));

    nv_write_safe(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta);
    nv_write_safe(NVDATA_ID_SPK_TEMPLATE,      dlen,         (void *)feats);

    mprintf("[SPK] tpl saved: %d frames (%d bytes)\n", n_frames, (int)dlen);
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
    uint16_t dlen = (uint16_t)(n * SPK_FEAT_DIM * sizeof(float));

    if (cinv_item_read(NVDATA_ID_SPK_TEMPLATE, dlen, feats, &rlen)
            != CINV_OPER_SUCCESS)
        return -1;

    *n_frames_out = n;
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
    cinv_item_delete(NVDATA_ID_SPK_TEMPLATE);
    return 0;
}