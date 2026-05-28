#include <string.h>
#include "spk_template_store.h"
#include "user_config.h"
#include "ci_nvdata_manage.h"
#include "ci_log.h"

typedef struct {
    uint32_t magic;
    int32_t  n_frames;
} spk_template_meta_t;

int spk_template_load(float feat[][SPK_FEAT_DIM], int *n_frames)
{
    spk_template_meta_t meta;
    uint16_t real_len = 0;
    cinv_item_ret_t ret;

    ret = cinv_item_read(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta, &real_len);
    if (ret != CINV_OPER_SUCCESS || meta.magic != SPK_TEMPLATE_MAGIC) {
        mprintf("[SPK] No template in Flash (meta ret=%d)\n", (int)ret);
        return -1;
    }
    if (meta.n_frames <= 0 || meta.n_frames > SPK_MAX_TEMPLATE_FRAMES) {
        mprintf("[SPK] Template meta n_frames=%d out of range\n", (int)meta.n_frames);
        return -1;
    }

    uint16_t data_size = (uint16_t)((size_t)meta.n_frames * SPK_FEAT_DIM * sizeof(float));
    ret = cinv_item_read(NVDATA_ID_SPK_TEMPLATE, data_size, feat, &real_len);
    if (ret != CINV_OPER_SUCCESS || real_len != data_size) {
        mprintf("[SPK] Template data read failed ret=%d\n", (int)ret);
        return -1;
    }

    *n_frames = (int)meta.n_frames;
    mprintf("[SPK] Template loaded: %d frames\n", *n_frames);
    return 0;
}

int spk_template_save(const float feat[][SPK_FEAT_DIM], int n_frames)
{
    spk_template_meta_t meta;
    meta.magic    = SPK_TEMPLATE_MAGIC;
    meta.n_frames = (int32_t)n_frames;

    uint16_t data_size = (uint16_t)((size_t)n_frames * SPK_FEAT_DIM * sizeof(float));
    cinv_item_ret_t ret;

    ret = cinv_item_write(NVDATA_ID_SPK_TEMPLATE, data_size, (void *)feat);
    if (ret != CINV_OPER_SUCCESS) {
        ret = cinv_item_init(NVDATA_ID_SPK_TEMPLATE, data_size, (void *)feat);
        if (ret != CINV_OPER_SUCCESS) {
            mprintf("[SPK] Template data write failed ret=%d\n", (int)ret);
            return -1;
        }
    }

    ret = cinv_item_write(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta);
    if (ret != CINV_OPER_SUCCESS) {
        ret = cinv_item_init(NVDATA_ID_SPK_TEMPLATE_META, sizeof(meta), &meta);
        if (ret != CINV_OPER_SUCCESS) {
            mprintf("[SPK] Template meta write failed ret=%d\n", (int)ret);
            return -1;
        }
    }

    mprintf("[SPK] Template saved: %d frames\n", n_frames);
    return 0;
}

int spk_template_delete(void)
{
    cinv_item_ret_t r1 = cinv_item_delete(NVDATA_ID_SPK_TEMPLATE_META);
    cinv_item_ret_t r2 = cinv_item_delete(NVDATA_ID_SPK_TEMPLATE);
    mprintf("[SPK] Template deleted (r1=%d r2=%d)\n", (int)r1, (int)r2);
    return (r1 == CINV_OPER_SUCCESS && r2 == CINV_OPER_SUCCESS) ? 0 : -1;
}