/**
 * @file spk_template_store.c
 * @brief M1 stub. Real implementation in M3.
 */
#include "spk_template_store.h"
#include "ci_nvdata_manage.h"
#include "user_config.h"
#include "ci_log.h"

int spk_template_save(const float tmpl[][SPK_FEAT_DIM], int n_frames)
{
    (void)tmpl; (void)n_frames;
    mprintf("[STORE] save stub\r\n");
    return -1;
}

int spk_template_load(float tmpl[][SPK_FEAT_DIM], int max_frames, int *out_frames)
{
    (void)tmpl; (void)max_frames;
    if (out_frames) *out_frames = 0;
    mprintf("[STORE] load stub\r\n");
    return -1;
}

int spk_template_load_meta(int *out_frames)
{
    if (out_frames) *out_frames = 0;
    /* M1 阶段直接返回"无模板"，让 spk_init 打印"first boot" */
    return -1;
}

int spk_template_clear(void)
{
    mprintf("[STORE] clear stub\r\n");
    return 0;
}