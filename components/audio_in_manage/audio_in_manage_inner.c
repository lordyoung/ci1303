/**
  ******************************************************************************
  * @file    voice_in_manage_inner.c
  * @version V1.0.0
  * @date    2021.11.02
  * @brief  在HOST端接收
  ******************************************************************************
  */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "codec_manage_inner_port.h"
#include "codec_manager.h"
#include "audio_in_manage.h"
//#include "asr_top_config.h"
#include "sdk_default_config.h"
#include "ci130x_system_ept.h"
#include "audio_in_manage_inner.h"
#include "ci130x_nuclear_com.h"
#include "status_share.h"
#include "system_msg_deal.h"
#include "ci130x_audio_pre_rslt_out.h"
#include "romlib_runtime.h"
#include "alg_preprocess.h"
#include "ci130x_nuclear_com.h"
#include "ci130x_iwdg.h"
#include "ci_flash_data_info.h"
#include "user_config.h"
#if USE_MFCC_DTW_SPK
#include "mfcc_dtw_spk.h"
#endif
#if USE_MFCC_DTW_SPK && SPK_USE_DUAL_CHIP_DENOISE
#include "ci130x_scu.h"
#include "ci130x_gpio.h"
#include "ci130x_dpmu.h"
#endif
/**************************************************************************
                    typedef
****************************************************************************/

/**************************************************************************
                    global
****************************************************************************/
volatile int sys_err_flag = false;

/**************************************************************************
                    function
****************************************************************************/
void print_nn_err_cnt(void);

static void send_audio_data_to_asr(uint32_t buf, uint32_t size)
{
    extern int get_asrtop_asrfrmshift(void);
    int frm_shif = get_asrtop_asrfrmshift();
    int frms = size / (frm_shif * sizeof(short));
    extern int send_runvad_msg(unsigned int voice_ptr, int voice_frm_nums, int irq_flag);
    send_runvad_msg(buf, frms, 0);
}

static void audio_in_iis_callback(void)
{
    uint32_t iis3_reg[7] = {0};
    for (int i = 0; i < 7; i++)
    {
        iis3_reg[i] = *(volatile uint32_t *)((uint32_t)IIS2 + 4 * i);
    }
    //scu_set_device_reset((uint32_t)IIS2);
    //scu_set_device_reset_release((uint32_t)IIS2);
    for (int i = 0; i < 7; i++)
    {
        *(volatile uint32_t *)((uint32_t)IIS2 + 4 * i) = iis3_reg[i];
    }
    mprintf("IIS watchdog callback\n");
}

static void audio_in_ept_cal_serve_rpmsg(char *para, uint32_t size)
{
    nuclear_com_t str;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void*)&str,0,sizeof(str));
    str.src_ept_num = 0;
    str.dst_ept_num = audio_in_serve_ept_num;
    str.data_p = (void *)para;
    str.data_len = size;
    nuclear_com_send(&str, 0xfffff);
}

void send_voice_info_to_audio_in_manage(audio_in_get_voice_t *str_p)
{
    uint32_t para[6];
    para[0] = send_voice_info_to_audio_in_manage_ept_num;
    para[1] = str_p->cha_num;
    para[2] = str_p->channel_flag;
    para[3] = str_p->addr1;
    para[4] = str_p->addr2;
    para[5] = str_p->size;
    audio_in_ept_cal_serve_rpmsg((char *)para, sizeof(para));
}


void set_ssp_registe(audio_capture_t* audio_capture, ci_ssp_st* ci_ssp, int module_num)
{
	uint32_t para[4];
    para[0] = set_ssp_registe_ept_num;
    para[1] = (uint32_t)audio_capture;
    para[2] = (uint32_t)ci_ssp;
    para[3] = (uint32_t)module_num;
    audio_in_ept_cal_serve_rpmsg((char *)para, sizeof(para));
}



//语音处理完一帧的回调函数，里面的执行过程一定要尽可能短
void audio_deal_one_frm_callback(void* para)
{
#if USE_MFCC_DTW_SPK && !SPK_USE_DUAL_CHIP_DENOISE
        /* 单芯片模式: 本地SSP输出的每帧PCM喂给说话人识别环形缓冲区
           para[0] = 音频数据地址, para[1] = short 采样点数
           双芯片模式下改由 spk_iis0_rx_task 从IIS0喂入, 此处不再喂本地PCM(ASR不受影响) */
        {
            uint32_t *spk_d = (uint32_t *)para;
            spk_feed_pcm((const short *)spk_d[0], (int)spk_d[1]);
        }
#endif
    #if VOICE_UPLOAD_BY_UART
    uint32_t *data = (uint32_t *)para; //data[0] = 音频数据地址  data[1] = 音频数据个数， 10ms， 256个short数据
    int8_t *pcm_data = (int8_t *)data[0];
    extern volatile bool voice_upload_vad_enable;
    extern QueueHandle_t pcm_speex_deal_queue;
    if(CI_SS_PLAY_STATE_PLAYING == ciss_get(CI_SS_PLAY_STATE))
    {
        return;
    }
    if (voice_upload_vad_enable)
    {
        if (xQueueSend(pcm_speex_deal_queue, pcm_data, 0) == pdFALSE)
        {
            ci_loginfo(LOG_VOICE_UPLOAD, ">>>> send pcm_speex_deal_queue fail !!!<<<<\n");
        }
    }
    #endif
}

static get_fft_rslt_fun_t get_fft_rslt_fun_callback = NULL;


//设置的回调函数执行过程尽可能短！
void set_get_fft_rslt_fun_callback(get_fft_rslt_fun_t fun)
{
    get_fft_rslt_fun_callback = fun;
}


//获取每帧语音的FFT数据的回调函数 示例
void my_get_fft_rslt_fun(float* fft_rslt_p,int num)
{
    mprintf("psd:\n");
    for(int i=0;i<num;i++)
    {
        mprintf("%d\n",(int)fft_rslt_p[i]);
    }
}


/**
 * @brief 获取每帧语音的FFT数据的回调函数（每16ms调用一次）,里面的执行过程一定要尽可能短
 * 
 * @param para 
 */
void deal_one_frm_fft_rslt_callback(void* para)
{
    uint32_t* data = (uint32_t*)para;
    float* psd_data = (float*)data[0];//psd的地址
    int psd_num = data[1];//psd的个数
    if(get_fft_rslt_fun_callback)
    {
        get_fft_rslt_fun_callback(psd_data,psd_num);
    }
}


static bool voice_energy_gate = false;
static get_voice_energy_fun_t get_voice_energy_callback_fun = NULL;
void is_open_voice_energy(bool gate,get_voice_energy_fun_t* fun)
{
    voice_energy_gate = gate;
    get_voice_energy_callback_fun = fun;
}
//获取NN降噪模型地址
void get_ci_ai_denoise_model_addr(void)
{
    #if USE_DENOISE_NN
    uint32_t addr;
    uint32_t size;
    if(get_dnn_addr_by_id(CI_AI_DENOISE_MODEL_ID,&addr,&size))
    {
         ciss_set(CI_SS_AI_DENOISE_MODEL_ADDR, addr);
    }
    else
    {
        mprintf("mode id err, please verify mode id = %d\n", CI_AI_DENOISE_MODEL_ID);
        CI_ASSERT(0,"\n");
    }
    #endif
}
//获取DOA模型地址
void get_ci_ai_doa_model_addr(void)
{
    #if USE_AI_DOA
    uint32_t addr;
    uint32_t size;
    if(get_dnn_addr_by_id(CI_AI_DOA_MODEL_ID,&addr,&size))
    {
        ciss_set(CI_SS_AI_DOA_MODEL_ADDR,addr);
    }
    else
    {
        mprintf("mode id err, please verify mode id = %d\n", CI_AI_DOA_MODEL_ID);
        CI_ASSERT(0,"\n");
    }
    #endif
}
//获取事件检测模型地址
void get_ci_sed_model_addr(void)
{
#if USE_SED
    uint32_t addr;
    uint32_t size;
#if USE_SED_CRY
    if(get_dnn_addr_by_id(CI_AI_CRY_MODEL_ID,&addr,&size))
#elif USE_SED_SNORE
    if(get_dnn_addr_by_id(CI_AI_SNORE_MODEL_ID,&addr,&size))
#endif
    {
        ciss_set(CI_SS_SED_MODEL_ADDR,addr);
    }
    else
    {
        mprintf("mode id err, please verify mode id = %d/%d\n", CI_AI_CRY_MODEL_ID, CI_AI_SNORE_MODEL_ID);
        CI_ASSERT(0,"\n");
    }
    #endif
}
//获取声纹模型地址
void get_ci_vpr_model_addr(void)
{
    #if USE_VPR 
    uint32_t addr;
    uint32_t size;
    if(get_dnn_addr_by_id(VOICE_VPR_DNN_ID, &addr, &size))
    {
        ciss_set(CI_SS_VPR_MODEL_ADDR, addr);
    }
    else
    {
        mprintf("mode id err, please verify mode id = %d\n", VOICE_VPR_DNN_ID);
        CI_ASSERT(0,"\n");
    }
    if((addr >= 0x1000000) || (addr <= 0x6000))
    {
        mprintf("VOICE_VPR_DNN_ID = %d addr = %x, please check\n",VOICE_VPR_DNN_ID, addr);
        CI_ASSERT(0,"\n");
    }
    #endif
}
//获取声纹男女声检测模型地址
void get_ci_wman_vpr_model_addr(void)
{
    #if USE_WMAN_VPR
    uint32_t addr;
    uint32_t size;
    if(get_dnn_addr_by_id(VOICE_WMAN_MODEL_ID, &addr, &size))
    {
        ciss_set(CI_SS_VPR_WMAN_MODEL_ADDR, addr);
    }
    else
    {
        mprintf("mode id err, please verify mode id = %d\n", VOICE_WMAN_MODEL_ID);
        CI_ASSERT(0,"\n");
    }
    if((addr >= 0x1000000) || (addr <= 0x6000))
    {
        mprintf("VOICE_WMAN_MODEL_ID = %d addr = %x, please check\n",VOICE_WMAN_MODEL_ID, addr);
        CI_ASSERT(0,"\n");
    }
    #endif
}

//获取TTS模型地址
void get_ci_tts_model_addr(void)
{
    uint32_t addr;
    uint32_t size;
    //for acu
    if(get_dnn_addr_by_id(CI_AI_TTS_ACU_MODEL_ID,&addr,&size))
    {
        ciss_set(CI_SS_TTS_ACU_MODEL_ADDR,addr);
    }
    else
    {
        CI_ASSERT(addr,"\n");
    }
    //for dur
     if(get_dnn_addr_by_id(CI_AI_TTS_DUR_MODEL_ID,&addr,&size))
    {
        ciss_set(CI_SS_TTS_DUR_MODEL_ADDR,addr);
    }
    else
    {
        CI_ASSERT(addr,"\n");
    }
}

void ci_ssp_alg_switch_init(void)
{   
    #if ON_LINE_SUPPORT
        #if USE_ALC_AUTO_SWITCH_MODULE
        ciss_set(CI_SS_ALC_AUTO_SWITCH_IS_OPEN,true);
        #else
        ciss_set(CI_SS_ALC_AUTO_SWITCH_IS_OPEN,false);
        #endif

        #if USE_AEC_MODULE
        ciss_set(CI_SS_AEC_IS_OPEN,true);
        #else
        ciss_set(CI_SS_AEC_IS_OPEN,false);
        #endif
        #if USE_AI_DOA
        ciss_set(CI_SS_DOA_IS_OPEN,true);
        #else
        ciss_set(CI_SS_DOA_IS_OPEN,false);
        #endif
        

        #if USE_DEREVERB_MODULE
        ciss_set(CI_SS_DR_IS_OPEN,true);
        #else
        ciss_set(CI_SS_DR_IS_OPEN,false);
        #endif

        #if USE_BEAMFORMING_MODULE
        ciss_set(CI_SS_BF_IS_OPEN,true);
        #else
        ciss_set(CI_SS_BF_IS_OPEN,false);
        #endif

        #if USE_DENOISE_MODULE
        ciss_set(CI_SS_DENOISE_IS_OPEN,true);
        #else
        ciss_set(CI_SS_DENOISE_IS_OPEN,false);
        #endif
    #endif

    #if (IF_USE_ANOTHER_CODEC_TO_GET_REF || USE_SOFT_AEC_REF)
    ciss_set(CI_SS_TWO_CODEC_IS_USED,true);
    #else
    ciss_set(CI_SS_TWO_CODEC_IS_USED,false);
    #endif
    
}


static void ven_callback_fun(float vn)
{
    mprintf("voice energy = %d\n",(int)vn);
}


#if USE_SOFT_AEC_REF
#define DATA_VOICE_BUFFER_NUM (4)
int16_t data_ref_buffer[DATA_VOICE_BUFFER_NUM][AUDIO_CAP_DIV_NUM][AUDIO_CAP_POINT_NUM_PER_FRM / AUDIO_CAP_DIV_NUM];


//MIC数据是双通道的
#if INNER_CODEC_AUDIO_IN_USE_RESAMPLE
#define MIC_DATA_NUM    AUDIO_CAP_POINT_NUM_PER_FRM / AUDIO_CAP_DIV_NUM * 2 * 2
#else
#define MIC_DATA_NUM    AUDIO_CAP_POINT_NUM_PER_FRM / AUDIO_CAP_DIV_NUM * 2
#endif

//软回采不能使用在单麦的方案上
int16_t data_mic_buffer[DATA_VOICE_BUFFER_NUM][AUDIO_CAP_DIV_NUM][MIC_DATA_NUM];
#endif
void audio_in_manage_inner_task(void *p)
{
    ci_ssp_alg_switch_init();
    //打印中间结果
    //ctr_asr_detail_result(1);

    // extern void set_nn_to_dtw_init_info(bool cinn_is_using_to_dtw,uint16_t max_num_to_dtw);
    // set_nn_to_dtw_init_info(true,20);

    #if !NO_ASR_FLOW
    while (CI_SS_ASR_SYS_POWER_OFF == ciss_get(CI_SS_ASR_SYS_STATE))
    {
        vTaskDelay(100);
        mprintf("wait asr start done\n");
    }

    char asr_ver_buf[100] = {0};

    extern int get_asr_sys_verinfo(char *version_buf);
    get_asr_sys_verinfo(asr_ver_buf);

    ci_loginfo(LOG_USER, "asr_ver:[%s]\n", asr_ver_buf);
    #endif

    vTaskSuspendAll();
    #if IF_USE_ANOTHER_CODEC_TO_GET_REF
    ref_in_codec_registe();
    cm_start_codec(REF_RECORD_CODEC_ID, CODEC_INPUT);
    #endif
    cm_start_codec(HOST_MIC_RECORD_CODEC_ID, CODEC_INPUT);
    ciss_set(CI_SS_MIC_VOICE_STATUE, CI_SS_MIC_VOICE_NORMAL);
    xTaskResumeAll();

    #if !NO_ASR_FLOW
    sys_msg_t send_msg;
    send_msg.msg_type = SYS_MSG_TYPE_AUDIO_IN_STARTED;
    send_msg_to_sys_task(&send_msg, NULL);
    #endif

    #if USE_SED
    cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,ENABLE);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    #endif

    #if USE_IIS1_OUT_PRE_RSLT_AUDIO || RECORD_PLAY_BY_FLASH_ENABLE || USE_UART_SEND_PRE_RSLT_AUDIO
    audio_pre_rslt_out_play_card_init();
    #endif
    //语音前端算法处理模块初始化

    int data_cnt = 0;
    int mic_cnt = 0;
    int ref_cnt = 0;

    #if USE_SOFT_AEC_REF
    int buffer_cnt = 0;
    int div_num = 0;
    uint32_t pre_ref_addr = 0;
    
    int mic_data_block_size = MIC_DATA_NUM * sizeof(int16_t);
    int ref_data_block_size = AUDIO_CAP_POINT_NUM_PER_FRM / AUDIO_CAP_DIV_NUM * sizeof(int16_t);
    #endif
    ciss_set(CI_SS_AST_STATE, 0);
    for (;;)
    {
        uint32_t data_addr = 0, data_size;

        #if IF_USE_ANOTHER_CODEC_TO_GET_REF
        uint32_t data_addr_ref = 0;
        uint32_t data_size_ref;
        cm_read_codec(REF_RECORD_CODEC_ID, &data_addr_ref, &data_size_ref); //读取IIS0数据

        print_nn_err_cnt();
        int16_t* data_p = (int16_t*)data_addr_ref;
        //  for(int i = 0; i < 16; i++)
        // {
        //     mprintf("%d ", data_p[i]);
        // }
        // mprintf("\r\n");
        // mprintf("%d %d %d\n",data_p[0],data_p[100],data_p[200]);
        // init_timer0();
        // timer0_start_count();
        // ref_cnt++;
        if (!data_addr_ref)
        {
            continue;
        }
        #endif
        
        #if USE_SED
        if(data_cnt == 100)
        {
            cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,DISABLE);
            data_cnt++;
            // mprintf("resume mute\n");
        }
        else if(data_cnt < 100)
        {
            data_cnt++;
        }
        #endif

        cm_read_codec(HOST_MIC_RECORD_CODEC_ID, &data_addr, &data_size);
        print_nn_err_cnt();
        int asr_state = ciss_get(CI_SS_AST_STATE);
        int asr_warning = ciss_get(CI_SS_ASR_WARNING);

        if(sys_err_flag || asr_state)
        {
            ci_logerr(CI_LOG_ERROR,"sys_err_flag = %d\n",sys_err_flag);

            ci_logerr(CI_LOG_ERROR,"CI_SS_AST_STATE = %d\n",asr_state);

            _delay_10us_240M(1000);
            extern void dpmu_software_reset_system_config(void);
            dpmu_software_reset_system_config();
        }
        if(asr_warning)
        {
            ci_logwarn(CI_LOG_WARN,"CI_SS_ASR_WARNING = %d\n",asr_warning);
            ciss_set(CI_SS_ASR_WARNING,0);
        }
        if (!data_addr)
        {
            ci_logwarn(CI_LOG_WARN,"iisdma int too slow\n");
            continue;
        }

        #if USE_SOFT_AEC_REF
        uint32_t dst_addr = (uint32_t)&data_mic_buffer[buffer_cnt][div_num][0];

        memcpy((void*)dst_addr,(void*)data_addr,data_size);

        int ref_data_size = ref_data_block_size;
        uint32_t data_addr_ref = get_play_buffer_cur_addr(data_addr);
        if((pre_ref_addr != 0) && (data_addr_ref == 0))
        {
            memset((void*)data_ref_buffer,0,AUDIO_CAP_POINT_NUM_PER_FRM * DATA_VOICE_BUFFER_NUM * sizeof(int16_t));
        }
        pre_ref_addr = data_addr_ref;
        memcpy((void*)&data_ref_buffer[buffer_cnt][div_num][0],data_addr_ref,ref_data_size);

        int16_t* data_scale = (int16_t*)&data_ref_buffer[buffer_cnt][div_num][0];
        for(int n=0;n<ref_data_size/sizeof(int16_t);n++)
        {
            data_scale[n] /= 10;
        }
        
        if((AUDIO_CAP_DIV_NUM - 1) == div_num)
        #endif
        {
        audio_in_get_voice_t voice_info_str;
        voice_info_str.cha_num = 1;
        voice_info_str.channel_flag = 1;
            
            #if USE_SOFT_AEC_REF
            voice_info_str.addr1 = (uint32_t)&data_mic_buffer[buffer_cnt][0][0];
            #else
            voice_info_str.addr1 = data_addr;
            #endif

            #if IF_USE_ANOTHER_CODEC_TO_GET_REF
            voice_info_str.addr2 = data_addr_ref;
            voice_info_str.cha_num = 2;
            #else
            voice_info_str.cha_num = 1;
                #if USE_SOFT_AEC_REF
                voice_info_str.addr2 = (uint32_t)&data_ref_buffer[buffer_cnt][0][0];
                #else
                voice_info_str.addr2 = 0;
                #endif
            #endif
        voice_info_str.size = data_size;


            if(voice_energy_gate)
            {
                int16_t* voice_data = (int16_t*)data_addr;
                float voice_energy = 0.0f;
                for(int i=0;i<data_size/2;i++)
                {
                    voice_energy += (float)(voice_data[i] * voice_data[i]);
                }
                if(get_voice_energy_callback_fun)
                {
                    get_voice_energy_callback_fun(voice_energy/((float)(data_size/2)));
                }
            }

        
            send_voice_info_to_audio_in_manage(&voice_info_str);

        #if USE_SOFT_AEC_REF
            buffer_cnt++;
            buffer_cnt %= DATA_VOICE_BUFFER_NUM;

            div_num = 0;
        }
        else
        {
            div_num++;
            continue;
        #endif
        }

        
        iwdg_feed(IWDG);
    }
}


void print_nn_err_cnt(void)
{
    static int nn_err_cnt = 0;
    int cnt = ciss_get(CI_SS_NN_OUT_ERR_CNT);
    if(cnt != nn_err_cnt)
    {
        ci_logwarn(CI_LOG_WARN,"CI_SS_ASR_WARNING[%d] = %d\n", 400, nn_err_cnt);
        nn_err_cnt = cnt;
    }
}


__attribute__((weak)) void ci_doa_get_cb(int doa){}
__attribute__((weak)) void ci_pwk_get_cb(int db_val){}
__attribute__((weak)) int ci_doa_get(void){return 0;}
__attribute__((weak)) void sed_rslt_cb(void* data){}
__attribute__((weak)) void ci_bnpu_err_print(int bnpu_err_type) {}
#if USE_MFCC_DTW_SPK && SPK_USE_DUAL_CHIP_DENOISE
/* ── 双芯片NN降噪: 从IIS0(codec 0)接收CI1306降噪后PCM, 喂给MFCC说话人识别 ──
 * 独立任务, 与主MIC录音(audio_in_manage_inner_task / HOST_MIC codec 1)并行。
 * 两个codec各有独立信号量+队列(MAX_CODEC_NUM=2正好用满), 跨任务读取安全。
 *
 * 根本原因说明: cm_start_codec() 进入前先取 cm.semaphore 互斥锁; 若 CI1306
 * 未接, IIS0 SLAVE 的 iis_wait_load_ctrl() 死循环永不退出, 锁永久持有,
 * 导致所有后续 cm_start_codec(PLAY_CODEC_ID) 也永久阻塞 → 喇叭无声。
 * 修复: 先用 GPIO 检测 PA3(LRCK) 是否在切换, 仅 CI1306 已接时才调用
 * cm_start_codec(), 未接时任务循环等待, 不持有互斥锁, 播音不受影响。 */
extern void spk_iis0_slave_codec_registe(void);  /* impl in board file CI-D03GS02S.c */

/* 检测 CI1306 是否已接并驱动 LRCK:
 * 将 PA3 配为 GPIO 输入 + 下拉; CI1306 LRCK@16kHz 每 31μs 翻转一次,
 * 在 ~2ms 窗口内采样 40 次(每次间隔 ~50μs), 若同时观测到高低电平即认为有时钟。
 * 无 CI1306 时 PA3 被拉低, 只能看到 0 → 返回 false。 */
static bool spk_detect_ci1306_lrck(void)
{
    /* 按板级代码约定顺序: direction → pull → reuse → gpio_set_input_mode → read */
    scu_set_device_gate(PA, ENABLE);
    dpmu_set_io_direction(PA3, DPMU_IO_DIRECTION_INPUT);
    dpmu_set_io_pull(PA3, DPMU_IO_PULL_DOWN);
    dpmu_set_io_reuse(PA3, FIRST_FUNCTION);
    gpio_set_input_mode(PA, pin_3);   /* 必须: 将GPIO模块方向寄存器设为输入 */

    bool saw_high = false, saw_low = false;
    for (int i = 0; i < 40; i++)
    {
        if (gpio_get_input_level(PA, pin_3)) saw_high = true;
        else                                 saw_low  = true;
        if (saw_high && saw_low) break;
        _delay_10us_240M(5);   /* ~50μs, non-multiple of 62.5μs LRCK period */
    }
    dpmu_set_io_pull(PA3, DPMU_IO_PULL_DISABLE);
    return saw_high && saw_low;
}
void spk_iis0_rx_task(void *arg)
{
    (void)arg;

    /* 等主MIC录音启动完成, 确保codec框架就绪 */
    while (CI_SS_MIC_VOICE_NORMAL != ciss_get(CI_SS_MIC_VOICE_STATUE))
        vTaskDelay(pdMS_TO_TICKS(10));

    /* 检测 CI1306 LRCK; 未接时每 2s 重试, 不持有 cm.semaphore, 播音正常 */
    mprintf("[SPK] waiting for CI1306 LRCK on PA3...\n");
    while (!spk_detect_ci1306_lrck())
    {
        mprintf("[SPK] CI1306 not detected, retry in 2s\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    mprintf("[SPK] CI1306 LRCK detected, starting IIS0 slave\n");

    spk_iis0_slave_codec_registe();
    cm_start_codec(REF_RECORD_CODEC_ID, CODEC_INPUT);
    mprintf("[SPK] IIS0 rx_task started, feeding CI1306 denoised PCM to MFCC\n");

    int frame_cnt = 0;
    for (;;)
    {
        uint32_t data_addr = 0, data_size = 0;
        cm_read_codec(REF_RECORD_CODEC_ID, &data_addr, &data_size);
        if (!data_addr)
            continue;

        const short *pcm = (const short *)data_addr;
        int n_samples = (int)(data_size / sizeof(short));
        spk_feed_pcm(pcm, n_samples);

        if (++frame_cnt % 100 == 0)
        {
            int32_t peak = 0;
            for (int i = 0; i < n_samples; i++)
            {
                int32_t v = pcm[i] < 0 ? -pcm[i] : pcm[i];
                if (v > peak) peak = v;
            }
            mprintf("[SPK] IIS0 rx: frm=%d peak=%d n=%d\n",
                    frame_cnt, (int)peak, n_samples);
        }
    }
}
#endif /* USE_MFCC_DTW_SPK && SPK_USE_DUAL_CHIP_DENOISE */