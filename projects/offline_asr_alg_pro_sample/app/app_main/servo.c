#include "servo.h"
#include "ci130x_pwm.h"
#include "ci130x_scu.h"    /* scu_set_device_gate, FIFTH_FUNCTION */
#include "ci130x_dpmu.h"   /* dpmu_set_io_reuse */
#include "ci_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define DUTY_MAX    20000u
#define DUTY_0DEG   1000u
#define DUTY_180DEG 2000u

static volatile int   s_pending_door;
static QueueHandle_t  s_servo_q;

/* 实际驱动 PWM 输出到指定角度 (仅在 servo_init 和 servo_task 中调用, 单线程访问PWM) */
static void servo_apply_angle(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    unsigned int duty = DUTY_0DEG +
        (unsigned int)((DUTY_180DEG - DUTY_0DEG) * (unsigned int)angle / 180u);
    pwm_set_duty(SERVO_PWM_CH, duty, DUTY_MAX);
    mprintf("[SERVO] >>> set_angle: angle=%d duty=%u (pulse=%uus)\n",
            angle, duty, duty);
}

/* 舵机任务: 收到目标角度 -> 转到该角度 -> 保持300ms -> 复位到45° */
static void servo_task(void *p)
{
    (void)p;
    int angle;
    for (;;) {
        if (xQueueReceive(s_servo_q, &angle, portMAX_DELAY) == pdTRUE) {
            servo_apply_angle(angle);                   /* 转到目标角度 */
            vTaskDelay(pdMS_TO_TICKS(SERVO_HOLD_MS));    /* 保持 300ms */
            servo_apply_angle(SERVO_ANGLE_REST);         /* 复位到 45° */
            mprintf("[SERVO] hold %dms done, reset to rest(%d)\n",
                    (int)SERVO_HOLD_MS, SERVO_ANGLE_REST);
        }
    }
}

void servo_init(void)
{
    /* 关键1: 使能 PWM2 时钟门控 */
    scu_set_device_gate((uint32_t)SERVO_PWM_CH, ENABLE);
    /* 关键2: PA4 复用为 PWM2 输出 —— 必须用 FIFTH_FUNCTION, 不是 SECOND_FUNCTION! */
    dpmu_set_io_reuse(SERVO_PWM_PAD, SERVO_PWM_PAD_FUNC);

    pwm_init_t cfg;
    cfg.clk_sel  = 0;          /* PCLK */
    cfg.freq     = 50;         /* 50Hz */
    cfg.duty     = DUTY_0DEG;
    cfg.duty_max = DUTY_MAX;
    pwm_init(SERVO_PWM_CH, cfg);
    pwm_set_restart_md(SERVO_PWM_CH, 1);
    pwm_start(SERVO_PWM_CH);

    /* 开机复位到 45° (队列/任务创建前直接驱动, 不经队列) */
    servo_apply_angle(SERVO_ANGLE_REST);
    s_pending_door = 0;

    /* 创建舵机任务 + 队列 (用于"保持后自动复位", 避免阻塞ASR/SPK回调) */
    s_servo_q = xQueueCreate(4, sizeof(int));
    xTaskCreate(servo_task, "servo", 512, NULL, 3, NULL);

    mprintf("[SERVO] init done: ch=0x%x pad=PA4 func=%d freq=50Hz rest=%d\n",
            (unsigned)SERVO_PWM_CH, (int)SERVO_PWM_PAD_FUNC, SERVO_ANGLE_REST);
}

/* 立即设定角度并保持 (不自动复位) — 兼容保留 */
void servo_set_angle(int angle)
{
    servo_apply_angle(angle);
}

/* 转到 angle, 保持 SERVO_HOLD_MS 后自动复位到 REST(45°) */
void servo_pulse_to(int angle)
{
    if (s_servo_q) {
        if (xQueueSend(s_servo_q, &angle, 0) != pdTRUE)
            mprintf("[SERVO] queue full, drop angle=%d\n", angle);
    } else {
        /* 任务未就绪时退化为直接设定 */
        servo_apply_angle(angle);
    }
}

void servo_set_pending_door(int pending)
{
    s_pending_door = pending;
    mprintf("[SERVO] pending_door <- %d\n", pending);
}

int servo_get_pending_door(void)
{
    return s_pending_door;
}