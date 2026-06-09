#include "servo.h"
#include "ci130x_pwm.h"
#include "ci130x_scu.h"    /* scu_set_device_gate, FIFTH_FUNCTION */
#include "ci130x_dpmu.h"   /* dpmu_set_io_reuse */
#include "ci_log.h"

#define DUTY_MAX    20000u
#define DUTY_0DEG   1000u
#define DUTY_180DEG 2000u

static volatile int s_pending_door;

void servo_init(void)
{
    /* 关键1: 使能 PWM2 时钟门控 */
    scu_set_device_gate((uint32_t)SERVO_PWM_CH, ENABLE);
    /* 关键2: PA4 复用为 PWM2 输出 —— 必须用 FIFTH_FUNCTION, 不是 SECOND_FUNCTION! */
    dpmu_set_io_reuse(SERVO_PWM_PAD, SERVO_PWM_PAD_FUNC);

    pwm_init_t cfg;
    cfg.clk_sel  = 0;          /* PCLK */
    cfg.freq     = 50;         /* 50Hz */
    cfg.duty     = DUTY_0DEG;  /* 初始 0° */
    cfg.duty_max = DUTY_MAX;
    pwm_init(SERVO_PWM_CH, cfg);
    pwm_set_restart_md(SERVO_PWM_CH, 1);
    pwm_start(SERVO_PWM_CH);
    s_pending_door = 0;
    mprintf("[SERVO] init done: ch=0x%x pad=PA4 func=%d freq=50Hz\n",
            (unsigned)SERVO_PWM_CH, (int)SERVO_PWM_PAD_FUNC);
}

void servo_set_angle(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    unsigned int duty = DUTY_0DEG +
        (unsigned int)((DUTY_180DEG - DUTY_0DEG) * (unsigned int)angle / 180u);
    pwm_set_duty(SERVO_PWM_CH, duty, DUTY_MAX);
    mprintf("[SERVO] >>> set_angle: angle=%d duty=%u (pulse=%uus)\n",
            angle, duty, duty);
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