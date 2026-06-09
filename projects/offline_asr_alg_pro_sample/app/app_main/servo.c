#include "servo.h"
#include "ci130x_pwm.h"
#include "ci_log.h"

/* 标准舵机 50Hz：周期 20ms，duty_max=20000 刻度（1 刻度=1us）
 *   1.0ms(1000) → 0° ; 2.0ms(2000) → 180°
 * 若舵机行程不同（如 0.5~2.5ms），改 DUTY_0DEG / DUTY_180DEG（500/2500） */
#define DUTY_MAX    20000u
#define DUTY_0DEG   1000u
#define DUTY_180DEG 2000u

static volatile int s_pending_door;

void servo_init(void)
{
    /* 注意：调用前需已在 user_control.c / board.c 完成：
     *  1) PWM 时钟门控使能   2) 对应 GPIO 引脚复用为 PWM 输出
     * 具体 API 与引脚见 CI1303 datasheet 引脚复用表及 board.c 示例 */
    pwm_init_t cfg;
    cfg.clk_sel  = 0;          /* 0=PCLK */
    cfg.freq     = 50;         /* 50Hz */
    cfg.duty     = DUTY_0DEG;  /* 上电初始 0° */
    cfg.duty_max = DUTY_MAX;
    pwm_init(SERVO_PWM_CH, cfg);
    pwm_start(SERVO_PWM_CH);
    s_pending_door = 0;
    mprintf("[SERVO] init done: ch=%d freq=50Hz duty_max=%u init_duty=%u\n",
            (int)SERVO_PWM_CH, DUTY_MAX, DUTY_0DEG);
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