#ifndef SERVO_H
#define SERVO_H

#include "ci130x_pwm.h"
#include "ci130x_dpmu.h"   /* PinPad_Name (PA4), dpmu_set_io_reuse */
#include "ci130x_scu.h"    /* IOResue_FUNCTION (FIFTH_FUNCTION) */

/* ====================== 用户配置区 ====================== */
#define SERVO_PWM_CH        PWM2            /* 舵机 PWM 通道 */
#define SERVO_PWM_PAD       PA4             /* 舵机信号脚 = PA4 */
#define SERVO_PWM_PAD_FUNC  FIFTH_FUNCTION  /* PA4 第5功能=PWM2 (查ci130x_scu.h引脚表) */

#define SERVO_ANGLE_A       0               /* "小屁锁车" 触发角度 */
#define SERVO_ANGLE_B       90              /* "小屁开门" 声纹通过后角度 */
#define SERVO_ANGLE_REST    45              /* 复位/开机角度 */
#define SERVO_HOLD_MS       300             /* 触发后保持时间(ms), 之后回到REST */

#define SERVO_CMD_OPEN_DOOR 1
#define SERVO_CMD_LOCK_CAR  5
/* ====================== end ============================ */

void servo_init(void);
void servo_set_angle(int angle);   /* 立即设定角度并保持(不自动复位), 兼容保留 */
void servo_pulse_to(int angle);    /* 转到angle, 保持SERVO_HOLD_MS后自动复位到REST */
void servo_set_pending_door(int pending);
int  servo_get_pending_door(void);

#endif