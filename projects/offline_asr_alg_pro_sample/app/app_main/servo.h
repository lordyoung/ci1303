#ifndef SERVO_H
#define SERVO_H

#include "ci130x_pwm.h"

/* ====================== 用户配置区 ====================== */
#define SERVO_PWM_CH        PWM2    /* 舵机信号线接的 PWM 通道，按原理图改 PWM0~PWM5 */
#define SERVO_ANGLE_A       90      /* "小屁锁车" 目标角度 (0~180°) */
#define SERVO_ANGLE_B       0       /* "小屁开门" 声纹通过后目标角度 (0~180°) */

#define SERVO_CMD_OPEN_DOOR 1       /* 小屁开门 cmd_id（与 user_msg_deal.c send_data[] 一致）*/
#define SERVO_CMD_LOCK_CAR  5       /* 小屁锁车 cmd_id */
/* ====================== end 用户配置 ==================== */

void servo_init(void);
void servo_set_angle(int angle);            /* angle: 0~180° */
void servo_set_pending_door(int pending);   /* 标记是否在等"小屁开门"的声纹结果 */
int  servo_get_pending_door(void);

#endif /* SERVO_H */