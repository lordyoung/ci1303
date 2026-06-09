#include "user_control.h"

//GPIO init
void GPIO_control_init(void)
{
    ///tag-user-defined-gpio-init-by-pin-num-start

    ///"tag-user-defined-gpio-init-by-pin-num-end
}

// 输出高低电平
// void gpio_set_output_level_single(gpio_base_t gpio,gpio_pin_t pins,uint8_t level);

// 获取输入电平
// uint8_t gpio_get_input_level_single(gpio_base_t gpio,gpio_pin_t pins);


//pwn init
void pwm_control_init(void)
{
    pwm_init_t pwm_config;
    ///tag-user-defined-pwm-init-by-pin-num-start

    /* 步骤1：使能 PWM 时钟门控（示例，实际 API 参考 board.c）
     *   scu_set_device_gate(HAL_PWM0_BASE, ENABLE);              */
    /* 步骤2：配置 GPIO 引脚复用为 PWM 输出（按原理图选引脚）
     *   例如 dpmu_padmux_config(PAD_GPIOxx, PADMUX_PWM0);        */

    mprintf("[SERVO] pwm_control_init enter\n");
    servo_init();   /* 初始化舵机 PWM 并启动输出 */

    ///tag-user-defined-pwm-init-by-pin-num-end
}

void user_pin_control_init(void)
{
    GPIO_control_init();
    pwm_control_init();
}
