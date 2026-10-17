/*
 * bsp.c - BSP 层初始化聚合
 * 把所有 BSP 驱动的 init 集中调, system_init 只调 bsp_init 一个.
 */
#include "bsp.h"
#include "uart.h"
#include "led_buzzer.h"
#include "oled.h"
#include "key.h"
#include "Encoder.h"
#include "Motor.h"
#include "step_motor.h"

void bsp_init(void)
{
    uart_camera_init();
    uart_communicate_init();  /* UART2: K230 or PID tuning, selected in uart.h */
    led_buzzer_init();
    OLED_Init();
    key_init();
    encoder_init();
    motor_init();
    Step_Init();
    step_off(X);       /* 步进上电停转 */
    step_off(Y);
}
