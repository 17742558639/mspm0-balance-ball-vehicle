#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

/* ========================= 步进电机驱动 =========================
 * 2 路步进电机, PWM 脉冲 + 方向 GPIO. PWM CC0_DN 中断计步, 到数自动停.
 * 移植自队友工程, 宏名适配新 syscfg.
 *
 * 引脚 (syscfg):
 *   STEP_MOTOR_1 = TIMA1 CCP0 = PA10  (Y 轴, step1)
 *   STEP_MOTOR_2 = TIMG7 CCP0 = PA23  (X 轴, step2)
 *   GPIO_STEP_MOTOR: DIR1=PB13, DIR2=PA25, EN1=PB1, EN2=PB24
 *
 * 用法:
 *   Step_Init();
 *   step_on(步数, X);   // 正数正转, 负数反转, 0 停
 *   step_off(X);        // 立即停
 */

#define X  2
#define Y  1

/* 第一路步进电机的滚球控制参数。旧的 X/Y 接口继续保留。 */
#define STEP_SPEED_MIN_HZ       200
#define STEP_SPEED_MAX_HZ     12000
#define STEP_ENCODER_PPR       1024
#define STEP_MICROSTEPS          64
#define STEP_PULSES_PER_REV    (200 * STEP_MICROSTEPS)

extern volatile uint32_t step1_count;
extern volatile uint32_t step2_count;
extern int step1_sum;
extern int step2_sum;

void  Step_Init(void);
void  step_off(uint8_t id);
void  step_on(int step_num, uint8_t id);
void step_on_at_freq(int32_t step_num, int32_t hz);
void step_drive_hz(int32_t hz);

#endif /* __STEP_MOTOR_H */
