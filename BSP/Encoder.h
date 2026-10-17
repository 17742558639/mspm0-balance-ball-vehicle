#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* ========================= 编码器驱动 (后轮 bL/bR) =========================
 * 2 路增量编码器, A 相 GPIO 上升沿中断计数, B 相判方向.
 * 移植自队友工程, 精简: 只保留后轮 bL/bR (前轮 al/ar 已删).
 *
 * 引脚 (syscfg):
 *   GPIO_ENCODER_BL: PIN_BL_A=PB20 (中断 RISE), PIN_BL_B=PA24 (方向) -> 左后 (跨端口)
 *   GPIO_ENCODER_BR: PIN_BR_A=PB0  (中断 RISE), PIN_BR_B=PA13 (方向) -> 右后 (跨端口)
 *
 * 中断: 两个 A 相都在 GPIOB, 共享 GROUP1 (GPIOB_INT_IRQn=1) 中断.
 *       ISR (GROUP1_IRQHandler) 在本模块 (Encoder.c) 实现.
 *       bL/bR 的 B 相都在 GPIOA, 读方向用 per-pin _B_PORT 宏 (非组级 _PORT).
 *
 * 用法:
 *   encoder_init();          // 使能 GPIOB NVIC (GPIO 模块级中断 syscfg 已配)
 *   // ISR 里 encoder_bl/br 自增/减
 *   // get_speed() 在 Control.c: 读 encoder_bl/br -> *_speed, 清零
 */

extern volatile int16_t encoder_bl;
extern volatile int16_t encoder_br;
extern volatile int16_t encoder_bl_speed;
extern volatile int16_t encoder_br_speed;
extern volatile int32_t encoder_step;

void encoder_init(void);

#endif /* __ENCODER_H */
