#ifndef MID_H
#define MID_H

#include <stdint.h>

/*
 * ========================= MID 层初始化聚合 =========================
 * mid_init(): at_params 加载 + attitude 初始化(含零偏标定 ~2.25s) + 启动 200Hz 姿态采样.
 *             返回 0=成功, 1=attitude(IMU) 初始化失败 (调用方负责报错显示).
 *             须在 SYSCFG_DL_init() + bsp_init() 之后调 (OLED 已就绪可用于报错).
 *             期间板子须静止 (零偏标定).
 */

uint8_t mid_init(void);

#endif /* MID_H */
