/*
 * mid.c - MID 层初始化聚合
 * at_params 加载 + attitude 初始化 + 启动 200Hz 姿态采样.
 */
#include "mid.h"
#include "ti_msp_dl_config.h"
#include "at_params.h"
#include "attitude.h"

uint8_t mid_init(void)
{
    at_params_init();

    if (attitude_init() != 0) return 1;   /* IMU 初始化失败, 调用方报错 */

    /* 启动 200Hz 姿态采样: 使能 NVIC + 开始计数 (init 已配好外设中断和优先级) */
    NVIC_EnableIRQ(TIMER_ATT_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_ATT_INST);
    return 0;
}
