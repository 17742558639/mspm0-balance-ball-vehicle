/*
 * system.c - 系统初始化与主循环
 *
 * system_init: 按层聚合调用 bsp_init / mid_init / control_init + 应用层 init.
 * system_run:  主循环 - 按键派发 + 控制任务 + 菜单刷新 + 实时页/心跳软周期.
 * main.c 只调这两个函数.
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "mid.h"
#include "oled.h"
#include "delay.h"
#include "app_menu.h"
#include "key.h"
#include "Control.h"
#include "scheduler.h"
#include "uart.h"
#include "ball.h"
#include "ball_target_task.h"

/* 按键回调: 转发菜单 + K3(BACK) 长按切换运行状态.
 * 长按不易误触; 若与菜单 BACK 长按冲突, 改为菜单项触发. */
static void on_key(uint8_t id, Key_Event event)
{
    app_menu_on_key(id, event);
    if (id == 3 && event == KEY_EVENT_LONG_PRESS) 
    {
        BallTargetTask_stop();
        control_toggle_run();
    }
}

void system_init(void)
{
    bool imu_ok;

    SYSCFG_DL_init();
    bsp_init();

    /* MID: at_params 加载 + attitude 初始化(含零偏标定 ~2.25s) + 启动 200Hz 采样.
     * 期间板子须静止. 失败 -> 显示错误 + 死循环闪灯 (不可继续). */
    imu_ok = (mid_init() == 0U);
    if (!imu_ok) {
        OLED_Clear();
        OLED_PrintfRC(0, 0, "IMU init FAIL");
        OLED_Update();
        /*
        while (1) {
            DL_GPIO_togglePins(GPIO_CORE_LED_PORT, GPIO_CORE_LED_CORE_LED_PIN);
            delay_ms(500);
        }
        */
    }

    /* 菜单: 建页面表 + 初始化上下文 (须在 OLED_Init 之后, OLED_Init 在 bsp_init 里) */
    app_menu_init();

    /* 按键回调注册 (key_init 在 bsp_init 里) */
    for (uint8_t i = 0; i < KEY_NUM; i++)
        key_set_callback(i, on_key);

    /* 控制: PID 参数 + 启动 100Hz 控制环. start_run=false, 上电不跑 */
    control_init();
    control_set_motion_enabled(imu_ok);
    ball_init();
    BallTargetTask_init();
}

void system_run(void)
{
    int last_live = 0;   /* 上次实时刷新的 g_tms */
    int last_led  = 0;   /* 上次心跳翻转的 g_tms */

    while (1) 
    {
        key_process();          /* 派发按键 -> on_key -> menu_handle_event / control_toggle_run */
        control_task();         /* 小车控制层主循环任务 (非 ISR) */
        BallTargetTask_update((uint32_t)g_tms);

        app_menu_refresh();     /* dirty 才刷 OLED, 否则空过 (零成本) */

        /* 实时数据页 (IMU Roll/Pitch/Yaw) 周期刷新: 每 100ms 置脏一次 */
        if (g_tms - last_live >= 100) 
        {
            last_live = g_tms;
            app_menu_invalidate();
        }

        /* 心跳: 时间驱动, 不再 delay_ms 阻塞 */
        if (g_tms - last_led >= 1000) 
        {
            last_led = g_tms;
            DL_GPIO_togglePins(GPIO_CORE_LED_PORT, GPIO_CORE_LED_CORE_LED_PIN);
            uint8_t data[4] = {2, 0, 0, 0};
            uart_camera_send_frame(0x01, data);
        }
    }
}
