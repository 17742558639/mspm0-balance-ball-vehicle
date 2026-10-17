/*
 * scheduler.c - 系统节拍 (SysTick 1ms)
 * g_tms 自增 + 每 KEY_SCAN_MS(10ms) 调一次 key_scan.
 * 原在 main.c, Step 3 搬到本模块, 逻辑不变.
 */
#include "scheduler.h"
#include "key.h"

volatile int g_tms = 0;

void SysTick_Handler(void)
{
    g_tms++;
    /* SysTick=1ms, 每 KEY_SCAN_MS(10ms) 扫一次按键 */
    static uint8_t key_div = 0;
    if (++key_div >= KEY_SCAN_MS) {
        key_div = 0;
        key_scan();
    }
}
