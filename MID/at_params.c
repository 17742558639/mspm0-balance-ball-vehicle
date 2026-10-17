/*
 * at_params.c - 简易参数持久化实现 (AT24C02, 独立软件 I2C 总线 PA26/PA30).
 *
 * 总线与 IMU 硬件 I2C1、OLED 软件 I2C 都独立, 无竞争: 存取在主循环阻塞 ~14ms,
 * 200Hz 姿态 ISR 不受影响.
 *
 * 完整性: 仅 1 字节 magic 防 0xFF 当 float 读成 NaN; 无 CRC/版本 (调试用, 够简单).
 *   代价: 表新增变量后, 该变量首次会读到 EEPROM 旧残留, 手动编辑存一次即好.
 */
#include "at_params.h"
#include "at24c02.h"

#define AT_MAGIC_ADDR   255u   /* 末字节做"已初始化"标志 */
#define AT_MAGIC_VAL    0xA5u

/* ---- 持久化变量定义 ---- */
float    g_demo_kp    = 0.5f;
uint16_t g_demo_speed = 500U;

/* ---- 持久化表: 要存什么就加一行 {&var, addr, type} ----
 * addr 手动分配 (0, 4, 6, ...), 留意 AT24C02 共 256B、8 字节页边界. */
const AT_Param at_params[] = {
    {&g_demo_kp,    0u, AT_PARAM_FLOAT},
    {&g_demo_speed, 4u, AT_PARAM_UINT16},
};

static uint8_t at_size(AT_ParamType t)
{
    switch (t) {
    case AT_PARAM_FLOAT:  return 4u;
    case AT_PARAM_UINT16: return 2u;
    case AT_PARAM_INT:    return 4u;
    default:              return 0u;
    }
}

/* volatile 安全的字节拷贝: 变量可能被 ISR 读 (如后续 mahony_kp), 不能裸 cast 丢 volatile. */
static void at_copy_from_var(uint8_t *dst, volatile void *src, uint8_t sz)
{
    volatile uint8_t *s = (volatile uint8_t *)src;
    for (uint8_t i = 0u; i < sz; i++) {
        dst[i] = s[i];
    }
}

static void at_copy_to_var(volatile void *dst, const uint8_t *src, uint8_t sz)
{
    volatile uint8_t *d = (volatile uint8_t *)dst;
    for (uint8_t i = 0u; i < sz; i++) {
        d[i] = src[i];
    }
}

void at_params_init(void)
{
    AT24C02_Init();

    uint8_t magic;
    if (AT24C02_ReadByte(AT_MAGIC_ADDR, &magic) != AT24C02_OK || magic != AT_MAGIC_VAL) {
        return;   /* 首次上电/损坏: 不加载, 保留编译初值 */
    }

    uint8_t buf[8];
    uint16_t count = (uint16_t)(sizeof(at_params) / sizeof(at_params[0]));
    for (uint16_t i = 0u; i < count; i++) {
        uint8_t sz = at_size(at_params[i].type);
        if (sz != 0u &&
            AT24C02_Read(at_params[i].addr, buf, (uint16_t)sz) == AT24C02_OK) {
            at_copy_to_var(at_params[i].var, buf, sz);
        }
    }
}

void at_params_save_var(volatile void *addr)
{
    uint8_t buf[8];
    uint16_t count = (uint16_t)(sizeof(at_params) / sizeof(at_params[0]));

    for (uint16_t i = 0u; i < count; i++) {
        if (at_params[i].var == addr) {
            uint8_t sz = at_size(at_params[i].type);
            if (sz != 0u) {
                at_copy_from_var(buf, at_params[i].var, sz);
                if (AT24C02_Write(at_params[i].addr, buf, (uint16_t)sz) == AT24C02_OK) {
                    (void)AT24C02_WriteByte(AT_MAGIC_ADDR, AT_MAGIC_VAL);
                }
            }
            return;   /* 找到并处理完即返回; 找不到 (addr 不在表里) 也走到这里不操作 */
        }
    }
}
