#ifndef AT_PARAMS_H
#define AT_PARAMS_H

#include <stdint.h>

/*
 * at_params - 简易参数持久化 (调试用, 基于 AT24C02).
 *
 * 设计: 一张 at_params[] 表, 每行 {变量指针, EEPROM地址, 类型}.
 *   - 上电 at_params_init() 从 EEPROM 加载一次.
 *   - 菜单编辑提交时调 at_params_save_var(addr) 存该变量.
 *   - 要持久化谁, 就在 at_params[] 加一行, 菜单不用改.
 * 与菜单解耦: 菜单只调 at_params_save_var, 不感知表内容.
 *
 * 比赛时不想依赖 EEPROM: 把 main.c 里 at_params_init() 注释掉即可,
 * 变量退回编译初值, 运行不依赖存储芯片.
 */

typedef enum {
    AT_PARAM_FLOAT  = 0,
    AT_PARAM_UINT16,
    AT_PARAM_INT,
    /* 需要别的类型在这里追加 */
} AT_ParamType;

typedef struct {
    volatile void *var;   /* 要持久化的变量 (volatile: 可能被 ISR 读) */
    uint8_t        addr;  /* EEPROM 字节地址 (手动分配, 勿重叠, AT24C02 共 256B) */
    AT_ParamType   type;
} AT_Param;

/* 持久化变量定义在 at_params.c (后续 mahony_kp 等也放那里) */
extern float    g_demo_kp;
extern uint16_t g_demo_speed;

/* 持久化表: 要存什么就加一行 {&var, addr, type} */
extern const AT_Param at_params[];

/* 上电调一次: 从 EEPROM 加载所有变量. 首次(未初始化)跳过, 保留编译初值. */
void at_params_init(void);

/* 存指定变量: 表里找 var==addr 的项写 EEPROM; 找不到不操作. 菜单 commit 回调里调. */
void at_params_save_var(volatile void *addr);

#endif /* AT_PARAMS_H */
