#ifndef __MPU6050_RAW_H
#define __MPU6050_RAW_H

#include <stdint.h>

/*
 * ====================== MPU6050 / MPU6500 驱动 (复用 attitude) ======================
 * 芯片: MPU6050 或 MPU6500 (6 轴 = 加速度计 + 陀螺仪).
 * 不用 DMP, 纯寄存器突发读取. 两者基础数据寄存器、量程和换算系数兼容.
 * 接口: 硬件 I2C (syscfg 实例 I2C_IMU, SCL=PB2, SDA=PB3, 400kHz) -- 与 imu660rb 共用.
 *
 * 接线:
 *   SCL  -> PB2
 *   SDA  -> PB3
 *   AD0  -> GND   (地址 0x68; 接 VCC 则 0x69, 需改 MPU6050_DEV_ADDR)
 *   VCC  -> 3.3V  (注意: 部分 GY-521 模块自带稳压可接 5V, 裸芯片必须 3.3V)
 *   GND  -> GND
 *   INT  -> 悬空  (自写方案不使用外部中断, 定时器轮询读取)
 *
 * 与 imu660rb 的关系: 两者共用 I2C_IMU 总线, 通过 imu_select.h 的 IMU_SELECT 宏
 * 在编译期二选一. attitude.c 姿态算法对两者通用, 只换底层驱动.
 *
 * 量程: 加速度 ±8G (1g = 4096 LSB), 陀螺仪 ±2000dps (1°/s = 16.4 LSB).
 * 采样率 200Hz (匹配 attitude 的 ATTITUDE_SAMPLE_HZ).
 *
 * 数据字节序: MPU6050 大端 (高字节在前), 与 imu660rb (LSM6DSR, 小端) 相反,
 * 拼接时注意高低字节顺序.
 *
 * 用法 (一般通过 imu_select.h 间接调用, 不直接 include 本文件):
 *   if (mpu6050_init() != 0) { ... 失败处理 ... }   // 0=成功
 *   mpu6050_get_gyro();                              // -> mpu6050_gyro_x/y/z
 *   mpu6050_get_acc();                               // -> mpu6050_acc_x/y/z
 *   float gx = mpu6050_gyro_transition(mpu6050_gyro_x);  // 原始值 -> °/s
 */

#define MPU6050_DEV_ADDR          0x68    /* 7 位 I2C 地址 (AD0=GND 时) */
#define MPU6050_WHO_AM_I_VAL      0x68    /* MPU6050 的 WHO_AM_I 值 */
#define MPU6500_WHO_AM_I_VAL      0x70    /* MPU6500 的 WHO_AM_I 值 */

/* 当前工程的量程配置，适配层和姿态算法共用，避免重复写死换算系数。 */
#define MPU6050_ACCEL_LSB_PER_G   (4096.0f)  /* ±8 g */
#define MPU6050_GYRO_LSB_PER_DPS  (16.4f)   /* ±2000 dps */

/* 原始数据 (LSB, 有符号 16 位). 在 200 Hz 姿态 ISR 中更新，主循环只读快照。 */
extern volatile int16_t mpu6050_acc_x;
extern volatile int16_t mpu6050_acc_y;
extern volatile int16_t mpu6050_acc_z;
extern volatile int16_t mpu6050_gyro_x;
extern volatile int16_t mpu6050_gyro_y;
extern volatile int16_t mpu6050_gyro_z;
/* 读取一组加速度数据后递增；用于主循环做无锁一致性快照。 */
extern volatile uint32_t mpu6050_raw_sequence;
extern volatile uint8_t mpu6050_last_acc_read_ok;
extern volatile uint8_t mpu6050_last_gyro_read_ok;

/* 初始化时读到的 WHO_AM_I，便于调试器确认实际芯片: 0x68=MPU6050, 0x70=MPU6500. */
extern volatile uint8_t mpu6050_device_id;

/* 初始化: 软复位 -> WHO_AM_I 自检 -> 配置量程/采样率/滤波. 返回 0=成功, 1=失败 */
uint8_t mpu6050_init(void);

/* 读加速度计, 结果存 mpu6050_acc_x/y/z */
void mpu6050_get_acc(void);

/* 读陀螺仪, 结果存 mpu6050_gyro_x/y/z */
void mpu6050_get_gyro(void);

/* 原始值 -> 物理量. acc: g(≈9.8m/s^2), gyro: °/s
 * ±8G -> 1g = 4096 LSB; ±2000dps -> 1°/s = 16.4 LSB (datasheet) */
#define mpu6050_acc_transition(v)   ((float)(v) / MPU6050_ACCEL_LSB_PER_G)
#define mpu6050_gyro_transition(v)  ((float)(v) / MPU6050_GYRO_LSB_PER_DPS)

#endif /* __MPU6050_RAW_H */
