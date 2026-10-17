#ifndef __IMU_SELECT_H
#define __IMU_SELECT_H

/*
 * ========================= IMU 选型切换 =========================
 * 编译期二选一: imu660rb (LSM6DSR) 或 mpu6050_raw (MPU6050, 自写非 DMP).
 * 两者共用 I2C_IMU 总线 (PB2/PB3), 同一时刻只编译一个驱动.
 * attitude.c 的 Mahony 算法对两者通用, 通过本头文件的抽象宏访问数据源.
 *
 * 切换方法: 改下面的 IMU_SELECT 宏.
 *   IMU_USE_IMU660RB  -> 用 imu660rb (默认)
 *   IMU_USE_MPU6050   -> 用 mpu6050 (自写, 非 DMP)
 *
 * 各 IMU 的轴方向符号 IMU_AXIS_*_SIGN 也在此按选型配置
 * (两个 IMU 安装方向可能不同, 实测调整对应分支).
 */
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define IMU_USE_MPU6050    1
#define IMU_USE_IMU660RB   2

#define IMU_SELECT         IMU_USE_MPU6050   /* <-- 改这行切换 IMU */


#if IMU_SELECT == IMU_USE_MPU6050

#include "mpu6050_raw.h"

/* 驱动接口 */
#define imu_dev_init()        mpu6050_init()
#define imu_dev_get_acc()     mpu6050_get_acc()
#define imu_dev_get_gyro()    mpu6050_get_gyro()

/* 原始数据变量 */
#define imu_acc_x             mpu6050_acc_x
#define imu_acc_y             mpu6050_acc_y
#define imu_acc_z             mpu6050_acc_z
#define imu_gyro_x            mpu6050_gyro_x
#define imu_gyro_y            mpu6050_gyro_y
#define imu_gyro_z            mpu6050_gyro_z

/* 原始值 -> 物理量 (acc: g, gyro: °/s) */
#define imu_acc_transition    mpu6050_acc_transition
#define imu_gyro_transition   mpu6050_gyro_transition

/* 轴方向符号 (MPU6050 安装方向, 实测调整) */
#define IMU_AXIS_X_SIGN       (+1.0f)
#define IMU_AXIS_Y_SIGN       (+1.0f)
#define IMU_AXIS_Z_SIGN       (+1.0f)


#elif IMU_SELECT == IMU_USE_IMU660RB

#include "imu660rb.h"

#define imu_dev_init()        imu660rb_init()
#define imu_dev_get_acc()     imu660rb_get_acc()
#define imu_dev_get_gyro()    imu660rb_get_gyro()

#define imu_acc_x             imu660rb_acc_x
#define imu_acc_y             imu660rb_acc_y
#define imu_acc_z             imu660rb_acc_z
#define imu_gyro_x            imu660rb_gyro_x
#define imu_gyro_y            imu660rb_gyro_y
#define imu_gyro_z            imu660rb_gyro_z

#define imu_acc_transition    imu660rb_acc_transition
#define imu_gyro_transition   imu660rb_gyro_transition

/* 轴方向符号 (imu660rb 实测正装: 三轴 +1) */
#define IMU_AXIS_X_SIGN       (+1.0f)
#define IMU_AXIS_Y_SIGN       (+1.0f)
#define IMU_AXIS_Z_SIGN       (+1.0f)


#else
#error "IMU_SELECT must be IMU_USE_MPU6050 or IMU_USE_IMU660RB"
#endif

#endif /* __IMU_SELECT_H */
