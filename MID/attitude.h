#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

/*
 * ========================= 姿态解算模块 (Fusion AHRS) =========================
 *
 * 基于 xioTechnos Fusion 库 (Seb Madgwick, Mahony 变体) 移植. 相比手写 Mahony:
 *   - 完整 3D 叉乘反馈 (含 ez), 大倾角修正正确;
 *   - roll/pitch/yaw 从同一四元数提取, 运动学一致;
 *   - accelerationRejection + recovery 机制替代手写门限, 优雅拒运动干扰;
 *   - ramped gain 启动收敛, 不用手动 acc 初始化 q.
 *
 * 功能:
 *   roll/pitch/yaw -- Fusion AHRS 融合陀螺+加速度计, 输出 ZYX 欧拉角 (度).
 *   无磁力计, yaw 会缓慢漂移 (~2°/min), 运动学仍一致.
 *
 * 坐标系: ENU (X前 / Y右 / Z上), 静止 az≈+1g.
 *
 * 硬件依赖:
 *   - IMU 驱动 (imu660rb 或 mpu6050_raw, 由 imu_select.h 的 IMU_SELECT 切换), 硬件 I2C.
 *   - 定时器实例 (TIMER_ATT, 200Hz), 在其 IRQHandler 调 update_isr.
 *
 * 用法:
 *   if (attitude_init() != 0) { 失败处理 }
 *   NVIC_EnableIRQ(TIMER_ATT_INST_INT_IRQN);
 *   DL_TimerG_startCounter(TIMER_ATT_INST);
 *   // 主循环读 attitude.roll / .pitch / .yaw (度)
 *
 * 注意:
 *   - 上电标定期间(~2.25s)板子须静止, 否则零偏偏大, yaw 长期漂.
 *   - pitch=±90° 是 ZYX 欧拉角奇异点 (gimbal lock), 附近 roll/yaw 会耦合跳变,
 *     数学特性无法消除; 需大倾角场景改用四元数 (FusionAhrsGetQuaternion).
 *   - 调试记录见 docs/姿态解算调试记录.md.
 */

/* ---- 采样配置 ---- */
#define ATTITUDE_SAMPLE_HZ        200.0f
#define ATTITUDE_DT               (1.0f / ATTITUDE_SAMPLE_HZ)

/* ---- Fusion AHRS 参数 (调参改这里) ----
 * gain: 互补滤波增益, 大->加速度计权重高(响应快但噪声大). 典型 0.5.
 * gyroscopeRange: 陀螺量程(dps), 超出触发自动 reset. 0=禁用.
 * accelerationRejection: 加速度误差阈值(度), 超过则不信加速度计(防运动干扰). 0=禁用.
 * recoveryTriggerPeriod: 加速度持续异常多少采样后强制恢复接受 (采样数). */
#define FUSION_GAIN               0.5f
#define FUSION_GYRO_RANGE_DPS     2000.0f
#define FUSION_ACC_REJECTION_DEG  10.0f
#define FUSION_RECOVERY_PERIOD    500

/* ---- 零偏标定: 丢前 N 帧瞬态, 再采 N 帧求平均 ---- */
#define GYRO_CALIB_DISCARD        50       /* 丢前 50 帧 (~0.25s) */
#define GYRO_CALIB_FRAMES         400      /* 采 400 帧 (2s) */

/* ---- 姿态输出 (度) ---- */
typedef struct {
    float roll;      /* 绕 X 轴, 右倾为正, ±180° */
    float pitch;     /* 绕 Y 轴, 抬头为正, ±90°  */
    float yaw;       /* 绕 Z 轴, 右偏为正, ±180° (无磁, 会漂) */
} attitude_t;

extern attitude_t attitude;

/* 姿态 ISR 已完成零偏补偿和轴符号映射的 IMU 快照。
 * 主循环通过 attitude_get_imu_sample() 读取，避免诊断层再次访问 I2C。 */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} attitude_imu_sample_t;

extern volatile attitude_imu_sample_t attitude_imu_sample;
extern volatile uint32_t attitude_imu_sequence;

/* 返回一次一致的 ISR 快照；false 表示快照正被更新。 */
uint8_t attitude_get_imu_sample(attitude_imu_sample_t *sample);

/* 初始化: IMU 初始化 + 阻塞零偏标定(~2.25s) + Fusion AHRS 初始化.
 * 期间板子须静止. 返回 0=成功, 1=IMU 初始化失败. */
uint8_t attitude_init(void);

/* 定时器中断里调: 读 IMU + Fusion AHRS 更新 + 提取欧拉角.
 * 须以固定 ATTITUDE_DT 周期调用, 否则积分步长不准. */
void attitude_update_isr(void);

#endif /* __ATTITUDE_H__ */
