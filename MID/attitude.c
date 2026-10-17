/*
 * attitude.c - 姿态解算 (基于 xioTechnos Fusion 库)
 *
 * 算法: FusionAhrs (Mahony 变体, 完整 3D 叉乘反馈 + 加速度拒绝 + ramped gain).
 * 详见 attitude.h; 调试踩坑记录见 docs/姿态解算调试记录.md.
 * IMU 数据源由 imu_select.h 切换 (imu660rb / mpu6050_raw), 算法本体不变.
 *
 * 移植要点:
 *   - 保留自写零偏标定 (2s 静止采平均), 比 FusionOffset 在线估计更准.
 *   - 轴映射 IMU_AXIS_*_SIGN 在 imu_select.h, 映射后送 Fusion (载体系 ENU).
 *   - 无磁力计用 FusionAhrsUpdateNoMagnetometer, yaw 会漂但运动学一致.
 */
#include "ti_msp_dl_config.h"
#include "attitude.h"
#include "imu_select.h"
#include "delay.h"
#include "Fusion.h"
#include <math.h>

/* ---- 全局姿态输出 (度) ---- */
attitude_t attitude = {0, 0, 0};
volatile attitude_imu_sample_t attitude_imu_sample = {0, 0, 0, 0, 0, 0};
volatile uint32_t attitude_imu_sequence = 0U;

/* ---- Fusion AHRS 实例 ---- */
static FusionAhrs ahrs;

/* ---- 陀螺零偏 (°/s, 传感器坐标系, 标定求得) ---- */
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;


/* =====================================================================
 * attitude_init: IMU 初始化 + 静止零偏标定 + Fusion AHRS 初始化
 * 返回 0=成功, 1=IMU 初始化失败. 期间板子须静止.
 * ===================================================================== */
uint8_t attitude_init(void)
{
    if (imu_dev_init() != 0) return 1;

    /* 1. 丢前 N 帧: 软复位后陀螺有启动瞬态, 直接纳入平均会拉偏零偏. */
    for (uint16_t i = 0; i < GYRO_CALIB_DISCARD; i++) {
        imu_dev_get_gyro();
        delay_ms(1000u / (uint32_t)ATTITUDE_SAMPLE_HZ);
    }

    /* 2. 静止采集 N 帧陀螺求平均 -> 零偏. */
    int32_t sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < GYRO_CALIB_FRAMES; i++) {
        imu_dev_get_gyro();
        sx += imu_gyro_x;
        sy += imu_gyro_y;
        sz += imu_gyro_z;
        delay_ms(1000u / (uint32_t)ATTITUDE_SAMPLE_HZ);
    }
    gyro_bias_x = imu_gyro_transition((float)sx / GYRO_CALIB_FRAMES);
    gyro_bias_y = imu_gyro_transition((float)sy / GYRO_CALIB_FRAMES);
    gyro_bias_z = imu_gyro_transition((float)sz / GYRO_CALIB_FRAMES);

    /* 3. Fusion AHRS 初始化. ramped gain (INITIAL_GAIN=10 -> gain, 3s) 自带启动收敛,
     *    不用手动 acc 初始化 q. ENU 约定, 无磁力计. */
    FusionAhrsInitialise(&ahrs);
    const FusionAhrsSettings settings = {
        .convention            = FusionConventionEnu,
        .gain                  = FUSION_GAIN,
        .gyroscopeRange        = FUSION_GYRO_RANGE_DPS,
        .accelerationRejection = FUSION_ACC_REJECTION_DEG,
        .magneticRejection     = 0.0f,        /* 无磁力计, 禁用 */
        .recoveryTriggerPeriod = FUSION_RECOVERY_PERIOD,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    attitude.roll = attitude.pitch = attitude.yaw = 0.0f;
    attitude_imu_sequence = 0U;
    attitude_imu_sample.accel_x_g = 0.0f;
    attitude_imu_sample.accel_y_g = 0.0f;
    attitude_imu_sample.accel_z_g = 0.0f;
    attitude_imu_sample.gyro_x_dps = 0.0f;
    attitude_imu_sample.gyro_y_dps = 0.0f;
    attitude_imu_sample.gyro_z_dps = 0.0f;
    return 0;
}


/* =====================================================================
 * attitude_update_isr: 定时器中断调用, 读 IMU + Fusion 解算
 * 须以固定 ATTITUDE_DT 周期调用.
 * ===================================================================== */
void attitude_update_isr(void)
{
    /* 1. 读 IMU, 换算物理量 (°/s, g), 减陀螺零偏 */
    imu_dev_get_gyro();
    imu_dev_get_acc();

#if IMU_SELECT == IMU_USE_MPU6050
    /* I2C 读失败时不发布新心跳；控制层会在超时后立即关闭所有电机。 */
    if ((mpu6050_last_gyro_read_ok == 0U) ||
        (mpu6050_last_acc_read_ok == 0U)) {
        return;
    }
#endif

    float gx = imu_gyro_transition(imu_gyro_x) - gyro_bias_x;
    float gy = imu_gyro_transition(imu_gyro_y) - gyro_bias_y;
    float gz = imu_gyro_transition(imu_gyro_z) - gyro_bias_z;

    float ax = imu_acc_transition(imu_acc_x);
    float ay = imu_acc_transition(imu_acc_y);
    float az = imu_acc_transition(imu_acc_z);

    /* 2. 轴映射: IMU 传感器轴 -> 载体轴 (ENU). 陀螺与加速度计用同一组符号. */
    gx *= IMU_AXIS_X_SIGN;  gy *= IMU_AXIS_Y_SIGN;  gz *= IMU_AXIS_Z_SIGN;
    ax *= IMU_AXIS_X_SIGN;  ay *= IMU_AXIS_Y_SIGN;  az *= IMU_AXIS_Z_SIGN;

    /* 以奇偶序号发布一组完整快照，主循环可以无锁读取且不会看到半组数据。 */
    attitude_imu_sequence++;
    attitude_imu_sample.accel_x_g = ax;
    attitude_imu_sample.accel_y_g = ay;
    attitude_imu_sample.accel_z_g = az;
    attitude_imu_sample.gyro_x_dps = gx;
    attitude_imu_sample.gyro_y_dps = gy;
    attitude_imu_sample.gyro_z_dps = gz;
    attitude_imu_sequence++;

    /* 3. Fusion AHRS 更新 (无磁力计). gyro=°/s, acc=g, dt=s.
     *    库内部做完整 3D 叉乘反馈 + 加速度拒绝 + 四元数传播. */
    const FusionVector gyro = {.axis = {.x = gx, .y = gy, .z = gz}};
    const FusionVector acc  = {.axis = {.x = ax, .y = ay, .z = az}};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, acc, ATTITUDE_DT);

    /* 4. 从四元数提取 ZYX 欧拉角 (roll/pitch/yaw 同源, 运动学一致).
     *    pitch=±90° 附近 gimbal lock, roll/yaw 会耦合跳变 (数学特性). */
    const FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    attitude.roll  = e.angle.roll;
    attitude.pitch = e.angle.pitch;
    attitude.yaw   = e.angle.yaw;
}

uint8_t attitude_get_imu_sample(attitude_imu_sample_t *sample)
{
    uint32_t before;
    uint32_t after;

    if (sample == NULL) return 0U;

    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
        before = attitude_imu_sequence;
        if ((before & 1U) != 0U) continue;

        sample->accel_x_g = attitude_imu_sample.accel_x_g;
        sample->accel_y_g = attitude_imu_sample.accel_y_g;
        sample->accel_z_g = attitude_imu_sample.accel_z_g;
        sample->gyro_x_dps = attitude_imu_sample.gyro_x_dps;
        sample->gyro_y_dps = attitude_imu_sample.gyro_y_dps;
        sample->gyro_z_dps = attitude_imu_sample.gyro_z_dps;

        after = attitude_imu_sequence;
        if ((before == after) && ((after & 1U) == 0U)) return 1U;
    }
    return 0U;
}


/* =====================================================================
 * TIMER_ATT_INST 中断: 清 ZERO 标志 + 调姿态解算.
 * 用宏名, 换定时器实例时此处自动跟着改. 启动 (NVIC_EnableIRQ +
 * startCounter) 由系统编排层负责, 不在本模块.
 * ===================================================================== */
void TIMER_ATT_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(TIMER_ATT_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    attitude_update_isr();
}
