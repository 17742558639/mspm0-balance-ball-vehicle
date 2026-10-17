#ifndef MPU6050_VEHICLE_H
#define MPU6050_VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MPU6050 车辆坐标适配与只读诊断层。
 *
 * 车辆坐标约定：+forward 向车头，+right 向车右，+up 向车顶；
 * yaw_rate_right_dps > 0 表示向右/顺时针转弯。
 *
 * 本模块不会初始化 MPU6050、不会访问 I2C、不会写任何电机输出。
 * 运行时优先读取 attitude.c 已完成零偏补偿的一致性快照。
 */

typedef enum {
    MPU6050_VEHICLE_SENSOR_X = 0,
    MPU6050_VEHICLE_SENSOR_Y = 1,
    MPU6050_VEHICLE_SENSOR_Z = 2
} Mpu6050VehicleSensorAxis;

/* 安装方向不同时只改这里，并通过 OLED 的 YawPol/UpG 做实物确认。 */
#define MPU6050_VEHICLE_FORWARD_SOURCE  (0)
#define MPU6050_VEHICLE_RIGHT_SOURCE    (1)
#define MPU6050_VEHICLE_UP_SOURCE       (2)

#define MPU6050_VEHICLE_FORWARD_SIGN    (+1.0f)
#define MPU6050_VEHICLE_RIGHT_SIGN      (+1.0f)
#define MPU6050_VEHICLE_UP_SIGN         (+1.0f)
#define MPU6050_VEHICLE_YAW_RIGHT_SIGN  (-1.0f)

#define MPU6050_VEHICLE_STATIONARY_HORIZONTAL_G_MAX  (0.15f)
#define MPU6050_VEHICLE_STATIONARY_UP_G_TOLERANCE    (0.20f)
#define MPU6050_VEHICLE_STATIONARY_GYRO_DPS_MAX      (3.0f)
#define MPU6050_VEHICLE_YAW_TEST_MIN_DPS             (5.0f)

#if (MPU6050_VEHICLE_FORWARD_SOURCE < 0) || \
    (MPU6050_VEHICLE_FORWARD_SOURCE > 2) || \
    (MPU6050_VEHICLE_RIGHT_SOURCE < 0) || \
    (MPU6050_VEHICLE_RIGHT_SOURCE > 2) || \
    (MPU6050_VEHICLE_UP_SOURCE < 0) || \
    (MPU6050_VEHICLE_UP_SOURCE > 2)
#error "MPU6050 vehicle axis source must be 0, 1, or 2"
#endif

#if (MPU6050_VEHICLE_FORWARD_SOURCE == MPU6050_VEHICLE_RIGHT_SOURCE) || \
    (MPU6050_VEHICLE_FORWARD_SOURCE == MPU6050_VEHICLE_UP_SOURCE) || \
    (MPU6050_VEHICLE_RIGHT_SOURCE == MPU6050_VEHICLE_UP_SOURCE)
#error "MPU6050 vehicle axis sources must be unique"
#endif

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} Mpu6050VehicleRawSample;

typedef struct {
    float accel_forward_g;
    float accel_right_g;
    float accel_up_g;
    float gyro_forward_dps;
    float gyro_right_dps;
    float gyro_up_dps;
    float yaw_rate_right_dps;
} Mpu6050VehicleSample;

typedef struct {
    bool accel_forward_ok;
    bool accel_right_ok;
    bool accel_up_ok;
    bool gyro_ok;
    bool all_ok;
} Mpu6050VehicleStationaryCheck;

typedef enum {
    MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE = 0,
    MPU6050_VEHICLE_YAW_TEST_RIGHT_POSITIVE = 1,
    MPU6050_VEHICLE_YAW_TEST_POLARITY_REVERSED = 2
} Mpu6050VehicleYawTestResult;

/* 纯换算与只读快照接口；均不触发 I2C。 */
bool mpu6050_vehicle_convert_raw(
    const Mpu6050VehicleRawSample *raw,
    Mpu6050VehicleSample *sample);
bool mpu6050_vehicle_sample_from_existing_raw(
    Mpu6050VehicleSample *sample);
bool mpu6050_vehicle_sample_from_attitude(
    Mpu6050VehicleSample *sample);

bool mpu6050_vehicle_check_stationary(
    const Mpu6050VehicleSample *sample,
    Mpu6050VehicleStationaryCheck *check);
Mpu6050VehicleYawTestResult mpu6050_vehicle_check_right_turn(
    const Mpu6050VehicleSample *sample);
bool mpu6050_vehicle_axis_self_test(void);

/* OLED/调试器使用的被动监视量：YawPol 0=未测试，1=正确，2=极性相反。 */
extern volatile float mpu6050_vehicle_monitor_yaw_rate_dps;
extern volatile float mpu6050_vehicle_monitor_accel_up_g;
extern volatile uint8_t mpu6050_vehicle_monitor_ready;
extern volatile uint8_t mpu6050_vehicle_monitor_stationary_ok;
extern volatile uint8_t mpu6050_vehicle_monitor_yaw_test_latched;

bool mpu6050_vehicle_monitor_init(void);
bool mpu6050_vehicle_monitor_update(void);
void mpu6050_vehicle_monitor_reset_yaw_test(void);

#endif /* MPU6050_VEHICLE_H */
