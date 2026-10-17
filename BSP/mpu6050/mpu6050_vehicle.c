#include "mpu6050_vehicle.h"

#include "attitude.h"
#include "mpu6050_raw.h"

#include <stddef.h>

#define MPU6050_SELF_TEST_TOLERANCE       (0.001f)
#define MPU6050_MONITOR_FILTER_ALPHA      (0.25f)

volatile float mpu6050_vehicle_monitor_yaw_rate_dps = 0.0f;
volatile float mpu6050_vehicle_monitor_accel_up_g = 0.0f;
volatile uint8_t mpu6050_vehicle_monitor_ready = 0U;
volatile uint8_t mpu6050_vehicle_monitor_stationary_ok = 0U;
volatile uint8_t mpu6050_vehicle_monitor_yaw_test_latched =
    (uint8_t)MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE;

static uint8_t monitor_axis_config_ok = 0U;
static uint8_t monitor_filter_valid = 0U;

static float absolute_value(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool nearly_equal(float actual, float expected)
{
    return absolute_value(actual - expected) <= MPU6050_SELF_TEST_TOLERANCE;
}

static bool valid_sign(float sign)
{
    return (sign == 1.0f) || (sign == -1.0f);
}

static float select_axis(const float values[3], uint8_t axis)
{
    return values[axis];
}

static void convert_physical_axes(const float accel[3],
                                  const float gyro[3],
                                  Mpu6050VehicleSample *sample)
{
    sample->accel_forward_g =
        select_axis(accel, MPU6050_VEHICLE_FORWARD_SOURCE) *
        MPU6050_VEHICLE_FORWARD_SIGN;
    sample->accel_right_g =
        select_axis(accel, MPU6050_VEHICLE_RIGHT_SOURCE) *
        MPU6050_VEHICLE_RIGHT_SIGN;
    sample->accel_up_g =
        select_axis(accel, MPU6050_VEHICLE_UP_SOURCE) *
        MPU6050_VEHICLE_UP_SIGN;

    sample->gyro_forward_dps =
        select_axis(gyro, MPU6050_VEHICLE_FORWARD_SOURCE) *
        MPU6050_VEHICLE_FORWARD_SIGN;
    sample->gyro_right_dps =
        select_axis(gyro, MPU6050_VEHICLE_RIGHT_SOURCE) *
        MPU6050_VEHICLE_RIGHT_SIGN;
    sample->gyro_up_dps =
        select_axis(gyro, MPU6050_VEHICLE_UP_SOURCE) *
        MPU6050_VEHICLE_UP_SIGN;
    sample->yaw_rate_right_dps =
        sample->gyro_up_dps * MPU6050_VEHICLE_YAW_RIGHT_SIGN;
}

bool mpu6050_vehicle_convert_raw(const Mpu6050VehicleRawSample *raw,
                                 Mpu6050VehicleSample *sample)
{
    float accel[3];
    float gyro[3];

    if ((raw == NULL) || (sample == NULL)) return false;

    for (uint8_t i = 0U; i < 3U; i++) {
        accel[i] = (float)raw->accel[i] / MPU6050_ACCEL_LSB_PER_G;
        gyro[i] = (float)raw->gyro[i] / MPU6050_GYRO_LSB_PER_DPS;
    }
    convert_physical_axes(accel, gyro, sample);
    return true;
}

bool mpu6050_vehicle_sample_from_existing_raw(Mpu6050VehicleSample *sample)
{
    Mpu6050VehicleRawSample raw;

    if (sample == NULL) return false;

    /* 只复制现有全局量，不调用 mpu6050_get_acc/get_gyro。 */
    raw.accel[0] = mpu6050_acc_x;
    raw.accel[1] = mpu6050_acc_y;
    raw.accel[2] = mpu6050_acc_z;
    raw.gyro[0] = mpu6050_gyro_x;
    raw.gyro[1] = mpu6050_gyro_y;
    raw.gyro[2] = mpu6050_gyro_z;
    return mpu6050_vehicle_convert_raw(&raw, sample);
}

bool mpu6050_vehicle_sample_from_attitude(Mpu6050VehicleSample *sample)
{
    attitude_imu_sample_t source;
    float accel[3];
    float gyro[3];

    if ((sample == NULL) || (attitude_get_imu_sample(&source) == 0U) ||
        (attitude_imu_sequence == 0U)) {
        return false;
    }

    accel[0] = source.accel_x_g;
    accel[1] = source.accel_y_g;
    accel[2] = source.accel_z_g;
    gyro[0] = source.gyro_x_dps;
    gyro[1] = source.gyro_y_dps;
    gyro[2] = source.gyro_z_dps;
    convert_physical_axes(accel, gyro, sample);
    return true;
}

bool mpu6050_vehicle_check_stationary(
    const Mpu6050VehicleSample *sample,
    Mpu6050VehicleStationaryCheck *check)
{
    if ((sample == NULL) || (check == NULL)) return false;

    check->accel_forward_ok =
        absolute_value(sample->accel_forward_g) <=
        MPU6050_VEHICLE_STATIONARY_HORIZONTAL_G_MAX;
    check->accel_right_ok =
        absolute_value(sample->accel_right_g) <=
        MPU6050_VEHICLE_STATIONARY_HORIZONTAL_G_MAX;
    check->accel_up_ok =
        absolute_value(sample->accel_up_g - 1.0f) <=
        MPU6050_VEHICLE_STATIONARY_UP_G_TOLERANCE;
    check->gyro_ok =
        (absolute_value(sample->gyro_forward_dps) <=
         MPU6050_VEHICLE_STATIONARY_GYRO_DPS_MAX) &&
        (absolute_value(sample->gyro_right_dps) <=
         MPU6050_VEHICLE_STATIONARY_GYRO_DPS_MAX) &&
        (absolute_value(sample->gyro_up_dps) <=
         MPU6050_VEHICLE_STATIONARY_GYRO_DPS_MAX);
    check->all_ok = check->accel_forward_ok && check->accel_right_ok &&
                    check->accel_up_ok && check->gyro_ok;
    return true;
}

Mpu6050VehicleYawTestResult mpu6050_vehicle_check_right_turn(
    const Mpu6050VehicleSample *sample)
{
    if (sample == NULL) return MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE;
    if (sample->yaw_rate_right_dps >= MPU6050_VEHICLE_YAW_TEST_MIN_DPS) {
        return MPU6050_VEHICLE_YAW_TEST_RIGHT_POSITIVE;
    }
    if (sample->yaw_rate_right_dps <= -MPU6050_VEHICLE_YAW_TEST_MIN_DPS) {
        return MPU6050_VEHICLE_YAW_TEST_POLARITY_REVERSED;
    }
    return MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE;
}

bool mpu6050_vehicle_axis_self_test(void)
{
    const Mpu6050VehicleRawSample raw = {
        .accel = {4096, 8192, 12288},
        .gyro = {164, 328, 492},
    };
    const float accel_expected[3] = {1.0f, 2.0f, 3.0f};
    const float gyro_expected[3] = {10.0f, 20.0f, 30.0f};
    Mpu6050VehicleSample sample;
    float forward_accel;
    float right_accel;
    float up_accel;
    float forward_gyro;
    float right_gyro;
    float up_gyro;

    if (!valid_sign(MPU6050_VEHICLE_FORWARD_SIGN) ||
        !valid_sign(MPU6050_VEHICLE_RIGHT_SIGN) ||
        !valid_sign(MPU6050_VEHICLE_UP_SIGN) ||
        !valid_sign(MPU6050_VEHICLE_YAW_RIGHT_SIGN) ||
        !mpu6050_vehicle_convert_raw(&raw, &sample)) {
        return false;
    }

    forward_accel = accel_expected[MPU6050_VEHICLE_FORWARD_SOURCE] *
                    MPU6050_VEHICLE_FORWARD_SIGN;
    right_accel = accel_expected[MPU6050_VEHICLE_RIGHT_SOURCE] *
                  MPU6050_VEHICLE_RIGHT_SIGN;
    up_accel = accel_expected[MPU6050_VEHICLE_UP_SOURCE] *
               MPU6050_VEHICLE_UP_SIGN;
    forward_gyro = gyro_expected[MPU6050_VEHICLE_FORWARD_SOURCE] *
                   MPU6050_VEHICLE_FORWARD_SIGN;
    right_gyro = gyro_expected[MPU6050_VEHICLE_RIGHT_SOURCE] *
                 MPU6050_VEHICLE_RIGHT_SIGN;
    up_gyro = gyro_expected[MPU6050_VEHICLE_UP_SOURCE] *
              MPU6050_VEHICLE_UP_SIGN;

    return nearly_equal(sample.accel_forward_g, forward_accel) &&
           nearly_equal(sample.accel_right_g, right_accel) &&
           nearly_equal(sample.accel_up_g, up_accel) &&
           nearly_equal(sample.gyro_forward_dps, forward_gyro) &&
           nearly_equal(sample.gyro_right_dps, right_gyro) &&
           nearly_equal(sample.gyro_up_dps, up_gyro) &&
           nearly_equal(sample.yaw_rate_right_dps,
                        up_gyro * MPU6050_VEHICLE_YAW_RIGHT_SIGN);
}

bool mpu6050_vehicle_monitor_init(void)
{
    mpu6050_vehicle_monitor_yaw_rate_dps = 0.0f;
    mpu6050_vehicle_monitor_accel_up_g = 0.0f;
    mpu6050_vehicle_monitor_ready = 0U;
    mpu6050_vehicle_monitor_stationary_ok = 0U;
    mpu6050_vehicle_monitor_yaw_test_latched =
        (uint8_t)MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE;
    monitor_filter_valid = 0U;
    monitor_axis_config_ok = mpu6050_vehicle_axis_self_test() ? 1U : 0U;
    return monitor_axis_config_ok != 0U;
}

bool mpu6050_vehicle_monitor_update(void)
{
    Mpu6050VehicleSample sample;
    Mpu6050VehicleStationaryCheck stationary;
    Mpu6050VehicleYawTestResult yaw_test;

    if ((monitor_axis_config_ok == 0U) ||
        !mpu6050_vehicle_sample_from_attitude(&sample) ||
        !mpu6050_vehicle_check_stationary(&sample, &stationary)) {
        mpu6050_vehicle_monitor_ready = 0U;
        return false;
    }

    if (monitor_filter_valid == 0U) {
        mpu6050_vehicle_monitor_yaw_rate_dps = sample.yaw_rate_right_dps;
        mpu6050_vehicle_monitor_accel_up_g = sample.accel_up_g;
        monitor_filter_valid = 1U;
    } else {
        mpu6050_vehicle_monitor_yaw_rate_dps += MPU6050_MONITOR_FILTER_ALPHA *
            (sample.yaw_rate_right_dps -
             mpu6050_vehicle_monitor_yaw_rate_dps);
        mpu6050_vehicle_monitor_accel_up_g += MPU6050_MONITOR_FILTER_ALPHA *
            (sample.accel_up_g - mpu6050_vehicle_monitor_accel_up_g);
    }

    mpu6050_vehicle_monitor_stationary_ok = stationary.all_ok ? 1U : 0U;
    mpu6050_vehicle_monitor_ready = 1U;

    yaw_test = mpu6050_vehicle_check_right_turn(&sample);
    if (yaw_test != MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE) {
        mpu6050_vehicle_monitor_yaw_test_latched = (uint8_t)yaw_test;
    }
    return true;
}

void mpu6050_vehicle_monitor_reset_yaw_test(void)
{
    mpu6050_vehicle_monitor_yaw_test_latched =
        (uint8_t)MPU6050_VEHICLE_YAW_TEST_INCONCLUSIVE;
}
