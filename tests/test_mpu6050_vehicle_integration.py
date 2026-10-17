import re
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
VEHICLE_H = ROOT / "BSP" / "mpu6050" / "mpu6050_vehicle.h"
VEHICLE_C = ROOT / "BSP" / "mpu6050" / "mpu6050_vehicle.c"
MENU_C = ROOT / "APP" / "menu" / "app_menu.c"
RAW_H = ROOT / "BSP" / "mpu6050" / "mpu6050_raw.h"
CONTROL_C = ROOT / "APP" / "Control.c"
SYSTEM_C = ROOT / "APP" / "system.c"


class Mpu6050VehicleIntegrationTests(unittest.TestCase):
    def test_vehicle_adapter_is_present_and_uses_existing_raw_path(self):
        self.assertTrue(VEHICLE_H.exists())
        self.assertTrue(VEHICLE_C.exists())
        source = VEHICLE_C.read_text(encoding="utf-8")
        self.assertIn("mpu6050_vehicle_sample_from_existing_raw", source)
        self.assertIn("mpu6050_vehicle_monitor_update", source)
        self.assertNotIn("mpu6050_init();", source.split(
            "mpu6050_vehicle_monitor_init", 1)[-1])

    def test_raw_samples_are_volatile_for_isr_to_main_snapshot(self):
        header = RAW_H.read_text(encoding="utf-8")
        for name in ("mpu6050_acc_x", "mpu6050_acc_y", "mpu6050_acc_z",
                     "mpu6050_gyro_x", "mpu6050_gyro_y", "mpu6050_gyro_z"):
            self.assertRegex(header, rf"extern volatile int16_t\s+{name}")

    def test_track_mode_menu_keeps_vehicle_monitor_without_extra_pages(self):
        menu = MENU_C.read_text(encoding="utf-8")
        self.assertIn('#include "mpu6050_vehicle.h"', menu)
        self.assertIn("mpu6050_vehicle_monitor_init", menu)
        self.assertIn("mpu6050_vehicle_monitor_update", menu)
        self.assertNotIn('"YawRate"', menu)
        self.assertNotIn('"VStatic"', menu)

    def test_scale_matches_current_raw_driver_configuration(self):
        raw = RAW_H.read_text(encoding="utf-8")
        self.assertRegex(raw, r"MPU6050_ACCEL_LSB_PER_G\s+\(?4096\.0f\)?")
        self.assertRegex(raw, r"MPU6050_GYRO_LSB_PER_DPS\s+\(?16\.4f\)?")

    def test_measured_right_turn_uses_negative_sensor_z_polarity(self):
        header = VEHICLE_H.read_text(encoding="utf-8")
        self.assertRegex(
            header,
            r"#define\s+MPU6050_VEHICLE_YAW_RIGHT_SIGN\s+\(-1\.0f\)",
        )

    def test_imu_failure_latches_motor_interlock(self):
        control = CONTROL_C.read_text(encoding="utf-8")
        system = SYSTEM_C.read_text(encoding="utf-8")
        self.assertIn("control_set_motion_enabled", control)
        self.assertIn("control_motion_enabled", control)
        self.assertIn("control_force_stop", control)
        self.assertRegex(system, r"control_set_motion_enabled\s*\(\s*imu_ok\s*\)")

    def test_runtime_control_checks_imu_heartbeat(self):
        control = CONTROL_C.read_text(encoding="utf-8")
        self.assertIn("CONTROL_IMU_STALE_TICKS", control)
        self.assertIn("attitude_imu_sequence", control)
        self.assertIn("control_imu_heartbeat_ok", control)

    def test_control_force_stop_remains_the_motor_safety_boundary(self):
        menu = MENU_C.read_text(encoding="utf-8")
        control = CONTROL_C.read_text(encoding="utf-8")
        body = re.search(
            r"static void control_force_stop\(void\)\s*\{(?P<body>.*?)\n\}",
            control,
            re.S,
        )
        self.assertIsNotNone(body)
        self.assertIn("step_off(X)", body.group("body"))
        self.assertIn("step_off(Y)", body.group("body"))
        self.assertIn("motor_behind_control(0, 0)", body.group("body"))
        self.assertNotIn("step_on(", body.group("body"))
        self.assertNotIn("action_motor_stop", menu)


if __name__ == "__main__":
    unittest.main()
