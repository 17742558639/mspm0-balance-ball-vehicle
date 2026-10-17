import re
import unittest
from pathlib import Path

from course_speed_policy_model import (
    Controller,
    RAMP_STEP_MS,
    SLOWDOWN_DELAY_MS,
    SLOW_SPEED,
    START_SPEED,
    State,
)


class CourseSpeedPolicyTests(unittest.TestCase):
    def test_control_c_uses_system_time_for_course_speed(self):
        source = (Path(__file__).parents[1] / "APP" / "Control.c").read_text(
            encoding="utf-8")
        self.assertIn('#include "course_speed.h"', source)
        self.assertIn(
            'CourseSpeed_start(&g_courseSpeed, (uint32_t)g_tms);', source)
        self.assertRegex(
            source,
            r"CourseSpeed_update\(\s*&g_courseSpeed,\s*"
            r"\(uint32_t\)g_tms,\s*basicSpeed\);")
        self.assertNotIn(
            'CourseSpeed_start(&g_courseSpeed, attitude.yaw);', source)
        self.assertNotRegex(
            source,
            r"CourseSpeed_update\(\s*&g_courseSpeed,\s*attitude\.yaw")
        self.assertIn('speed_control_base(effectiveSpeed);', source)

    def test_c_header_constants_match_model(self):
        header = (
            Path(__file__).parents[1] / "APP" / "course_speed.h"
        ).read_text(encoding="utf-8")
        expected = {
            "COURSE_START_SPEED": START_SPEED,
            "COURSE_RAMP_STEP_MS": RAMP_STEP_MS,
            "COURSE_SLOWDOWN_DELAY_MS": SLOWDOWN_DELAY_MS,
            "COURSE_SLOW_SPEED": SLOW_SPEED,
        }
        for name, value in expected.items():
            match = re.search(rf"#define\s+{name}\s+(\d+)U?", header)
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), value, name)

    def test_c_uses_wrap_safe_elapsed_time(self):
        source = (
            Path(__file__).parents[1] / "APP" / "course_speed.c"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "(uint32_t)(nowMs - controller->runStartMs)", source)
        self.assertNotIn("Yaw", source)
        self.assertNotIn("yaw", source)

    def test_c_source_applies_soft_start_before_slowdown(self):
        source = (
            Path(__file__).parents[1] / "APP" / "course_speed.c"
        ).read_text(encoding="utf-8")
        self.assertIn("CourseSpeed_ramp", source)
        self.assertIn("elapsedMs / COURSE_RAMP_STEP_MS", source)
        self.assertIn(
            "return CourseSpeed_ramp(requestedSpeed, elapsedMs);", source)

    def test_soft_start_begins_at_twelve_and_steps_every_100ms(self):
        controller = Controller()
        controller.start(1000)
        self.assertEqual(controller.update(1000, 26), 12)
        self.assertEqual(controller.update(1099, 26), 12)
        self.assertEqual(controller.update(1100, 26), 13)
        self.assertEqual(controller.update(2399, 26), 25)
        self.assertEqual(controller.update(2400, 26), 26)

    def test_soft_start_never_accelerates_a_lower_request(self):
        controller = Controller()
        controller.start(0)
        self.assertEqual(controller.update(0, 8), 8)

    def test_restart_resets_soft_start(self):
        controller = Controller()
        controller.start(0)
        self.assertEqual(controller.update(1400, 26), 26)
        controller.start(5000)
        self.assertEqual(controller.update(5000, 26), 12)

    def test_waits_until_configured_delay(self):
        controller = Controller()
        controller.start(1000)
        self.assertEqual(
            controller.update(1000 + SLOWDOWN_DELAY_MS - 1, 22), 22)
        self.assertEqual(controller.state, State.WAIT_DELAY)

    def test_slows_at_configured_delay_after_start(self):
        controller = Controller()
        controller.start(1000)
        self.assertEqual(
            controller.update(1000 + SLOWDOWN_DELAY_MS, 22), SLOW_SPEED)
        self.assertEqual(controller.state, State.SLOW)

    def test_slow_state_persists(self):
        controller = Controller()
        controller.start(0)
        controller.update(SLOWDOWN_DELAY_MS, 22)
        self.assertEqual(controller.update(30000, 22), SLOW_SPEED)

    def test_slow_state_never_accelerates_a_lower_request(self):
        controller = Controller()
        controller.start(0)
        self.assertEqual(controller.update(SLOWDOWN_DELAY_MS, 8), 8)

    def test_cancel_disables_slowdown(self):
        controller = Controller()
        controller.start(0)
        controller.cancel()
        self.assertEqual(controller.update(SLOWDOWN_DELAY_MS, 22), 22)
        self.assertEqual(controller.state, State.IDLE)

    def test_restart_resets_delay(self):
        controller = Controller()
        controller.start(0)
        self.assertEqual(controller.update(SLOWDOWN_DELAY_MS, 22), SLOW_SPEED)
        controller.start(20000)
        self.assertEqual(
            controller.update(20000 + SLOWDOWN_DELAY_MS - 1, 22), 22)
        self.assertEqual(
            controller.update(20000 + SLOWDOWN_DELAY_MS, 22), SLOW_SPEED)

    def test_uint32_timer_wrap_is_supported(self):
        controller = Controller()
        controller.start(0xFFFFF000)
        ramp_step = (0xFFFFF000 + RAMP_STEP_MS) & 0xFFFFFFFF
        before_delay = (0xFFFFF000 + SLOWDOWN_DELAY_MS - 1) & 0xFFFFFFFF
        at_delay = (0xFFFFF000 + SLOWDOWN_DELAY_MS) & 0xFFFFFFFF
        self.assertEqual(controller.update(ramp_step, 26), 13)
        self.assertEqual(controller.update(before_delay, 22), 22)
        self.assertEqual(controller.update(at_delay, 22), SLOW_SPEED)


if __name__ == "__main__":
    unittest.main()
