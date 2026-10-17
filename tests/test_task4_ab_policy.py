import re
import unittest
from pathlib import Path

from task4_ab_policy_model import (
    Controller,
    RAMP_STEP_MS,
    SPEED,
    START_SPEED,
    State,
    TARGET_PULSES,
    TIMEOUT_MS,
    UINT32_MASK,
    UINT32_MAX,
)


class Task4AbPolicyTests(unittest.TestCase):
    def test_target_pulse_does_not_stop_continuous_run(self):
        controller = Controller()
        controller.start(100)
        controller.record_encoder(TARGET_PULSES, TARGET_PULSES)
        controller.update(7010)
        self.assertEqual(controller.state, State.RUNNING)
        self.assertFalse(controller.has_result)
        self.assertEqual(controller.elapsed_ms, 6910)

    def test_timeout_does_not_stop_continuous_run(self):
        controller = Controller()
        controller.start(1000)
        controller.update(9000)
        self.assertEqual(controller.state, State.RUNNING)
        self.assertFalse(controller.has_result)
        self.assertEqual(controller.elapsed_ms, TIMEOUT_MS)

    def test_target_and_timeout_together_still_continue(self):
        controller = Controller()
        controller.start(0)
        controller.record_encoder(TARGET_PULSES, TARGET_PULSES)
        controller.update(TIMEOUT_MS)
        self.assertEqual(controller.state, State.RUNNING)
        self.assertFalse(controller.has_result)

    def test_command_speed_ramps_from_seven_to_eleven(self):
        controller = Controller()
        controller.start(100)
        self.assertEqual(controller.command_speed(100), 7)
        self.assertEqual(controller.command_speed(199), 7)
        self.assertEqual(controller.command_speed(200), 8)
        self.assertEqual(controller.command_speed(500), 11)
        self.assertEqual(controller.command_speed(5000), 11)
        controller.cancel()
        self.assertEqual(controller.command_speed(5000), 0)

    def test_negative_encoder_samples_do_not_add_distance(self):
        controller = Controller()
        controller.start(0)
        controller.record_encoder(-20, -30)
        self.assertEqual(controller.distance_pulses, 0)

    def test_average_keeps_an_odd_pulse_pair(self):
        controller = Controller()
        controller.start(0)
        controller.record_encoder(1, 1)
        self.assertEqual(controller.distance_pulses, 1)

    def test_distance_saturates_instead_of_wrapping(self):
        controller = Controller()
        controller.start(0)
        controller.left_pulses = UINT32_MAX - 2
        controller.right_pulses = UINT32_MAX - 2
        controller.record_encoder(10, 10)
        self.assertEqual(controller.left_pulses, UINT32_MAX)
        self.assertEqual(controller.right_pulses, UINT32_MAX)

    def test_encoder_samples_are_ignored_after_result(self):
        controller = Controller()
        controller.start(0)
        controller.record_encoder(TARGET_PULSES, TARGET_PULSES)
        controller.state = State.PASSED
        controller.record_encoder(50, 50)
        self.assertEqual(controller.distance_pulses, TARGET_PULSES)

    def test_cancel_and_restart_clear_task4_state(self):
        controller = Controller()
        controller.start(0)
        controller.record_encoder(100, 100)
        controller.cancel()
        self.assertEqual(controller.state, State.IDLE)
        self.assertFalse(controller.has_result)
        controller.start(500)
        self.assertEqual(controller.state, State.RUNNING)
        self.assertEqual(controller.distance_pulses, 0)

    def test_uint32_timer_wrap_is_supported(self):
        controller = Controller()
        controller.start(0xFFFFF000)
        wrapped_now = (0xFFFFF000 + TIMEOUT_MS) & UINT32_MASK
        controller.update(wrapped_now)
        self.assertEqual(controller.state, State.RUNNING)
        self.assertEqual(controller.elapsed_ms, TIMEOUT_MS)
        self.assertEqual(controller.command_speed(wrapped_now), SPEED)

    def test_task4_c_files_define_expected_constants_and_api(self):
        root = Path(__file__).parents[1]
        header_path = root / "APP" / "task4_ab.h"
        source_path = root / "APP" / "task4_ab.c"
        self.assertTrue(header_path.exists(), "APP/task4_ab.h")
        self.assertTrue(source_path.exists(), "APP/task4_ab.c")

        header = header_path.read_text(encoding="utf-8")
        source = source_path.read_text(encoding="utf-8")
        expected = {
            "TASK4_AB_SPEED": SPEED,
            "TASK4_AB_START_SPEED": START_SPEED,
            "TASK4_AB_RAMP_STEP_MS": RAMP_STEP_MS,
            "TASK4_AB_TARGET_PULSES": TARGET_PULSES,
            "TASK4_AB_TIMEOUT_MS": TIMEOUT_MS,
        }
        for name, value in expected.items():
            match = re.search(rf"#define\s+{name}\s+(\d+)U?", header)
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), value, name)

        for api in (
            "Task4Ab_init",
            "Task4Ab_start",
            "Task4Ab_commandSpeed",
            "Task4Ab_cancel",
            "Task4Ab_recordEncoder",
            "Task4Ab_update",
            "Task4Ab_distancePulses",
            "Task4Ab_hasResult",
            "Task4Ab_succeeded",
        ):
            self.assertIn(api, header)

        self.assertIn("UINT32_MAX", source)
        self.assertIn("(uint32_t)(nowMs - controller->runStartMs)", source)
        self.assertIn("Task4Ab_commandSpeed", source)
        update_match = re.search(
            r"void\s+Task4Ab_update\s*\([^)]*\)\s*"
            r"\{(?P<body>.*?)^\}",
            source,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(update_match)
        self.assertNotIn("TASK4_AB_TARGET_PULSES", update_match.group("body"))
        self.assertNotIn("TASK4_AB_TIMEOUT_MS", update_match.group("body"))
        self.assertNotIn("step_on", source)


if __name__ == "__main__":
    unittest.main()
