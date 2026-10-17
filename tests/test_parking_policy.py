import re
import unittest
from pathlib import Path

from parking_policy_model import (
    Action,
    BRAKE_MS,
    Controller,
    FORWARD_CONFIRM_TICKS,
    LEAVE_TICKS,
    SLOW_TARGET_MIN_COUNT,
    State,
    TARGET_COUNT,
)


class ParkingPolicyTests(unittest.TestCase):
    def test_c_header_constants_match_model(self):
        header = (Path(__file__).parents[1] / "APP" / "parking.h").read_text(
            encoding="utf-8")
        expected = {
            "PARK_TARGET_ACTIVE_COUNT": TARGET_COUNT,
            "PARK_SLOW_TARGET_MIN_ACTIVE_COUNT": SLOW_TARGET_MIN_COUNT,
            "PARK_LEAVE_CONFIRM_TICKS": LEAVE_TICKS,
            "PARK_FORWARD_CONFIRM_TICKS": FORWARD_CONFIRM_TICKS,
            "PARK_BRAKE_HOLD_MS": BRAKE_MS,
        }
        for name, value in expected.items():
            match = re.search(rf"#define\s+{name}\s+\(?(-?\d+)U?\)?", header)
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), value, name)

    def test_starting_on_target_does_not_stop(self):
        c = Controller()
        c.start(1000)
        for now in range(1000, 1100, 10):
            self.assertEqual(c.update(TARGET_COUNT, now), Action.FORWARD)
        self.assertEqual(c.state, State.WAIT_LEAVE_A)

    def test_leave_requires_five_consecutive_samples(self):
        c = Controller()
        c.start(0)
        for now in (0, 10, 20, 30):
            c.update(2, now)
        c.update(TARGET_COUNT, 40)
        self.assertEqual(c.state, State.WAIT_LEAVE_A)
        for now in (50, 60, 70, 80, 90):
            c.update(2, now)
        self.assertEqual(c.state, State.RUNNING)

    def test_target_count_must_be_consecutive_before_braking(self):
        c = self.running_controller()
        self.assertEqual(c.update(TARGET_COUNT, 50), Action.FORWARD)
        self.assertEqual(c.update(2, 60), Action.FORWARD)
        for index in range(FORWARD_CONFIRM_TICKS - 1):
            now = 70 + index * 10
            self.assertEqual(c.update(TARGET_COUNT, now), Action.FORWARD)
        brake_start_ms = 70 + (FORWARD_CONFIRM_TICKS - 1) * 10
        self.assertEqual(c.update(TARGET_COUNT, brake_start_ms), Action.STOP)
        self.assertEqual(c.state, State.BRAKE_CHECK)

    def test_normal_mode_requires_exactly_four_active_sensors(self):
        for active_count in (3, 5, 8):
            with self.subTest(active_count=active_count):
                c = self.running_controller()
                for now in (50, 60, 70, 80):
                    self.assertEqual(c.update(active_count, now), Action.FORWARD)
                self.assertEqual(c.state, State.RUNNING)
                self.assertEqual(c.forward_ticks, 0)

    def test_slow_mode_accepts_three_or_more_active_sensors(self):
        for active_count in (3, 4, 5, 8):
            with self.subTest(active_count=active_count):
                c = self.running_controller()
                self.assertEqual(
                    c.update(active_count, 50, relaxed_target=True),
                    Action.FORWARD)
                self.assertEqual(
                    c.update(active_count, 60, relaxed_target=True),
                    Action.FORWARD)
                self.assertEqual(
                    c.update(active_count, 70, relaxed_target=True),
                    Action.STOP)
                self.assertEqual(c.state, State.BRAKE_CHECK)

    def test_slow_mode_rejects_fewer_than_three_active_sensors(self):
        for active_count in (0, 1, 2):
            with self.subTest(active_count=active_count):
                c = self.running_controller()
                for now in (50, 60, 70, 80):
                    self.assertEqual(
                        c.update(active_count, now, relaxed_target=True),
                        Action.FORWARD)
                self.assertEqual(c.state, State.RUNNING)
                self.assertEqual(c.forward_ticks, 0)

    def test_slow_mode_confirmation_must_be_three_consecutive_samples(self):
        c = self.running_controller()
        self.assertEqual(
            c.update(3, 50, relaxed_target=True), Action.FORWARD)
        self.assertEqual(
            c.update(2, 60, relaxed_target=True), Action.FORWARD)
        self.assertEqual(c.forward_ticks, 0)
        self.assertEqual(
            c.update(3, 70, relaxed_target=True), Action.FORWARD)
        self.assertEqual(
            c.update(3, 80, relaxed_target=True), Action.FORWARD)
        self.assertEqual(c.update(3, 90, relaxed_target=True), Action.STOP)
        self.assertEqual(c.state, State.BRAKE_CHECK)

    def test_relaxed_flag_does_not_change_wait_leave_a_rule(self):
        c = Controller()
        c.start(0)
        for now in (0, 10, 20, 30, 40, 50):
            self.assertEqual(
                c.update(3, now, relaxed_target=True), Action.FORWARD)
        self.assertEqual(c.state, State.RUNNING)

    def test_c_running_state_selects_strict_or_relaxed_target(self):
        root = Path(__file__).parents[1]
        header = (root / "APP" / "parking.h").read_text(encoding="utf-8")
        source = (root / "APP" / "parking.c").read_text(encoding="utf-8")
        self.assertIn("bool relaxedTarget", header)
        self.assertIn("Parking_targetDetected", source)
        self.assertIn(
            "activeCount >= PARK_SLOW_TARGET_MIN_ACTIVE_COUNT", source)
        self.assertIn("activeCount == PARK_TARGET_ACTIVE_COUNT", source)

    def test_control_enables_relaxed_target_only_in_course_slow_state(self):
        source = (
            Path(__file__).parents[1] / "APP" / "Control.c"
        ).read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"bool\s+relaxedParkingTarget\s*=\s*"
            r"g_courseSpeed\.state\s*==\s*COURSE_SPEED_SLOW\s*;")
        self.assertRegex(
            source,
            r"Parking_update\(\s*&g_parking,\s*track_active_count\(\),\s*"
            r"relaxedParkingTarget,\s*\(uint32_t\)g_tms\s*\);")

    def test_still_on_line_after_brake_is_success(self):
        c = self.running_controller()
        for index in range(FORWARD_CONFIRM_TICKS):
            now = 50 + index * 10
            c.update(TARGET_COUNT, now)
        brake_start_ms = 50 + (FORWARD_CONFIRM_TICKS - 1) * 10
        self.assertEqual(
            c.update(TARGET_COUNT, brake_start_ms + BRAKE_MS - 10),
            Action.STOP)
        finish_ms = brake_start_ms + BRAKE_MS
        self.assertEqual(c.update(TARGET_COUNT, finish_ms), Action.STOP)
        self.assertEqual(c.state, State.STOPPED)
        self.assertEqual(c.elapsed_ms, finish_ms)

    def test_brake_hold_finishes_when_target_signal_disappears(self):
        c = self.running_controller()
        for now in (50, 60, 70):
            c.update(TARGET_COUNT, now)
        self.assertEqual(c.update(2, 160), Action.STOP)
        self.assertEqual(c.state, State.BRAKE_CHECK)
        self.assertFalse(c.has_result)
        self.assertEqual(c.update(2, 170), Action.STOP)
        self.assertEqual(c.state, State.STOPPED)
        self.assertEqual(c.elapsed_ms, 170)

    def test_production_parking_path_has_no_reverse_recovery(self):
        root = Path(__file__).parents[1]
        sources = "\n".join(
            (root / relative).read_text(encoding="utf-8")
            for relative in ("APP/parking.h", "APP/parking.c", "APP/Control.c")
        )
        for symbol in (
                "PARK_STATE_REVERSE_SEARCH",
                "PARK_ACTION_REVERSE",
                "PARK_REVERSE_CONFIRM_TICKS",
                "PARK_REVERSE_TIMEOUT_MS",
                "PARK_REVERSE_SPEED",
                "reverseConfirmTicks"):
            self.assertNotIn(symbol, sources)

    def test_cancel_clears_result(self):
        c = self.running_controller()
        for now in (50, 60, 70, 80, 90):
            c.update(TARGET_COUNT, now)
        c.update(TARGET_COUNT, 190)
        self.assertTrue(c.has_result)
        c.cancel()
        self.assertEqual(c.state, State.IDLE)
        self.assertFalse(c.has_result)

    @staticmethod
    def running_controller():
        c = Controller()
        c.start(0)
        for now in (0, 10, 20, 30, 40):
            c.update(2, now)
        return c

if __name__ == "__main__":
    unittest.main()
