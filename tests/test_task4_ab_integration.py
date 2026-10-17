import re
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]


class Task4AbIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.control_h = (ROOT / "APP" / "Control.h").read_text(
            encoding="utf-8")
        cls.control_c = (ROOT / "APP" / "Control.c").read_text(
            encoding="utf-8")
        cls.menu_c = (ROOT / "APP" / "menu" / "app_menu.c").read_text(
            encoding="utf-8")
        cls.task4_c = (ROOT / "APP" / "task4_ab.c").read_text(
            encoding="utf-8")

    @staticmethod
    def function_body(source, name, return_type=r"(?:static\s+)?(?:void|bool|uint32_t|ControlTask)"):
        match = re.search(
            rf"{return_type}\s+{name}\s*\([^)]*\)\s*"
            r"\{(?P<body>.*?)^\}",
            source,
            re.MULTILINE | re.DOTALL,
        )
        if match is None:
            raise AssertionError(f"missing function {name}")
        return match.group("body")

    def test_control_exposes_explicit_a_stop_and_ab_tasks(self):
        for token in (
            "CONTROL_TASK_NONE",
            "CONTROL_TASK_A_STOP",
            "CONTROL_TASK_AB",
            "control_start_ab",
            "control_active_task",
        ):
            self.assertIn(token, self.control_h)

    def test_task2_start_uses_parking_and_course_speed_only(self):
        body = self.function_body(self.control_c, "control_set_run")
        self.assertIn("Task4Ab_cancel(&g_task4Ab);", body)
        self.assertIn("Parking_start(&g_parking", body)
        self.assertIn("CourseSpeed_start(&g_courseSpeed", body)
        self.assertIn("CONTROL_TASK_A_STOP", body)
        self.assertNotIn("Task4Ab_start", body)

    def test_task4_start_uses_task4_controller_only(self):
        body = self.function_body(
            self.control_c, "control_start_ab", return_type=r"bool")
        self.assertIn("Parking_cancel(&g_parking);", body)
        self.assertIn("CourseSpeed_cancel(&g_courseSpeed);", body)
        self.assertIn("Task4Ab_start(&g_task4Ab", body)
        self.assertIn("CONTROL_TASK_AB", body)
        self.assertNotIn("Parking_start", body)
        self.assertNotIn("CourseSpeed_start", body)

    def test_isr_has_separate_task2_and_task4_branches(self):
        body = self.function_body(
            self.control_c,
            "TIMER_0_INST_IRQHandler",
            return_type=r"void",
        )
        self.assertRegex(
            body,
            r"CONTROL_TASK_A_STOP[\s\S]+CourseSpeed_update"
            r"[\s\S]+Parking_update",
        )
        self.assertRegex(
            body,
            r"CONTROL_TASK_AB[\s\S]+Task4Ab_commandSpeed"
            r"[\s\S]+speed_control_base"
            r"[\s\S]+Task4Ab_recordEncoder[\s\S]+Task4Ab_update",
        )

    def test_ab_isr_uses_ramped_task4_speed(self):
        body = self.function_body(
            self.control_c,
            "TIMER_0_INST_IRQHandler",
            return_type=r"void",
        )
        self.assertRegex(
            body,
            r"CONTROL_TASK_AB[\s\S]+"
            r"Task4Ab_commandSpeed\s*\(\s*&g_task4Ab[\s\S]+"
            r"speed_control_base",
        )
        self.assertNotIn("Task4Ab_hasResult", body)

    def test_ab_menu_action_starts_ab_and_opens_live_view(self):
        body = self.function_body(self.menu_c, "action_ab")
        self.assertIn("g_ab_view_active = true", body)
        self.assertIn("control_start_ab()", body)
        self.assertNotIn("control_set_run(true)", body)

    def test_ab_starts_center_hold_only_after_vehicle_start_succeeds(self):
        body = self.function_body(self.menu_c, "action_ab")
        self.assertRegex(
            body,
            re.compile(
                r"BallTargetTask_stop\s*\(\s*\)\s*;.*?"
                r"if\s*\(\s*control_start_ab\s*\(\s*\)\s*\)\s*"
                r"\{.*?BallTargetTask_startCenterHold\s*\(\s*\)",
                re.DOTALL,
            ),
        )

    def test_ab_live_view_shows_continuous_running_state(self):
        body = self.function_body(self.menu_c, "ab_render")
        for token in (
            "control_ab_elapsed_ms()",
            "control_ab_distance_pulses()",
            '"RUNNING"',
            '"START BLOCKED"',
        ):
            self.assertIn(token, body)
        self.assertNotIn('"AB PASS"', body)
        self.assertNotIn('"TIMEOUT"', body)

    def test_back_from_ab_safely_stops_current_task(self):
        body = self.function_body(self.menu_c, "app_menu_on_key")
        self.assertIn("g_ab_view_active", body)
        self.assertIn("control_set_run(false)", body)
        self.assertIn("g_ab_view_active = false", body)

    def test_back_from_ab_stops_vehicle_and_ball(self):
        body = self.function_body(self.menu_c, "app_menu_on_key")
        self.assertRegex(
            body,
            re.compile(
                r"if\s*\(\s*g_ab_view_active\s*\).*?"
                r"control_set_run\s*\(\s*false\s*\)\s*;.*?"
                r"BallTargetTask_stop\s*\(\s*\)\s*;",
                re.DOTALL,
            ),
        )

    def test_ab_elapsed_getter_is_live_then_holds_result(self):
        body = self.function_body(
            self.control_c,
            "control_ab_elapsed_ms",
            return_type=r"uint32_t",
        )
        self.assertIn("g_task4Ab.runStartMs", body)
        self.assertIn("g_task4Ab.elapsedMs", body)
        self.assertIn("g_tms", body)

    def test_long_press_toggle_stops_selected_ab_instead_of_starting_task2(self):
        body = self.function_body(self.control_c, "control_toggle_run")
        self.assertIn("CONTROL_TASK_NONE", body)
        self.assertIn("control_set_run(false)", body)

    def test_task4_never_starts_a_step_motor(self):
        self.assertNotIn("step_on", self.task4_c)
        body = self.function_body(
            self.control_c, "control_start_ab", return_type=r"bool")
        self.assertNotIn("step_on", body)


if __name__ == "__main__":
    unittest.main()
