import re
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
MENU_C = ROOT / "APP" / "menu" / "app_menu.c"
CONTROL_C = ROOT / "APP" / "Control.c"


class TrackModeMenuTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.menu = MENU_C.read_text(encoding="utf-8")

    def array_body(self, name):
        match = re.search(
            rf"static\s+const\s+MenuItem\s+{name}\s*\[\s*\]\s*=\s*"
            r"\{(?P<body>.*?)^\};",
            self.menu,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match, f"missing {name}[]")
        return match.group("body")

    def function_body(self, name):
        match = re.search(
            rf"(?:static\s+)?void\s+{name}\s*\([^)]*\)\s*"
            r"\{(?P<body>.*?)^\}",
            self.menu,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match, f"missing {name}(void)")
        return match.group("body")

    def test_root_page_matches_track_mode_tree(self):
        root = self.array_body("main_items")
        labels = re.findall(r'^\s*\{"([^"]+)"', root, re.MULTILINE)
        self.assertEqual(labels, ["Track View", "A Stop", "A Pass", "Ball Only"])
        self.assertRegex(
            root,
            r'\{"Track View"\s*,\s*MENU_ITEM_SUBMENU\s*,\s*'
            r'\{\.page\s*=\s*&pages\[PAGE_TRACK_VIEW\]\}\}',
        )
        self.assertRegex(
            root,
            r'\{"A Stop"\s*,\s*MENU_ITEM_ACTION\s*,\s*'
            r'\{\.action\s*=\s*action_a_stop\}\}',
        )
        self.assertRegex(
            root,
            r'\{"A Pass"\s*,\s*MENU_ITEM_SUBMENU\s*,\s*'
            r'\{\.page\s*=\s*&pages\[PAGE_A_PASS\]\}\}',
        )
        self.assertRegex(
            root,
            r'\{"Ball Only"\s*,\s*MENU_ITEM_ACTION\s*,\s*'
            r'\{\.action\s*=\s*action_ball_only\}\}',
        )

    def test_a_pass_submenu_matches_tasks_4_to_6(self):
        items = self.array_body("a_pass_items")
        labels = re.findall(r'^\s*\{"([^"]+)"', items, re.MULTILINE)
        self.assertEqual(labels, ["AB", "Full Lap", "Ball Target"])
        for callback in ("action_ab", "action_full_lap", "action_ball_target"):
            self.assertIn(callback, items)

    def test_page_titles_match_framework(self):
        self.assertRegex(
            self.menu,
            r'\[\s*PAGE_MAIN\s*\]\s*=\s*\{\s*"Track Mode"\s*,',
        )
        self.assertRegex(
            self.menu,
            r'\[\s*PAGE_TRACK_VIEW\s*\]\s*=\s*\{\s*"Track View"\s*,',
        )
        self.assertRegex(
            self.menu,
            r'\[\s*PAGE_A_PASS\s*\]\s*=\s*\{\s*"A Pass"\s*,',
        )

    def test_track_view_displays_only_sensor_levels(self):
        body = self.function_body("track_view_render")
        self.assertIn('"Track View"', body)
        self.assertIn("track_update()", body)
        for index in range(8):
            self.assertIn(f"track_sensors[{index}]", body)
        self.assertNotIn("track_error_get", body)
        self.assertNotIn("track_active_count", body)
        self.assertNotIn("err:", body)

    def test_a_stop_uses_existing_task2_control(self):
        body = self.function_body("action_a_stop")
        self.assertRegex(
            body,
            r"g_a_stop_view_active\s*=\s*true\s*;\s*"
            r"control_set_run\s*\(\s*true\s*\)",
        )
        self.assertRegex(body, r"\bcontrol_set_run\s*\(\s*true\s*\)")
        self.assertNotIn("motor_", body)
        self.assertNotIn("step_", body)

    def test_a_stop_view_shows_live_time_sensor_bits_and_state(self):
        body = self.function_body("a_stop_render")
        self.assertIn("control_parking_elapsed_ms()", body)
        self.assertIn("track_update()", body)
        self.assertIn('"A Stop"', body)
        self.assertIn('"Time:%lu.%03lus"', body)
        self.assertIn('"IR:%u%u%u%u%u%u%u%u"', body)
        for index in range(8):
            self.assertIn(f"track_sensors[{index}]", body)
        for state in ("RUNNING", "PARK OK", "PARK FAIL", "START BLOCKED"):
            self.assertIn(f'"{state}"', body)
        self.assertNotIn("track_error_get", body)
        self.assertNotIn("track_active_count", body)

    def test_a_stop_view_owns_refresh_and_back_stops_safely(self):
        refresh = self.function_body("app_menu_refresh")
        self.assertRegex(
            refresh,
            r"if\s*\(\s*g_a_stop_view_active\s*\)\s*"
            r"\{\s*a_stop_render\s*\(\s*\)",
        )

        keys = self.function_body("app_menu_on_key")
        self.assertIn("g_a_stop_view_active", keys)
        self.assertRegex(keys, r"\bid\s*==\s*3U?\b")
        self.assertIn("KEY_EVENT_SHORT_PRESS", keys)
        self.assertRegex(keys, r"\bcontrol_set_run\s*\(\s*false\s*\)")
        self.assertRegex(keys, r"g_a_stop_view_active\s*=\s*false")
        self.assertIn("menu_invalidate(&g_menu)", keys)

    def test_parking_elapsed_getter_is_live_then_holds_final_value(self):
        control = CONTROL_C.read_text(encoding="utf-8")
        match = re.search(
            r"uint32_t\s+control_parking_elapsed_ms\s*\(\s*void\s*\)\s*"
            r"\{(?P<body>.*?)^\}",
            control,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")
        self.assertRegex(body, r"if\s*\(\s*start_run\s*\)")
        self.assertIn("g_tms", body)
        self.assertIn("g_parking.runStartMs", body)
        self.assertIn("return g_parking.elapsedMs", body)

    def test_full_lap_reuses_a_stop_course_and_holds_ball_center(self):
        body = self.function_body("action_full_lap")
        self.assertIn("BallTargetTask_stop()", body)
        self.assertRegex(body, r"control_set_run\s*\(\s*true\s*\)")
        self.assertRegex(body, r"if\s*\(\s*start_run\s*\)")
        self.assertIn("BallTargetTask_startCenterHold()", body)
        self.assertNotIn("control_start_ab", body)

    def test_ball_target_stops_vehicle_and_holds_ball_center(self):
        body = self.function_body("action_ball_target")
        self.assertIn("BallTargetTask_stop()", body)
        self.assertRegex(body, r"control_set_run\s*\(\s*false\s*\)")
        self.assertIn("BallTargetTask_startCenterHold()", body)
        self.assertNotIn("BallTargetTask_start()", body)
        self.assertNotRegex(body, r"control_set_run\s*\(\s*true\s*\)")

    def test_ball_only_uses_dedicated_task_without_starting_task2(self):
        body = self.function_body("action_ball_only")
        self.assertRegex(body, r"control_set_run\s*\(\s*false\s*\)")
        self.assertIn("BallTargetTask_start()", body)
        self.assertNotIn("BallTargetTask_startCenterHold()", body)
        self.assertNotRegex(body, r"control_set_run\s*\(\s*true\s*\)")

    def test_regular_back_stops_running_vehicle_and_ball_before_navigation(self):
        body = self.function_body("app_menu_on_key")
        back_case = re.search(
            r"case\s+3\s*:"
            r"(?P<body>.*?)menu_event\s*=\s*MENU_EVENT_BACK",
            body,
            re.DOTALL,
        )
        self.assertIsNotNone(back_case)
        self.assertIn("control_set_run(false)", back_case.group("body"))
        self.assertIn("BallTargetTask_stop()", back_case.group("body"))


if __name__ == "__main__":
    unittest.main()
