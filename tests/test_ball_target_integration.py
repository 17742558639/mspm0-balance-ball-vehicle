import re
import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]


class BallTargetIntegrationTests(unittest.TestCase):
    def text(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def function_body(self, source, name, return_type=r"(?:void|bool)"):
        match = re.search(
            rf"{return_type}\s+{name}\s*\([^)]*\)\s*"
            r"\{(?P<body>.*?)^\}",
            source,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match, f"missing function {name}")
        return match.group("body")

    def test_tuned_vehicle_constants_remain_unchanged(self):
        course = self.text("APP/course_speed.h")
        parking = self.text("APP/parking.h")
        task4 = self.text("APP/task4_ab.h")

        self.assertRegex(course, r"COURSE_START_SPEED\s*\(?12\)?")
        self.assertRegex(course, r"COURSE_SLOW_SPEED\s*\(?6\)?")
        self.assertRegex(course, r"COURSE_RAMP_STEP_MS\s*\(?100U?\)?")
        self.assertRegex(
            course, r"COURSE_SLOWDOWN_DELAY_MS\s*\(?10000U?\)?")
        self.assertRegex(
            parking, r"PARK_TARGET_ACTIVE_COUNT\s*\(?4U?\)?")
        self.assertRegex(
            parking, r"PARK_SLOW_TARGET_MIN_ACTIVE_COUNT\s*\(?3U?\)?")
        self.assertRegex(
            parking, r"PARK_FORWARD_CONFIRM_TICKS\s*\(?3U?\)?")
        self.assertRegex(parking, r"PARK_BRAKE_HOLD_MS\s*\(?100U?\)?")
        self.assertRegex(task4, r"TASK4_AB_SPEED\s*\(?11\)?")

    def test_existing_menu_layout_is_unchanged(self):
        menu = self.text("APP/menu/app_menu.c")
        root_match = re.search(
            r"static\s+const\s+MenuItem\s+main_items\s*\[\s*\]\s*=\s*"
            r"\{(?P<body>.*?)^\};",
            menu,
            re.MULTILINE | re.DOTALL,
        )
        a_pass_match = re.search(
            r"static\s+const\s+MenuItem\s+a_pass_items\s*\[\s*\]\s*=\s*"
            r"\{(?P<body>.*?)^\};",
            menu,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(root_match)
        self.assertIsNotNone(a_pass_match)
        self.assertEqual(
            re.findall(
                r'^\s*\{"([^"]+)"', root_match.group("body"), re.MULTILINE),
            ["Track View", "A Stop", "A Pass", "Ball Only"],
        )
        self.assertEqual(
            re.findall(
                r'^\s*\{"([^"]+)"',
                a_pass_match.group("body"),
                re.MULTILINE,
            ),
            ["AB", "Full Lap", "Ball Target"],
        )

    def test_wheel_encoder_direction_expressions_are_preserved(self):
        encoder = self.text("BSP/Encoder.c")
        self.assertRegex(
            encoder,
            re.compile(
                r"GPIO_ENCODER_BR_PIN_BR_B_PORT.*?encoder_br--;.*?"
                r"else\s*encoder_br\+\+;",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            encoder,
            re.compile(
                r"GPIO_ENCODER_BL_PIN_BL_B_PORT.*?encoder_bl\+\+;.*?"
                r"else\s*encoder_bl--;",
                re.DOTALL,
            ),
        )

    def test_generated_config_exposes_ball_resources(self):
        header = self.text("Debug/syscfg/ti_msp_dl_config.h")
        for symbol in (
            "GPIO_ENCODER_STEP_PORT",
            "GPIO_ENCODER_STEP_PIN_STEP_A_PIN",
            "GPIO_ENCODER_STEP_PIN_STEP_B_PIN",
            "TIMER_BALL_INST",
            "TIMER_BALL_INST_INT_IRQN",
        ):
            with self.subTest(symbol=symbol):
                self.assertIn(symbol, header)

    def test_stepper_keeps_two_axis_api_and_adds_ball_api(self):
        header = self.text("BSP/step_motor.h")
        source = self.text("BSP/step_motor.c")
        self.assertRegex(
            header, r"void\s+step_off\s*\(\s*uint8_t\s+id\s*\)\s*;")
        self.assertRegex(
            header,
            r"void\s+step_on\s*\(\s*int\s+step_num\s*,\s*uint8_t\s+id\s*\)\s*;",
        )
        self.assertIn(
            "void step_on_at_freq(int32_t step_num, int32_t hz);", header)
        self.assertIn("void step_drive_hz(int32_t hz);", header)
        self.assertIn("STEP_MOTOR_2_INST_IRQHandler", source)
        self.assertRegex(
            source,
            r"DL_Timer_disableInterrupt\s*\(\s*STEP_MOTOR_1_INST",
        )
        self.assertIn("DL_TIMER_INTERRUPT_CC0_DN_EVENT", source)

    def test_step_encoder_is_appended_to_shared_gpio_isr(self):
        header = self.text("BSP/Encoder.h")
        source = self.text("BSP/Encoder.c")
        self.assertIn("extern volatile int32_t encoder_step;", header)
        self.assertIn("GPIO_ENCODER_STEP_PIN_STEP_A_PIN", source)
        self.assertIn("GPIO_ENCODER_STEP_PIN_STEP_B_PIN", source)
        self.assertIn("volatile int32_t encoder_step = 0", source)

    def test_uart_camera_owns_ball_commands(self):
        uart = self.text("BSP/uart.c")
        header = self.text("BSP/uart.h")
        self.assertIn("#define UART_CMD_BALL_POSITION  0x21U", header)
        self.assertIn("#define UART_CMD_BALL_COUNT     0x31U", header)
        self.assertIn("ball_measurement_update(position)", uart)
        self.assertIn("g_ball_count", uart)
        self.assertRegex(uart, r"ch\s*==\s*UART_CH_CAMERA")

    def test_red_and_pid_text_paths_are_not_active(self):
        uart_h = self.text("BSP/uart.h")
        uart_c = self.text("BSP/uart.c")
        control = self.text("APP/Control.c")
        for obsolete in (
            "UART_CMD_RED_STATE",
            "uart_communicate_take_red_state",
            "uart_tuning_take_line",
            "pid_tuning_task",
            "VisionMotorTrigger_onRedState",
        ):
            with self.subTest(obsolete=obsolete):
                self.assertNotIn(obsolete, uart_h + uart_c + control)

    def test_ball_loop_is_isolated_on_timer_ball(self):
        expected_files = (
            "APP/ball.h",
            "APP/ball.c",
            "MID/mechanism.h",
            "MID/mechanism.c",
        )
        missing = [relative for relative in expected_files
                   if not (ROOT / relative).is_file()]
        self.assertEqual(missing, [])

        ball_h = self.text("APP/ball.h")
        ball_c = self.text("APP/ball.c")
        pid_h = self.text("MID/pid.h")
        for api in (
            "void ball_init(void);",
            "void ball_loop_run(void);",
            "void ball_loop_stop(void);",
            "void set_ball_position(int16_t pos);",
            "bool wait_ball_pos(int16_t pos);",
            "void ball_measurement_update(int16_t position);",
        ):
            with self.subTest(api=api):
                self.assertIn(api, ball_h)
        self.assertIn("TIMER_BALL_INST_IRQHandler", ball_c)
        self.assertIn("DL_TimerG_startCounter(TIMER_BALL_INST)", ball_c)
        self.assertIn("step_on_at_freq", ball_c)
        self.assertNotIn("TIMER_0_INST", ball_c)
        self.assertIn("extern PidObject PID_StepPos;", pid_h)
        self.assertIn("extern PidObject PID_BallPos;", pid_h)

    def test_ball_task_exposes_independent_center_hold_mode(self):
        header = self.text("APP/ball_target_task.h")
        source = self.text("APP/ball_target_task.c")

        self.assertIn("BALL_TARGET_TASK_CENTER_HOLD", header)
        self.assertIn(
            "void BallTargetTask_startCenterHold(void);", header)

        body = self.function_body(
            source, "BallTargetTask_startCenterHold")
        self.assertRegex(
            body,
            re.compile(
                r"ball_loop_stop\s*\(\s*\)\s*;.*?"
                r"set_ball_position\s*\(\s*0\s*\)\s*;.*?"
                r"ball_loop_run\s*\(\s*\)\s*;.*?"
                r"s_state\s*=\s*BALL_TARGET_TASK_CENTER_HOLD",
                re.DOTALL,
            ),
        )

    def test_ball_only_sequence_entry_remains_separate(self):
        source = self.text("APP/ball_target_task.c")
        body = self.function_body(source, "BallTargetTask_start")

        self.assertIn("set_ball_position(50)", body)
        self.assertNotIn("BallTargetTask_startCenterHold", body)
        self.assertNotIn("set_ball_position(0)", body)

    def test_firmware_ball_target_sequence_matches_policy(self):
        self.assertTrue((ROOT / "APP/ball_target_task.c").is_file())
        source = self.text("APP/ball_target_task.c")
        self.assertIn("set_ball_position(50)", source)
        self.assertIn("wait_ball_pos(50)", source)
        self.assertIn("set_ball_position(-50)", source)
        self.assertIn("wait_ball_pos(-50)", source)
        self.assertIn("BALL_TARGET_TASK_HOLD", source)
        self.assertIn("BALL_TARGET_BUZZER_MS", source)
        self.assertNotIn("delay_ms", source)

    def test_system_initializes_and_ticks_ball_task(self):
        system = self.text("APP/system.c")
        self.assertIn("ball_init();", system)
        self.assertIn("BallTargetTask_init();", system)
        self.assertIn("BallTargetTask_update((uint32_t)g_tms);", system)

    def test_long_press_stops_ball_before_toggling_vehicle(self):
        system = self.text("APP/system.c")
        self.assertRegex(
            system,
            re.compile(
                r"id\s*==\s*3.*?KEY_EVENT_LONG_PRESS.*?"
                r"BallTargetTask_stop\s*\(\s*\)\s*;.*?"
                r"control_toggle_run\s*\(\s*\)\s*;",
                re.DOTALL,
            ),
        )

    def test_ball_only_mapping_does_not_change_menu_arrays(self):
        menu = self.text("APP/menu/app_menu.c")
        match = re.search(
            r"static\s+void\s+action_ball_only\s*\(\s*void\s*\)\s*"
            r"\{(?P<body>.*?)^\}",
            menu,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match)
        self.assertIn("BallTargetTask_start();", match.group("body"))
        self.test_existing_menu_layout_is_unchanged()


if __name__ == "__main__":
    unittest.main()
