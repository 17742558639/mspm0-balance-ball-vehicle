import unittest

try:
    from ball_target_policy_model import Controller, State
except ModuleNotFoundError:
    from tests.ball_target_policy_model import Controller, State


class BallTargetPolicyTests(unittest.TestCase):
    def test_start_targets_positive_fifty(self):
        c = Controller()
        self.assertEqual(c.start(), 50)
        self.assertEqual(c.state, State.WAIT_POSITIVE)

    def test_positive_arrival_switches_to_negative_fifty(self):
        c = Controller()
        c.start()
        self.assertEqual(c.update(arrived=True), -50)
        self.assertEqual(c.state, State.WAIT_NEGATIVE)

    def test_negative_arrival_enters_hold(self):
        c = Controller()
        c.start()
        c.update(True)
        self.assertIsNone(c.update(arrived=True))
        self.assertEqual(c.state, State.HOLD)

    def test_stop_returns_to_idle(self):
        c = Controller()
        c.start()
        c.stop()
        self.assertEqual(c.state, State.IDLE)


if __name__ == "__main__":
    unittest.main()
