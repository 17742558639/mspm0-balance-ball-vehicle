from enum import Enum, auto


SPEED = 11
START_SPEED = 7
RAMP_STEP_MS = 100
TARGET_PULSES = 12500
TIMEOUT_MS = 8000
UINT32_MASK = 0xFFFFFFFF
UINT32_MAX = 0xFFFFFFFF


class State(Enum):
    IDLE = auto()
    RUNNING = auto()
    PASSED = auto()
    TIMEOUT = auto()


class Controller:
    def __init__(self):
        self.cancel()

    def start(self, now_ms):
        self.state = State.RUNNING
        self.run_start_ms = now_ms & UINT32_MASK
        self.elapsed_ms = 0
        self.left_pulses = 0
        self.right_pulses = 0

    def cancel(self):
        self.state = State.IDLE
        self.run_start_ms = 0
        self.elapsed_ms = 0
        self.left_pulses = 0
        self.right_pulses = 0

    @staticmethod
    def _add_saturated(total, delta):
        if delta <= 0:
            return total
        return min(UINT32_MAX, total + delta)

    def record_encoder(self, left, right):
        if self.state != State.RUNNING:
            return
        self.left_pulses = self._add_saturated(self.left_pulses, left)
        self.right_pulses = self._add_saturated(self.right_pulses, right)

    @property
    def distance_pulses(self):
        return ((self.left_pulses // 2) + (self.right_pulses // 2) +
                ((self.left_pulses % 2 + self.right_pulses % 2) // 2))

    @property
    def has_result(self):
        return self.state in (State.PASSED, State.TIMEOUT)

    @property
    def succeeded(self):
        return self.state == State.PASSED

    def command_speed(self, now_ms):
        if self.state != State.RUNNING:
            return 0
        elapsed = (now_ms - self.run_start_ms) & UINT32_MASK
        return min(SPEED, START_SPEED + elapsed // RAMP_STEP_MS)

    def update(self, now_ms):
        if self.state != State.RUNNING:
            return
        self.elapsed_ms = (now_ms - self.run_start_ms) & UINT32_MASK
