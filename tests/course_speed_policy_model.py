from enum import Enum, auto


SLOWDOWN_DELAY_MS = 10000
SLOW_SPEED = 8
START_SPEED = 12
RAMP_STEP_MS = 100
UINT32_MASK = 0xFFFFFFFF


class State(Enum):
    IDLE = auto()
    WAIT_DELAY = auto()
    SLOW = auto()


class Controller:
    def __init__(self):
        self.cancel()

    def start(self, now_ms):
        self.state = State.WAIT_DELAY
        self.run_start_ms = now_ms & UINT32_MASK

    def cancel(self):
        self.state = State.IDLE
        self.run_start_ms = 0

    def update(self, now_ms, requested_speed):
        if self.state == State.IDLE:
            return requested_speed

        elapsed_ms = (now_ms - self.run_start_ms) & UINT32_MASK
        if (self.state == State.WAIT_DELAY and
                elapsed_ms >= SLOWDOWN_DELAY_MS):
            self.state = State.SLOW

        if self.state == State.SLOW:
            return min(requested_speed, SLOW_SPEED)
        if requested_speed <= START_SPEED:
            return requested_speed
        return min(
            requested_speed,
            START_SPEED + elapsed_ms // RAMP_STEP_MS,
        )
