from enum import Enum, auto

TARGET_COUNT = 4
SLOW_TARGET_MIN_COUNT = 3
LEAVE_TICKS = 5
FORWARD_CONFIRM_TICKS = 3
BRAKE_MS = 100


class State(Enum):
    IDLE = auto()
    WAIT_LEAVE_A = auto()
    RUNNING = auto()
    BRAKE_CHECK = auto()
    STOPPED = auto()
    FAILED = auto()


class Action(Enum):
    FORWARD = auto()
    STOP = auto()


class Controller:
    def __init__(self):
        self.cancel()

    def start(self, now_ms):
        self.state = State.WAIT_LEAVE_A
        self.run_start_ms = now_ms
        self.state_start_ms = now_ms
        self.elapsed_ms = 0
        self.leave_ticks = 0
        self.forward_ticks = 0

    def cancel(self):
        self.state = State.IDLE
        self.run_start_ms = 0
        self.state_start_ms = 0
        self.elapsed_ms = 0
        self.leave_ticks = 0
        self.forward_ticks = 0

    @property
    def has_result(self):
        return self.state in (State.STOPPED, State.FAILED)

    def finish(self, state, now_ms):
        self.state = state
        self.elapsed_ms = now_ms - self.run_start_ms
        return Action.STOP

    def update(self, active_count, now_ms, relaxed_target=False):
        if self.state == State.WAIT_LEAVE_A:
            if active_count == TARGET_COUNT:
                self.leave_ticks = 0
            else:
                self.leave_ticks += 1
                if self.leave_ticks >= LEAVE_TICKS:
                    self.state = State.RUNNING
            return Action.FORWARD

        if self.state == State.RUNNING:
            target_detected = (
                active_count >= SLOW_TARGET_MIN_COUNT
                if relaxed_target
                else active_count == TARGET_COUNT
            )
            if target_detected:
                self.forward_ticks += 1
                if self.forward_ticks >= FORWARD_CONFIRM_TICKS:
                    self.state = State.BRAKE_CHECK
                    self.state_start_ms = now_ms
                    return Action.STOP
            else:
                self.forward_ticks = 0
            return Action.FORWARD

        if self.state == State.BRAKE_CHECK:
            if now_ms - self.state_start_ms < BRAKE_MS:
                return Action.STOP
            return self.finish(State.STOPPED, now_ms)

        return Action.STOP
