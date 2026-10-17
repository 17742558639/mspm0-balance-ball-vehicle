from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    WAIT_POSITIVE = auto()
    WAIT_NEGATIVE = auto()
    HOLD = auto()


class Controller:
    def __init__(self):
        self.state = State.IDLE

    def start(self):
        self.state = State.WAIT_POSITIVE
        return 50

    def update(self, arrived):
        if not arrived:
            return None
        if self.state is State.WAIT_POSITIVE:
            self.state = State.WAIT_NEGATIVE
            return -50
        if self.state is State.WAIT_NEGATIVE:
            self.state = State.HOLD
        return None

    def stop(self):
        self.state = State.IDLE
