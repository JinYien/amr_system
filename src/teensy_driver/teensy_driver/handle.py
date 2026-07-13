from typing import Optional
from teensy_driver.settings import HapticPattern


class HandleController:
    def __init__(self, nudge: HapticPattern, pulse: HapticPattern):
        self.nudge = nudge
        self.pulse = pulse
        self.mode = None
        self.direction = None
        self.pulse_step = 0
        self.start_time = 0.0
        self.pending = False

    def request_nudge(self, direction: str):
        if self.mode != "nudge":
            self.mode = "nudge"
            self.direction = direction
            self.pending = True

    def request_pulse(self):
        if self.mode is None:
            self.mode = "pulse"
            self.pending = True

    def update(self, now: float) -> Optional[float]:
        if self.mode == "nudge":
            return self.update_nudge(now)
        if self.mode == "pulse":
            return self.update_pulse(now)
        return None

    def update_nudge(self, now: float) -> Optional[float]:
        if self.pending:
            self.pending = False
            self.start_time = now
            sign = -1.0 if self.direction == "left" else 1.0
            return sign * self.nudge.torque

        if now - self.start_time > self.nudge.hold_time:
            self.mode = None
            return 0.0
        return None

    def update_pulse(self, now: float) -> Optional[float]:
        if self.pending:
            self.pending = False
            self.start_time = now
            self.pulse_step = 0
            return None

        if now - self.start_time < self.pulse.hold_time:
            return None

        if self.pulse_step < len(self.pulse.sequence):
            torque = self.pulse.sequence[self.pulse_step] * self.pulse.torque
            self.pulse_step += 1
            self.start_time = now
            return torque

        self.mode = None
        return None
