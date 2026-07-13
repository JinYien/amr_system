from dataclasses import dataclass

MODES = ("manual", "auto")
AUTHORITIES = ("", "user", "robot", "mix")
MIDDLE_MODES = ("", "nudge", "pulse")
MIDDLE_ACTIONS = ("", "left", "right")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass
class Teleop:
    mode: str = "manual"
    authority: str = ""
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    middle_mode: str = ""
    middle_action: str = ""

    def set_velocity(self, linear: float, angular: float, max_linear: float, max_angular: float):
        self.linear_velocity = clamp(linear, -max_linear, max_linear)
        self.angular_velocity = clamp(angular, -max_angular, max_angular)

    def set_mode(self, mode: str):
        if mode in MODES and mode != self.mode:
            self.mode = mode
            self.authority = ""

    def set_authority(self, authority: str):
        if self.mode == "auto" and authority in AUTHORITIES:
            self.authority = authority

    def set_middle(self, mode: str, action: str):
        if mode in MIDDLE_MODES:
            self.middle_mode = mode
        if action in MIDDLE_ACTIONS:
            self.middle_action = action

    def clear_middle(self):
        self.middle_mode = ""
        self.middle_action = ""
