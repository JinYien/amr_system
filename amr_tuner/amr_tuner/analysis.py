#!/usr/bin/env python3

from collections import deque
import numpy as np
from amr_tuner.settings import AnalysisSettings


class PidCalculator:
    def __init__(self, settings: AnalysisSettings, max_periods: int = 10):
        self.settings = settings
        self.periods = deque(maxlen=max_periods)
        self.last_peak_time = None
        self.recent_values = deque(maxlen=settings.peak_detection_window)

    def add_data_point(self, time: float, velocity: float) -> bool:
        self.recent_values.append(velocity)

        if len(self.recent_values) < self.settings.peak_detection_window:
            return False

        values = list(self.recent_values)
        midpoint = len(values) // 2
        if values[midpoint] != max(values):
            return False

        if self.last_peak_time is None:
            self.last_peak_time = time
            return False

        period = time - self.last_peak_time
        if not (self.settings.min_valid_period < period < self.settings.max_valid_period):
            return False

        self.periods.append(period)
        self.last_peak_time = time
        return True

    def get_stats(self) -> dict:
        if len(self.periods) < 2:
            return {"period": None, "frequency": None, "count": 0}

        recent = list(self.periods)[-self.settings.average_period_window :]
        average_period = float(np.mean(recent))
        return {
            "period": average_period,
            "frequency": 1.0 / average_period,
            "count": len(self.periods),
        }

    @staticmethod
    def calculate_pid(ultimate_gain: float, ultimate_period: float) -> dict:
        ku, tu = ultimate_gain, ultimate_period
        return {
            "p": {"Kp": 0.50 * ku, "Ki": 0.00, "Kd": 0.000 * ku * tu},
            "pi": {"Kp": 0.45 * ku, "Ki": 0.54 * ku / tu, "Kd": 0.000 * ku * tu},
            "pd": {"Kp": 0.80 * ku, "Ki": 0.00, "Kd": 0.125 * ku * tu},
            "pid": {"Kp": 0.60 * ku, "Ki": 1.20 * ku / tu, "Kd": 0.075 * ku * tu},
            "pessen": {"Kp": 0.70 * ku, "Ki": 1.75 * ku / tu, "Kd": 0.105 * ku * tu},
            "some_overshoot": {"Kp": 0.33 * ku, "Ki": 0.66 * ku / tu, "Kd": 0.110 * ku * tu},
            "no_overshoot": {"Kp": 0.20 * ku, "Ki": 0.40 * ku / tu, "Kd": 0.066 * ku * tu},
        }

    def reset(self):
        self.periods.clear()
        self.last_peak_time = None
        self.recent_values.clear()
