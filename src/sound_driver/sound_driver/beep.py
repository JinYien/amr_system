from typing import Optional, Tuple
from sound_driver.settings import BeepSettings


class BeepController:
    def __init__(self, settings: BeepSettings):
        self.settings = settings

    def pattern(self, distance: float) -> Optional[Tuple[float, float]]:
        settings = self.settings
        if distance <= 0.0 or distance > settings.far_distance:
            return None
        if distance <= settings.near_distance:
            return settings.near_interval, settings.near_interval
        span = settings.far_distance - settings.near_distance
        fraction = (distance - settings.near_distance) / span
        period = settings.near_interval + (settings.far_interval - settings.near_interval) * fraction
        on_time = max(0.0, period - settings.gap)
        return period, on_time
