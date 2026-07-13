import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Sector:
    min_angle: float
    max_angle: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def in_sector(angle: float, sector: Sector) -> bool:
    if sector.min_angle <= sector.max_angle:
        return sector.min_angle <= angle <= sector.max_angle
    return angle >= sector.min_angle or angle <= sector.max_angle
