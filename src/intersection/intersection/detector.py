import math
from dataclasses import dataclass
from intersection.geometry import Sector, normalize_angle, in_sector
from intersection.gap_finder import GapFinder, GapResult

UNKNOWN = 0
DEADEND = 1
CORRIDOR = 2
LEFT_CORNER = 3
RIGHT_CORNER = 4
LEFT_JUNCTION = 5
RIGHT_JUNCTION = 6
T_JUNCTION = 7
CROSS = 8
OPEN_SPACE = 9

JUNCTION_LABELS = {
    UNKNOWN: "unknown",
    DEADEND: "deadend",
    CORRIDOR: "corridor",
    LEFT_CORNER: "left_corner",
    RIGHT_CORNER: "right_corner",
    LEFT_JUNCTION: "left_junction",
    RIGHT_JUNCTION: "right_junction",
    T_JUNCTION: "t_junction",
    CROSS: "cross",
    OPEN_SPACE: "open_space",
}

_JUNCTION_TABLE = {
    (False, False, False): DEADEND,
    (True, False, False): CORRIDOR,
    (False, True, False): LEFT_CORNER,
    (False, False, True): RIGHT_CORNER,
    (True, True, False): LEFT_JUNCTION,
    (True, False, True): RIGHT_JUNCTION,
    (False, True, True): T_JUNCTION,
    (True, True, True): CROSS,
}


@dataclass
class DetectorParams:
    front: Sector
    left: Sector
    right: Sector
    min_open_distance: float
    required_clearance: float
    lidar_yaw_offset: float
    open_space_distance: float


@dataclass
class DetectionResult:
    front: GapResult
    left: GapResult
    right: GapResult
    nearest: float = float("inf")


@dataclass
class Decision:
    front: bool = False
    left: bool = False
    right: bool = False


def classify_junction(
    front_open: bool,
    left_open: bool,
    right_open: bool,
    nearest: float = float("inf"),
    open_space_distance: float = float("inf"),
    previous: int = UNKNOWN,
    hysteresis: float = 0.0,
) -> int:
    junction = _JUNCTION_TABLE[(bool(front_open), bool(left_open), bool(right_open))]
    if junction == CROSS:
        threshold = open_space_distance - (hysteresis if previous == OPEN_SPACE else 0.0)
        if nearest >= threshold:
            return OPEN_SPACE
    return junction


class Detector:
    def __init__(self, params: DetectorParams):
        self.params = params

    def detect(self, scan) -> DetectionResult:
        front = GapFinder(self.params.required_clearance, self.params.min_open_distance)
        left = GapFinder(self.params.required_clearance, self.params.min_open_distance)
        right = GapFinder(self.params.required_clearance, self.params.min_open_distance)
        nearest = float("inf")

        for index, range_value in enumerate(scan.ranges):
            raw_angle = scan.angle_min + index * scan.angle_increment
            robot_angle = normalize_angle(raw_angle + self.params.lidar_yaw_offset)

            if in_sector(robot_angle, self.params.front):
                finder = front
            elif in_sector(robot_angle, self.params.left):
                finder = left
            elif in_sector(robot_angle, self.params.right):
                finder = right
            else:
                continue

            if math.isnan(range_value) or math.isinf(range_value) or range_value > scan.range_max:
                finder.add_free(raw_angle)
            elif range_value < scan.range_min:
                finder.add_wall(scan.range_min, raw_angle)
                nearest = min(nearest, scan.range_min)
            elif range_value >= self.params.min_open_distance:
                finder.add_free(raw_angle)
                nearest = min(nearest, range_value)
            else:
                finder.add_wall(range_value, raw_angle)
                nearest = min(nearest, range_value)

        return DetectionResult(front.finish(), left.finish(), right.finish(), nearest)
