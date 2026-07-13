import math
from dataclasses import dataclass


@dataclass
class GapResult:
    open: bool = False
    has_gap: bool = False
    width: float = 0.0
    distance: float = 0.0
    center_angle: float = 0.0


class GapFinder:
    def __init__(self, required_clearance: float, min_open_distance: float):
        self.required_clearance = required_clearance
        self.min_open_distance = max(1e-3, min_open_distance)
        ratio = min(1.0, self.required_clearance / (2.0 * self.min_open_distance))
        self.boundary_span = 2.0 * math.asin(ratio)

        self.have_last_wall = False
        self.last_wall_distance = 0.0
        self.last_wall_angle = 0.0

        self.in_run = False
        self.left_edge_valid = False
        self.left_distance = 0.0
        self.left_angle = 0.0
        self.run_first_angle = 0.0
        self.run_last_angle = 0.0

        self.best = GapResult()

    def add_wall(self, distance: float, angle: float):
        if self.in_run:
            self.close_run(True, distance, angle)
        self.have_last_wall = True
        self.last_wall_distance = distance
        self.last_wall_angle = angle

    def add_free(self, angle: float):
        if not self.in_run:
            self.in_run = True
            self.left_edge_valid = self.have_last_wall
            self.left_distance = self.last_wall_distance
            self.left_angle = self.last_wall_angle
            self.run_first_angle = angle
        self.run_last_angle = angle

    def finish(self) -> GapResult:
        if self.in_run:
            self.close_run(False, 0.0, 0.0)
        self.best.open = self.best.has_gap and self.best.width >= self.required_clearance
        return self.best

    def close_run(self, right_edge_valid: bool, right_distance: float, right_angle: float):
        if self.left_edge_valid and right_edge_valid:
            span = right_angle - self.left_angle
            chord = math.sqrt(
                self.left_distance * self.left_distance
                + right_distance * right_distance
                - 2.0 * self.left_distance * right_distance * math.cos(span)
            )
            distance = min(self.left_distance, right_distance)
            near_chord = 2.0 * distance * math.sin(min(abs(span), math.pi) / 2.0)
            width = min(chord, near_chord)
            self.consider(width, distance, 0.5 * (self.left_angle + right_angle))
        else:
            span = abs(self.run_last_angle - self.run_first_angle)
            width = self.required_clearance if span >= self.boundary_span else 0.0
            self.consider(width, self.min_open_distance, 0.5 * (self.run_first_angle + self.run_last_angle))
        self.in_run = False

    def consider(self, width: float, distance: float, center_angle: float):
        if width > self.best.width:
            self.best.has_gap = True
            self.best.width = width
            self.best.distance = distance
            self.best.center_angle = center_angle
