#!/usr/bin/env python3

import math
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
)
from amr_tuner.settings import RangeSettings, WheelSettings


class VelocityCalculatorPanel(QGroupBox):
    def __init__(self, wheel: WheelSettings, linear_velocity: RangeSettings, parent=None):
        super().__init__("Velocity Calculation", parent)
        self.wheel = wheel
        self.linear_velocity = linear_velocity
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        grid = QGridLayout()

        grid.addWidget(QLabel("Wheel Radius (m):"), 0, 0)
        self.radius_spinbox = QDoubleSpinBox()
        self.radius_spinbox.setRange(self.wheel.min_radius, self.wheel.max_radius)
        self.radius_spinbox.setSingleStep(self.wheel.radius_step)
        self.radius_spinbox.setDecimals(self.wheel.radius_decimals)
        self.radius_spinbox.setValue(self.wheel.default_radius)
        self.radius_spinbox.setSuffix(" m")
        self.radius_spinbox.valueChanged.connect(self.update_conversion)
        grid.addWidget(self.radius_spinbox, 0, 1)

        grid.addWidget(QLabel("Linear Velocity (m/s):"), 1, 0)
        self.linear_velocity_spinbox = QDoubleSpinBox()
        self.linear_velocity_spinbox.setRange(self.linear_velocity.min, self.linear_velocity.max)
        self.linear_velocity_spinbox.setSingleStep(self.linear_velocity.step)
        self.linear_velocity_spinbox.setDecimals(self.linear_velocity.decimals)
        self.linear_velocity_spinbox.setValue(0.0)
        self.linear_velocity_spinbox.setSuffix(" m/s")
        self.linear_velocity_spinbox.valueChanged.connect(self.update_conversion)
        grid.addWidget(self.linear_velocity_spinbox, 1, 1)

        layout.addLayout(grid)
        layout.addSpacing(20)

        result_label = QLabel("Angular Velocity:")
        result_label.setFont(QFont("Arial", 14, QFont.Bold))
        result_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(result_label)

        self.angular_deg_label = self.make_result_label("0.00 deg/s", "#0D47A1", "#E3F2FD", "#BBDEFB")
        layout.addWidget(self.angular_deg_label)

        self.angular_rad_label = self.make_result_label("0.00 rad/s", "#1B5E20", "#E8F5E9", "#C8E6C9")
        layout.addWidget(self.angular_rad_label)

        layout.addStretch()
        self.setLayout(layout)

    @staticmethod
    def make_result_label(text, foreground, background, border):
        label = QLabel(text)
        label.setFont(QFont("Arial", 12, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(
            f"color: {foreground}; background-color: {background};"
            f" padding: 10px; border-radius: 8px; border: 1px solid {border};"
        )
        return label

    def update_conversion(self):
        radius = self.radius_spinbox.value()
        linear = self.linear_velocity_spinbox.value()

        if radius <= 0:
            self.angular_deg_label.setText("Invalid radius")
            self.angular_rad_label.setText("Invalid radius")
            return

        angular_rad_s = linear / radius
        angular_deg_s = angular_rad_s * (180.0 / math.pi)
        self.angular_deg_label.setText(f"{angular_deg_s:.2f} deg/s")
        self.angular_rad_label.setText(f"{angular_rad_s:.2f} rad/s")
