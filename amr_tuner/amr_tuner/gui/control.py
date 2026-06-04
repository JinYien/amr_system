#!/usr/bin/env python3

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
)
from amr_tuner.settings import RangeSettings, VelocitySettings


class ControlPanel(QGroupBox):
    pid_apply_clicked = pyqtSignal(float, float, float)
    velocity_changed = pyqtSignal(float, float)
    start_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()
    reset_clicked = pyqtSignal()
    calculate_clicked = pyqtSignal()

    def __init__(self, gain: RangeSettings, velocity: VelocitySettings, parent=None):
        super().__init__("PID and Velocity Control", parent)
        self.gain = gain
        self.velocity = velocity
        self.init_ui()

    def init_ui(self):
        layout = QGridLayout()
        row = 0

        layout.addWidget(QLabel("Proportional Gain (Kp):"), row, 0)
        self.kp_spinbox = self.make_pid_spinbox()
        layout.addWidget(self.kp_spinbox, row, 1)

        row += 1
        layout.addWidget(QLabel("Integral Gain (Ki):"), row, 0)
        self.ki_spinbox = self.make_pid_spinbox()
        layout.addWidget(self.ki_spinbox, row, 1)

        row += 1
        layout.addWidget(QLabel("Derivative Gain (Kd):"), row, 0)
        self.kd_spinbox = self.make_pid_spinbox()
        layout.addWidget(self.kd_spinbox, row, 1)

        self.set_pid_button = QPushButton("Set PID")
        self.set_pid_button.clicked.connect(self.on_set_pid_clicked)
        self.set_pid_button.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 8px;")
        layout.addWidget(self.set_pid_button, 0, 2, 3, 3)

        row += 1
        self.left_slider, self.left_value_label, self.left_spinbox = self.make_velocity_controls()
        self.add_velocity_row(
            layout, row, "Left Motor Velocity (deg/s):", self.left_slider, self.left_value_label, self.left_spinbox
        )

        row += 1
        self.right_slider, self.right_value_label, self.right_spinbox = self.make_velocity_controls()
        self.add_velocity_row(
            layout, row, "Right Motor Velocity (deg/s):", self.right_slider, self.right_value_label, self.right_spinbox
        )

        row += 1
        layout.addLayout(self.make_button_row(), row, 0, 1, 5)
        self.setLayout(layout)

    def add_velocity_row(self, layout, row, label_text, slider, value_label, spinbox):
        layout.addWidget(QLabel(label_text), row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)
        layout.addWidget(QLabel("Fine:"), row, 3)
        layout.addWidget(spinbox, row, 4)

    def make_pid_spinbox(self):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(self.gain.min, self.gain.max)
        spinbox.setSingleStep(self.gain.step)
        spinbox.setDecimals(self.gain.decimals)
        spinbox.setValue(0.0)
        spinbox.setMinimumWidth(150)
        return spinbox

    def make_velocity_controls(self):
        scale = self.velocity.slider_scale
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(self.velocity.min * scale), int(self.velocity.max * scale))
        slider.setValue(0)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(self.velocity.slider_tick_interval)
        slider.valueChanged.connect(lambda v: self.on_velocity_changed(v / scale, slider))

        value_label = QLabel("0.0")
        value_label.setFont(QFont("Arial", 10))
        value_label.setMinimumWidth(80)

        spinbox = QDoubleSpinBox()
        spinbox.setRange(self.velocity.min, self.velocity.max)
        spinbox.setSingleStep(self.velocity.step)
        spinbox.setValue(0.0)
        spinbox.valueChanged.connect(lambda v: self.on_velocity_changed(v, spinbox))

        return slider, value_label, spinbox

    def make_button_row(self):
        layout = QHBoxLayout()

        self.start_button = QPushButton("Start Motor")
        self.start_button.clicked.connect(self.start_clicked.emit)
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Motor")
        self.stop_button.clicked.connect(self.stop_clicked.emit)
        self.stop_button.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
        self.stop_button.setEnabled(False)
        layout.addWidget(self.stop_button)

        self.reset_button = QPushButton("Reset Graph")
        self.reset_button.clicked.connect(self.reset_clicked.emit)
        self.reset_button.setStyleSheet("padding: 10px;")
        layout.addWidget(self.reset_button)

        self.calculate_button = QPushButton("Calculate PID")
        self.calculate_button.clicked.connect(self.calculate_clicked.emit)
        self.calculate_button.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(self.calculate_button)

        return layout

    def on_velocity_changed(self, value, source):
        groups = (
            (self.left_slider, self.left_spinbox, self.left_value_label),
            (self.right_slider, self.right_spinbox, self.right_value_label),
        )
        for slider, spinbox, label in groups:
            if source in (slider, spinbox):
                self.sync_velocity_widgets(value, slider, spinbox, label)
                break

        self.velocity_changed.emit(self.left_spinbox.value(), self.right_spinbox.value())

    def sync_velocity_widgets(self, value, slider, spinbox, label):
        label.setText(f"{value:.{self.velocity.decimals}f}")
        slider.blockSignals(True)
        slider.setValue(int(value * self.velocity.slider_scale))
        slider.blockSignals(False)
        spinbox.blockSignals(True)
        spinbox.setValue(value)
        spinbox.blockSignals(False)

    def on_set_pid_clicked(self):
        self.pid_apply_clicked.emit(self.kp_spinbox.value(), self.ki_spinbox.value(), self.kd_spinbox.value())

    def get_pid_values(self):
        return self.kp_spinbox.value(), self.ki_spinbox.value(), self.kd_spinbox.value()

    def get_velocities(self):
        return self.left_spinbox.value(), self.right_spinbox.value()

    def enable_start_button(self, enabled: bool = True):
        self.start_button.setEnabled(enabled)

    def enable_stop_button(self, enabled: bool = True):
        self.stop_button.setEnabled(enabled)
