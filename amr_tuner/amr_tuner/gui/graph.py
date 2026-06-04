#!/usr/bin/env python3

from collections import deque
import numpy as np
import pyqtgraph as pg
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
)
from amr_tuner.settings import GraphSettings


class GraphPanel(QGroupBox):
    def __init__(self, settings: GraphSettings, parent=None):
        super().__init__("Velocity Graph", parent)
        self.settings = settings
        self.time_data = deque()
        self.right_velocity_data = deque()
        self.left_velocity_data = deque()
        self.start_time = None
        self.time_window = settings.default_time_window
        self.is_running = False
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.addLayout(self.make_time_window_control())
        self.plot_widget = self.make_plot_widget()
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def make_time_window_control(self):
        layout = QHBoxLayout()
        label = QLabel("Time Window:")
        label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(label)

        self.time_window_spinbox = QDoubleSpinBox()
        self.time_window_spinbox.setRange(self.settings.min_time_window, self.settings.max_time_window)
        self.time_window_spinbox.setSingleStep(self.settings.time_window_step)
        self.time_window_spinbox.setDecimals(self.settings.time_window_decimals)
        self.time_window_spinbox.setValue(self.settings.default_time_window)
        self.time_window_spinbox.setMinimumWidth(80)
        self.time_window_spinbox.setSuffix(" s")
        self.time_window_spinbox.setSpecialValueText("Auto")
        self.time_window_spinbox.valueChanged.connect(lambda v: setattr(self, "time_window", v))
        layout.addWidget(self.time_window_spinbox)
        layout.addStretch()
        return layout

    def make_plot_widget(self):
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setLabel("left", "Velocity (deg/s)")
        plot.setLabel("bottom", "Time (s)")
        plot.addLegend()
        plot.showGrid(x=True, y=True, alpha=0.3)

        self.right_curve = plot.plot(pen=pg.mkPen(color="r", width=2), name="Right Motor")
        self.left_curve = plot.plot(pen=pg.mkPen(color="b", width=2), name="Left Motor")
        return plot

    def start(self):
        self.is_running = True

    def stop(self):
        self.is_running = False

    def add_data_point(self, timestamp: float, right_velocity: float, left_velocity: float):
        if not self.is_running:
            return
        if self.start_time is None:
            self.start_time = timestamp

        self.time_data.append(timestamp - self.start_time)
        self.right_velocity_data.append(right_velocity)
        self.left_velocity_data.append(left_velocity)

    def update_plot(self):
        if not self.time_data:
            return

        precision = self.settings.decimal_precision
        time_array = np.array(self.time_data)
        right_array = np.round(np.array(self.right_velocity_data), precision)
        left_array = np.round(np.array(self.left_velocity_data), precision)

        self.right_curve.setData(time_array, right_array)
        self.left_curve.setData(time_array, left_array)

        if self.time_window > 0:
            self.apply_time_window(time_array)
        else:
            self.plot_widget.enableAutoRange()

    def apply_time_window(self, time_array):
        if len(time_array) == 0:
            return
        max_time = time_array[-1]
        self.plot_widget.setXRange(max_time - self.time_window, max_time, padding=0)
        self.plot_widget.disableAutoRange()

    def get_recent_amplitude(self, side: str = "right", num_samples: int = None):
        if num_samples is None:
            num_samples = self.settings.amplitude_sample_size
        data = self.right_velocity_data if side == "right" else self.left_velocity_data
        if len(data) < num_samples:
            return None
        recent = list(data)[-num_samples:]
        return float(np.max(recent) - np.min(recent))

    def reset(self):
        self.time_data.clear()
        self.right_velocity_data.clear()
        self.left_velocity_data.clear()
        self.start_time = None
        self.is_running = False
        self.right_curve.setData([], [])
        self.left_curve.setData([], [])
        self.plot_widget.enableAutoRange()
