#!/usr/bin/env python3

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
)

PID_METHODS = (
    ("p", "P", "#795548"),
    ("pi", "PI", "#607D8B"),
    ("pd", "PD", "#009688"),
    ("pid", "PID", "#E91E63"),
    ("pessen", "Pessen Integral", "#FF9800"),
    ("some_overshoot", "Some Overshoot", "#2196F3"),
    ("no_overshoot", "No Overshoot", "#4CAF50"),
)

SIDE_STYLES = {
    "right": {"label": "Right", "color": "#FF0022", "background": "#FFEBEE"},
    "left": {"label": "Left", "color": "#0095FF", "background": "#E3F2FD"},
}


def _label(text="", bold=False, size=10, color=None, align=None):
    label = QLabel(text)
    font = QFont("Arial", size)
    if bold:
        font.setBold(True)
    label.setFont(font)
    if color:
        label.setStyleSheet(f"color: {color};")
    if align is not None:
        label.setAlignment(align)
    label.setMinimumWidth(60)
    return label


def _frame():
    frame = QFrame()
    frame.setFrameStyle(QFrame.Box | QFrame.Plain)
    frame.setLineWidth(2)
    frame.setStyleSheet("QFrame { border: 2px solid #BDBDBD; border-radius: 4px;" " background-color: white; padding: 5px; }")
    return frame


def _separator():
    separator = QFrame()
    separator.setFrameShape(QFrame.VLine)
    separator.setFrameShadow(QFrame.Sunken)
    separator.setLineWidth(1)
    separator.setStyleSheet("QFrame { color: #BDBDBD; }")
    separator.setFixedWidth(2)
    return separator


class SidePanel(QFrame):
    def __init__(self, side_key, parent=None):
        super().__init__(parent)
        style = SIDE_STYLES[side_key]
        self.side_key = side_key
        self.side_label = style["label"]
        self.accent_color = style["color"]
        self.accent_background = style["background"]

        self.period_label = None
        self.frequency_label = None
        self.amplitude_label = None
        self.pid_value_labels = {}

        self.build_ui()

    def build_ui(self):
        self.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.setLineWidth(1)
        self.setStyleSheet("QFrame { border: 1px solid #E0E0E0; border-radius: 6px;" " background-color: #FAFAFA; }")

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        header = _label(f"{self.side_label} Motor", bold=True, size=12, align=Qt.AlignCenter)
        header.setStyleSheet(f"color: white; background-color: {self.accent_color};" " padding: 6px; border-radius: 4px;")
        layout.addWidget(header)

        layout.addWidget(self.build_oscillation_frame())
        layout.addWidget(self.build_pid_frame())
        layout.addStretch()

    def build_oscillation_frame(self):
        frame = _frame()
        layout = QVBoxLayout(frame)

        title = _label("Oscillation", bold=True, size=11, align=Qt.AlignCenter)
        title.setStyleSheet(f"background-color: {self.accent_background}; padding: 6px; border-radius: 4px;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(5)
        grid.setContentsMargins(10, 5, 10, 5)

        self.period_label = self.make_value_label()
        self.frequency_label = self.make_value_label()
        self.amplitude_label = self.make_value_label()

        entries = (
            ("Period (Tu)", self.period_label),
            ("Frequency", self.frequency_label),
            ("Amplitude", self.amplitude_label),
        )
        for column, (header_text, value_label) in enumerate(entries):
            if column > 0:
                grid.addWidget(_separator(), 0, column * 2 - 1, 2, 1)
            header_label = _label(header_text, bold=True, size=10, align=Qt.AlignCenter)
            header_label.setFixedHeight(28)
            grid.addWidget(header_label, 0, column * 2)
            value_label.setFixedHeight(28)
            grid.addWidget(value_label, 1, column * 2)
            grid.setColumnStretch(column * 2, 1)

        layout.addLayout(grid)
        return frame

    def build_pid_frame(self):
        frame = _frame()
        layout = QVBoxLayout(frame)

        title = _label("PID", bold=True, size=11, align=Qt.AlignCenter)
        title.setStyleSheet(f"background-color: {self.accent_background}; padding: 6px; border-radius: 4px;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(5)
        grid.setContentsMargins(10, 5, 10, 5)

        headers = ("Method", "Kp", "Ki", "Kd")
        method_row_span = len(PID_METHODS) + 1
        for column, header_text in enumerate(headers):
            if column > 0:
                grid.addWidget(_separator(), 0, column * 2 - 1, method_row_span, 1)
            header_label = _label(header_text, bold=True, size=10, align=Qt.AlignCenter)
            header_label.setFixedHeight(28)
            grid.addWidget(header_label, 0, column * 2)
            grid.setColumnStretch(column * 2, 2 if column == 0 else 1)

        for row, (method_key, display_name, color) in enumerate(PID_METHODS, start=1):
            method_label = _label(display_name, bold=True, size=10, align=Qt.AlignCenter)
            method_label.setStyleSheet(f"color: white; background-color: {color}; padding: 5px; border-radius: 3px;")
            method_label.setFixedHeight(28)
            grid.addWidget(method_label, row, 0)

            for column, gain_key in enumerate(("Kp", "Ki", "Kd"), start=1):
                value_label = _label("N/A", bold=True, size=10, align=Qt.AlignCenter)
                value_label.setFixedHeight(28)
                self.pid_value_labels[(method_key, gain_key)] = value_label
                grid.addWidget(value_label, row, column * 2)

        layout.addLayout(grid)
        return frame

    @staticmethod
    def make_value_label():
        return _label("N/A", bold=True, size=11, color="#1976D2", align=Qt.AlignCenter)

    def update_oscillation_metrics(self, period, frequency, amplitude):
        self.period_label.setText(f"{period:.3f} s" if period else "N/A")
        self.frequency_label.setText(f"{frequency:.3f} Hz" if frequency else "N/A")
        self.amplitude_label.setText(f"{amplitude:.3f} deg/s" if amplitude else "N/A")

    def update_pid_results(self, results):
        for method_key, _, _ in PID_METHODS:
            gains = results.get(method_key)
            if gains is None:
                continue
            kp, ki, kd = gains["Kp"], gains["Ki"], gains["Kd"]
            self.pid_value_labels[(method_key, "Kp")].setText(f"{kp:.5f}")
            self.pid_value_labels[(method_key, "Ki")].setText(f"{ki:.5f}" if ki > 0 else "N/A")
            self.pid_value_labels[(method_key, "Kd")].setText(f"{kd:.5f}" if kd > 0 else "N/A")

    def clear_results(self):
        self.update_oscillation_metrics(None, None, None)
        for label in self.pid_value_labels.values():
            label.setText("N/A")


class PidCalculatorPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("PID Calculation", parent)
        self.sides = {}
        self.build_ui()

    def build_ui(self):
        layout = QHBoxLayout()
        layout.setSpacing(10)
        for side_key in ("left", "right"):
            panel = SidePanel(side_key)
            self.sides[side_key] = panel
            layout.addWidget(panel)
        self.setLayout(layout)

    def update_oscillation_metrics(self, side, period, frequency, amplitude):
        self.sides[side].update_oscillation_metrics(period, frequency, amplitude)

    def update_pid_results(self, side, results):
        self.sides[side].update_pid_results(results)

    def clear_results(self, side=None):
        targets = self.sides.values() if side is None else (self.sides[side],)
        for panel in targets:
            panel.clear_results()
