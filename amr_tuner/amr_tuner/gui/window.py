#!/usr/bin/env python3

import time
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QMainWindow, QVBoxLayout, QWidget
from amr_tuner.analysis import PidCalculator
from amr_tuner.gui.control import ControlPanel
from amr_tuner.gui.converter import VelocityCalculatorPanel
from amr_tuner.gui.graph import GraphPanel
from amr_tuner.gui.tuning import PidCalculatorPanel
from amr_tuner.settings import Settings

SIDES = ("right", "left")


class MainWindow(QMainWindow):
    def __init__(self, settings: Settings):
        super().__init__()
        self.settings = settings
        self.ros_node = None
        self.is_running = False
        self.tuning_calculators = {side: PidCalculator(settings.analysis) for side in SIDES}

        self.init_ui()
        self.connect_signals()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(settings.window.gui_update_interval)

    def init_ui(self):
        window = self.settings.window
        self.setWindowTitle("PID Tuner - Ziegler-Nichols Method")
        self.setGeometry(window.x, window.y, window.width, window.height)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)

        self.control_panel = ControlPanel(self.settings.pid_gain, self.settings.velocity)
        self.velocity_calculator_panel = VelocityCalculatorPanel(self.settings.wheel, self.settings.linear_velocity)
        self.graph_panel = GraphPanel(self.settings.graph)
        self.pid_calculator_panel = PidCalculatorPanel()

        top = QHBoxLayout()
        top.addWidget(self.control_panel, window.left_panel)
        top.addWidget(self.velocity_calculator_panel, window.right_panel)
        main_layout.addLayout(top)

        bottom = QHBoxLayout()
        bottom.addWidget(self.graph_panel, window.left_panel)
        bottom.addWidget(self.pid_calculator_panel, window.right_panel)
        main_layout.addLayout(bottom)

        self.statusBar()

    def connect_signals(self):
        self.control_panel.pid_apply_clicked.connect(self.on_pid_apply)
        self.control_panel.velocity_changed.connect(self.on_velocity_changed)
        self.control_panel.start_clicked.connect(self.start_test)
        self.control_panel.stop_clicked.connect(self.stop_test)
        self.control_panel.reset_clicked.connect(self.reset_graphs)
        self.control_panel.calculate_clicked.connect(self.calculate_pid)

    def on_pid_apply(self, kp, ki, kd):
        decimals = self.settings.pid_gain.decimals
        self.ros_node.send_pid_gain(round(kp, decimals), round(ki, decimals), round(kd, decimals))

    def on_velocity_changed(self, left_velocity, right_velocity):
        if self.is_running:
            self.ros_node.send_velocity_command(left_velocity, right_velocity)

    def start_test(self):
        kp, ki, kd = self.control_panel.get_pid_values()
        self.ros_node.send_pid_gain(kp, ki, kd)

        left_velocity, right_velocity = self.control_panel.get_velocities()
        self.ros_node.send_velocity_command(left_velocity, right_velocity)

        self.is_running = True
        self.graph_panel.start()
        self.control_panel.enable_start_button(False)
        self.control_panel.enable_stop_button(True)

    def stop_test(self):
        self.ros_node.send_velocity_command(0.0, 0.0)
        self.is_running = False
        self.graph_panel.stop()
        self.control_panel.enable_start_button(True)
        self.control_panel.enable_stop_button(False)

    def reset_graphs(self):
        self.graph_panel.reset()
        for calculator in self.tuning_calculators.values():
            calculator.reset()
        self.pid_calculator_panel.clear_results()
        if self.is_running:
            self.graph_panel.start()

    def calculate_pid(self):
        ultimate_gain, _, _ = self.control_panel.get_pid_values()
        minimum_count = self.settings.analysis.min_oscillation_periods

        for side, calculator in self.tuning_calculators.items():
            stats = calculator.get_stats()
            if stats["period"] is None or stats["count"] < minimum_count:
                self.statusBar().showMessage(f"Unable to calculate {side} velocity PID — increase Kp gradually.", 5000)
                continue
            results = calculator.calculate_pid(ultimate_gain, stats["period"])
            self.pid_calculator_panel.update_pid_results(side, results)

    def motor_data(self, right_velocity, left_velocity):
        if not self.is_running:
            return

        velocity = self.settings.velocity
        right_velocity = min(max(right_velocity, velocity.min), velocity.max)
        left_velocity = min(max(left_velocity, velocity.min), velocity.max)

        now = time.time()
        self.graph_panel.add_data_point(now, right_velocity, left_velocity)
        self.tuning_calculators["right"].add_data_point(now, right_velocity)
        self.tuning_calculators["left"].add_data_point(now, left_velocity)

    def update_gui(self):
        self.graph_panel.update_plot()
        for side, calculator in self.tuning_calculators.items():
            stats = calculator.get_stats()
            amplitude = self.graph_panel.get_recent_amplitude(side)
            self.pid_calculator_panel.update_oscillation_metrics(side, stats["period"], stats["frequency"], amplitude)
