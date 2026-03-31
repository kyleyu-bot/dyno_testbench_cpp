#!/usr/bin/env python3
"""
Dyno Qt GUI — launches bridge_ros2 and provides drive control.

Usage (from repo root, after sourcing ROS2 and env_setup):
    python3 src/interface_bridges/ros2/dyno_gui.py [options]

Options:
    --bridge   <path>     Path to bridge_ros2 binary
                          (default: build/dyno_ros2_bridge/bridge_ros2)
    --topology <path>     Topology JSON passed to bridge_ros2
                          (default: config/topology.dyno2.template6.json)
    --pub-hz   <float>    ROS2 publish rate for /dyno/command (default: 20)
    --no-bridge           Don't launch bridge_ros2 (connect to an already-running bridge)
    --fault-reset-s <s>   fault_reset_s passed to bridge_ros2 (default: 2.0)

Controls:
    Main speed slider  →  main_speed  (rad/s LSB)
    DUT  speed slider  →  dut_speed
    Main Enable button →  main_enable (toggle)
    DUT  Enable button →  dut_enable  (toggle)
    Fault Reset button →  momentary fault_reset pulse
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

# ── Qt ──────────────────────────────────────────────────────────────────────
try:
    from PyQt5.QtCore    import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication, QWidget, QVBoxLayout, QHBoxLayout,
        QSlider, QPushButton, QLabel, QGroupBox,
        QSpinBox,
    )
except ImportError:
    print("ERROR: PyQt5 not found.  Install with:  pip install PyQt5", file=sys.stderr)
    sys.exit(1)

# ── ROS2 ────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node          import Node
    from std_msgs.msg        import String as StringMsg
except ImportError:
    print("ERROR: rclpy not found.  Source ROS2:  source /opt/ros/humble/setup.bash",
          file=sys.stderr)
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# Defaults
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_BRIDGE   = "build/dyno_ros2_bridge/bridge_ros2"
DEFAULT_TOPOLOGY = "config/topology.dyno2.template6.json"
DEFAULT_PUB_HZ   = 200.0
DEFAULT_FAULT_S  = 2.0
SPEED_MAX        = 1000   # rad/s LSB slider range


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 commander node (runs on a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DynoCommander(Node):
    """Publishes /dyno/command at a fixed rate from the current GUI state."""

    def __init__(self, pub_hz: float):
        super().__init__("dyno_gui_commander")
        self._pub = self.create_publisher(StringMsg, "/dyno/command", 10)

        # Shared command state — written by GUI thread, read by ROS timer.
        self._lock        = threading.Lock()
        self._main_speed  = 0
        self._dut_speed   = 0
        self._main_enable = False
        self._dut_enable  = False
        self._fault_reset = False
        self._hold_output1 = False

        period = 1.0 / max(pub_hz, 1.0)
        self.create_timer(period, self._publish)

    # ── called by GUI thread ─────────────────────────────────────────────────

    def set_command(self, main_speed: int, dut_speed: int,
                    main_enable: bool, dut_enable: bool,
                    fault_reset: bool = False, hold_output1: bool = False):
        with self._lock:
            self._main_speed   = main_speed
            self._dut_speed    = dut_speed
            self._main_enable  = main_enable
            self._dut_enable   = dut_enable
            self._fault_reset  = fault_reset
            self._hold_output1 = hold_output1

    def pulse_fault_reset(self):
        """Send fault_reset=true for one publish cycle."""
        with self._lock:
            self._fault_reset = True

    # ── ROS timer callback ───────────────────────────────────────────────────

    def _publish(self):
        with self._lock:
            payload = {
                "main_speed":    self._main_speed,
                "dut_speed":     self._dut_speed,
                "main_enable":   self._main_enable,
                "dut_enable":    self._dut_enable,
                "fault_reset":   self._fault_reset,
                "hold_output1":  self._hold_output1,
            }
            # fault_reset is a one-shot pulse — clear after publishing.
            self._fault_reset = False

        msg      = StringMsg()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Qt main window
# ─────────────────────────────────────────────────────────────────────────────

class DynoWindow(QWidget):

    def __init__(self, commander: DynoCommander):
        super().__init__()
        self._cmd = commander
        self._main_enabled  = False
        self._dut_enabled   = False
        self._hold_output1  = False
        self._build_ui()
        self.setWindowTitle("Dyno Control")

        # Publish timer driven by Qt (mirrors ROS publish rate).
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._push_command)
        self._timer.start(5)    # 200 Hz

    # ── UI construction ──────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)

        drives_row = QHBoxLayout()

        # ── Main drive ───────────────────────────────────────────────────────
        drives_row.addWidget(self._make_drive_group(
            label         = "Main Drive",
            attr_speed    = "_main_speed_slider",
            attr_spinbox  = "_main_speed_spin",
            attr_label    = "_main_speed_label",
            attr_btn      = "_main_enable_btn",
            attr_min_spin = "_main_min_spin",
            attr_max_spin = "_main_max_spin",
            on_enable     = self._toggle_main,
        ))

        # ── DUT drive ────────────────────────────────────────────────────────
        drives_row.addWidget(self._make_drive_group(
            label         = "DUT Drive",
            attr_speed    = "_dut_speed_slider",
            attr_spinbox  = "_dut_speed_spin",
            attr_label    = "_dut_speed_label",
            attr_btn      = "_dut_enable_btn",
            attr_min_spin = "_dut_min_spin",
            attr_max_spin = "_dut_max_spin",
            on_enable     = self._toggle_dut,
        ))

        root.addLayout(drives_row)

        # ── Output / Fault row ───────────────────────────────────────────────
        btn_row = QHBoxLayout()

        self._output1_btn = QPushButton("Hold Output 1")
        self._output1_btn.setCheckable(True)
        self._output1_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44aaff; font-weight: bold; }")
        self._output1_btn.clicked.connect(self._toggle_output1)
        btn_row.addWidget(self._output1_btn)

        fault_btn = QPushButton("Fault Reset")
        fault_btn.setStyleSheet("background-color: #f0a000; font-weight: bold;")
        fault_btn.clicked.connect(self._fault_reset)
        btn_row.addWidget(fault_btn)

        root.addLayout(btn_row)

        # ── Status label ─────────────────────────────────────────────────────
        self._status_label = QLabel("bridge_ros2 starting…")
        self._status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self._status_label)

    def _make_drive_group(self, label, attr_speed, attr_spinbox,
                          attr_label, attr_btn, attr_min_spin, attr_max_spin,
                          on_enable):
        grp   = QGroupBox(label)
        outer = QHBoxLayout(grp)

        # ── Left column: max spinbox / vertical slider / min spinbox ─────────
        slider_col = QVBoxLayout()

        max_spin = QSpinBox()
        max_spin.setMinimum(-SPEED_MAX)
        max_spin.setMaximum( SPEED_MAX)
        max_spin.setValue(SPEED_MAX)
        max_spin.setPrefix("Max: ")
        setattr(self, attr_max_spin, max_spin)
        slider_col.addWidget(max_spin)

        slider = QSlider(Qt.Vertical)
        slider.setMinimum(-SPEED_MAX)
        slider.setMaximum( SPEED_MAX)
        slider.setValue(0)
        slider.setTickInterval(SPEED_MAX // 4)
        slider.setTickPosition(QSlider.TicksRight)
        slider.setMinimumHeight(180)
        setattr(self, attr_speed, slider)
        slider_col.addWidget(slider, alignment=Qt.AlignHCenter)

        min_spin = QSpinBox()
        min_spin.setMinimum(-SPEED_MAX)
        min_spin.setMaximum( SPEED_MAX)
        min_spin.setValue(-SPEED_MAX)
        min_spin.setPrefix("Min: ")
        setattr(self, attr_min_spin, min_spin)
        slider_col.addWidget(min_spin)

        outer.addLayout(slider_col)

        # ── Right column: value label / exact entry / buttons ─────────────────
        ctrl_col = QVBoxLayout()

        speed_lbl = QLabel("Speed (LSB):\n0")
        speed_lbl.setAlignment(Qt.AlignCenter)
        setattr(self, attr_label, speed_lbl)
        ctrl_col.addWidget(speed_lbl)

        ctrl_col.addWidget(QLabel("Exact:"))
        spinbox = QSpinBox()
        spinbox.setMinimum(-SPEED_MAX)
        spinbox.setMaximum( SPEED_MAX)
        spinbox.setValue(0)
        setattr(self, attr_spinbox, spinbox)
        ctrl_col.addWidget(spinbox)

        ctrl_col.addStretch()

        enable_btn = QPushButton("Enable")
        enable_btn.setCheckable(True)
        enable_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44cc44; font-weight: bold; }")
        enable_btn.clicked.connect(on_enable)
        setattr(self, attr_btn, enable_btn)
        ctrl_col.addWidget(enable_btn)

        zero_btn = QPushButton("Zero")
        zero_btn.clicked.connect(lambda: slider.setValue(0))
        ctrl_col.addWidget(zero_btn)

        outer.addLayout(ctrl_col)

        # ── Signal wiring ─────────────────────────────────────────────────────
        max_spin.valueChanged.connect(lambda v: (
            slider.setMaximum(v), spinbox.setMaximum(v)))
        min_spin.valueChanged.connect(lambda v: (
            slider.setMinimum(v), spinbox.setMinimum(v)))

        spinbox.valueChanged.connect(
            lambda v: (slider.blockSignals(True), slider.setValue(v),
                       slider.blockSignals(False)))
        slider.valueChanged.connect(lambda v: (
            speed_lbl.setText(f"Speed (LSB):\n{v}"),
            spinbox.blockSignals(True),
            spinbox.setValue(v),
            spinbox.blockSignals(False),
        ))

        return grp

    # ── Slot helpers ─────────────────────────────────────────────────────────

    def _toggle_main(self):
        self._main_enabled = self._main_enable_btn.isChecked()
        self._main_enable_btn.setText("Enabled" if self._main_enabled else "Enable")

    def _toggle_dut(self):
        self._dut_enabled = self._dut_enable_btn.isChecked()
        self._dut_enable_btn.setText("Enabled" if self._dut_enabled else "Enable")

    def _toggle_output1(self):
        self._hold_output1 = self._output1_btn.isChecked()
        self._output1_btn.setText("Output 1 ON" if self._hold_output1 else "Hold Output 1")

    def _fault_reset(self):
        self._cmd.pulse_fault_reset()

    # ── Timer: push current state to ROS2 commander ──────────────────────────

    def _push_command(self):
        self._cmd.set_command(
            main_speed   = self._main_speed_slider.value(),
            dut_speed    = self._dut_speed_slider.value(),
            main_enable  = self._main_enabled,
            dut_enable   = self._dut_enabled,
            hold_output1 = self._hold_output1,
        )

    def set_status(self, text: str):
        self._status_label.setText(text)


# ─────────────────────────────────────────────────────────────────────────────
# Bridge subprocess management
# ─────────────────────────────────────────────────────────────────────────────

def launch_bridge(bridge_path: str, topology: str, fault_reset_s: float, pub_hz: float, debug: bool = False):
    """Launch bridge_ros2 as a subprocess with sudo -E."""
    cmd = [
        "sudo", "-E",
        bridge_path,
        "--ros-args",
        "-p", f"topology:={topology}",
        "-p", f"fault_reset_s:={fault_reset_s}",
        "-p", f"pub_hz:={pub_hz}",
    ]
    if debug:
        cmd += ["-p", "debug:=1"]
    print(f"[dyno_gui] Launching: {' '.join(cmd)}")
    # Inherit stdout/stderr so bridge output appears in the terminal.
    proc = subprocess.Popen(cmd)
    return proc


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Dyno Qt GUI + ROS2 command publisher")
    p.add_argument("--bridge",        default=DEFAULT_BRIDGE,
                   help=f"Path to bridge_ros2 binary (default: {DEFAULT_BRIDGE})")
    p.add_argument("--topology",      default=DEFAULT_TOPOLOGY,
                   help=f"Topology JSON (default: {DEFAULT_TOPOLOGY})")
    p.add_argument("--pub-hz",        type=float, default=DEFAULT_PUB_HZ,
                   help=f"Command publish rate Hz (default: {DEFAULT_PUB_HZ})")
    p.add_argument("--fault-reset-s", type=float, default=DEFAULT_FAULT_S,
                   help=f"fault_reset_s for bridge (default: {DEFAULT_FAULT_S})")
    p.add_argument("--no-bridge",     action="store_true",
                   help="Don't launch bridge_ros2 (assume it's already running)")
    p.add_argument("--debug",         action="store_true",
                   help="Pass debug:=1 to bridge_ros2 (enables verbose status prints)")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Launch bridge subprocess ─────────────────────────────────────────────
    bridge_proc = None
    if not args.no_bridge:
        if not os.path.isfile(args.bridge):
            print(f"ERROR: bridge binary not found at '{args.bridge}'\n"
                  f"       Build it first:  bash src/interface_bridges/ros2/build.sh",
                  file=sys.stderr)
            sys.exit(1)
        bridge_proc = launch_bridge(args.bridge, args.topology, args.fault_reset_s, args.pub_hz, args.debug)

    # ── ROS2 init ────────────────────────────────────────────────────────────
    rclpy.init()
    commander = DynoCommander(pub_hz=args.pub_hz)

    # Spin ROS2 on a background thread so the Qt event loop owns the main thread.
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(commander,), daemon=True
    )
    ros_thread.start()

    # ── Qt GUI ───────────────────────────────────────────────────────────────
    app    = QApplication(sys.argv)
    window = DynoWindow(commander)
    window.resize(420, 380)

    if bridge_proc is not None:
        window.set_status(f"bridge_ros2 PID {bridge_proc.pid}")
    else:
        window.set_status("--no-bridge: connecting to existing bridge")

    window.show()

    # Graceful shutdown on Ctrl+C in terminal.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    exit_code = app.exec_()

    # ── Cleanup ──────────────────────────────────────────────────────────────
    print("[dyno_gui] Window closed — shutting down.")

    # Zero and disable before tearing down ROS.
    commander.set_command(0, 0, False, False, hold_output1=False)
    time.sleep(0.1)

    rclpy.shutdown()

    if bridge_proc is not None:
        print(f"[dyno_gui] Sending SIGINT to bridge_ros2 (PID {bridge_proc.pid})…")
        bridge_proc.send_signal(signal.SIGINT)
        try:
            bridge_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge_proc.kill()
        print("[dyno_gui] bridge_ros2 stopped.")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
