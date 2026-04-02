#!/usr/bin/env python3
"""
Dyno Qt GUI — drag-and-drop command slider assignment.

Usage (from repo root):
    bash src/interface_bridges/ros2/run_gui.sh [options]

Options:
    --bridge   <path>     Path to bridge_ros2 binary
                          (default: build/dyno_ros2_bridge/bridge_ros2)
    --topology <path>     Topology JSON passed to bridge_ros2
                          (default: config/topology.dyno2.template6.json)
    --pub-hz   <float>    ROS2 publish rate Hz (default: 200)
    --no-bridge           Don't launch bridge_ros2 (connect to already-running bridge)
    --fault-reset-s <s>   fault_reset_s passed to bridge_ros2 (default: 2.0)
    --debug               Pass debug:=1 to bridge_ros2

Controls:
    Drag a field from the left panel onto a slider slot to assign it.
    Right-click a slot to unassign.
    Main Enable / DUT Enable  — toggle drive enable
    Main Zero  / DUT Zero     — zero all four command types for that drive
    Fault Reset               — one-shot fault clear pulse
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

# ── Qt ────────────────────────────────────────────────────────────────────────
try:
    from PyQt5.QtCore    import Qt, QTimer, QMimeData, QByteArray
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QSplitter,
        QSlider, QPushButton, QLabel, QGroupBox,
        QSpinBox, QListWidget, QListWidgetItem,
        QMenu, QAction, QComboBox,
    )
except ImportError:
    print("ERROR: PyQt5 not found.  pip install PyQt5", file=sys.stderr)
    sys.exit(1)

# ── ROS2 ─────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node   import Node
    from std_msgs.msg import String as StringMsg
except ImportError:
    print("ERROR: rclpy not found.  source /opt/ros/humble/setup.bash", file=sys.stderr)
    sys.exit(1)

# ── Constants ─────────────────────────────────────────────────────────────────

DEFAULT_BRIDGE   = "build/dyno_ros2_bridge/bridge_ros2"
DEFAULT_TOPOLOGY = "config/topology.dyno2.template6.json"
DEFAULT_PUB_HZ   = 200.0
DEFAULT_FAULT_S  = 2.0

CMD_MIME  = "application/x-dyno-command-field"
NUM_SLOTS = 4   # number of slider slots shown

# All drag-assignable command fields: (json_key, display_label)
COMMAND_FIELDS = [
    ("main_velocity", "Main Velocity"),
    ("main_position", "Main Position"),
    ("main_torque",   "Main Torque"),
    ("main_current",  "Main Current"),
    ("dut_velocity",  "DUT Velocity"),
    ("dut_position",  "DUT Position"),
    ("dut_torque",    "DUT Torque"),
    ("dut_current",   "DUT Current"),
]

# DS402 modes of operation: (display label, int value sent in JSON)
DS402_MODES = [
    ("No Mode (0)",                  0),
    ("Profile Position (1)",         1),
    ("Profile Velocity (2)",         2),
    ("Profile Torque (4)",           4),
    ("Cyclic Sync Position (8)",     8),
    ("Cyclic Sync Velocity (9)",     9),
    ("Cyclic Sync Torque (10)",     10),
]
DS402_DEFAULT_MODE = 9   # Cyclic Sync Velocity

MAIN_ZERO_FIELDS = ["main_velocity", "main_position", "main_torque", "main_current"]
DUT_ZERO_FIELDS  = ["dut_velocity",  "dut_position",  "dut_torque",  "dut_current"]
ALL_CMD_KEYS     = [k for k, _ in COMMAND_FIELDS]


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 commander node (runs on a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DynoCommander(Node):
    """Publishes /dyno/command at a fixed rate from the current GUI state."""

    def __init__(self, pub_hz: float):
        super().__init__("dyno_gui_commander")
        self._pub = self.create_publisher(StringMsg, "/dyno/command", 10)

        self._lock         = threading.Lock()
        self._numeric      = {k: 0 for k in ALL_CMD_KEYS}
        self._main_enable  = False
        self._dut_enable   = False
        self._fault_reset  = False
        self._hold_output1 = False
        self._main_mode    = DS402_DEFAULT_MODE
        self._dut_mode     = DS402_DEFAULT_MODE

        # Drive limits — updated from status topics, used to auto-set slider ranges.
        self._limits_lock  = threading.Lock()
        self._main_limits  = {"max_velocity_abs": 0.0, "min_position": 0, "max_position": 0}
        self._dut_limits   = {"max_velocity_abs": 0.0, "min_position": 0, "max_position": 0}

        self.create_subscription(StringMsg, "/dyno/main_drive/status",
            lambda msg: self._on_status(msg, "main"), 10)
        self.create_subscription(StringMsg, "/dyno/dut/status",
            lambda msg: self._on_status(msg, "dut"), 10)

        period = 1.0 / max(pub_hz, 1.0)
        self.create_timer(period, self._publish)

    def set_command(self, numeric: dict,
                    main_enable: bool, dut_enable: bool,
                    fault_reset: bool = False, hold_output1: bool = False,
                    main_mode: int = DS402_DEFAULT_MODE,
                    dut_mode:  int = DS402_DEFAULT_MODE):
        with self._lock:
            self._numeric.update(numeric)
            self._main_enable  = main_enable
            self._dut_enable   = dut_enable
            self._fault_reset  = fault_reset
            self._hold_output1 = hold_output1
            self._main_mode    = main_mode
            self._dut_mode     = dut_mode

    def _on_status(self, msg: StringMsg, drive: str) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        limits = {
            "max_velocity_abs": float(data.get("max_velocity_abs", 0.0)),
            "min_position":     int(data.get("min_position", 0)),
            "max_position":     int(data.get("max_position", 0)),
        }
        with self._limits_lock:
            if drive == "main":
                self._main_limits = limits
            else:
                self._dut_limits = limits

    def get_limits(self, field_key: str):
        """Return (min, max) slider limits for a command field, or None if unavailable."""
        _INT32_MIN = -(2 ** 31)
        _INT32_MAX =  (2 ** 31) - 1

        if field_key.startswith("main_"):
            with self._limits_lock:
                limits = dict(self._main_limits)
            field_type = field_key[5:]   # strip "main_"
        elif field_key.startswith("dut_"):
            with self._limits_lock:
                limits = dict(self._dut_limits)
            field_type = field_key[4:]   # strip "dut_"
        else:
            return None

        if field_type == "velocity":
            max_vel = int(limits["max_velocity_abs"])
            if max_vel > 0:
                return (-max_vel, max_vel)
        elif field_type == "position":
            lo = limits["min_position"]
            hi = limits["max_position"]
            # Skip if drive reported no limits (stored as INT32 extremes)
            if lo != _INT32_MIN and hi != _INT32_MAX and not (lo == 0 and hi == 0):
                return (lo, hi)
        return None

    def pulse_fault_reset(self):
        """Send fault_reset=true for one publish cycle."""
        with self._lock:
            self._fault_reset = True

    def _publish(self):
        with self._lock:
            payload = dict(self._numeric)
            payload["main_enable"]  = self._main_enable
            payload["dut_enable"]   = self._dut_enable
            payload["fault_reset"]  = self._fault_reset
            payload["hold_output1"] = self._hold_output1
            payload["main_mode"]    = self._main_mode
            payload["dut_mode"]     = self._dut_mode
            self._fault_reset = False   # one-shot pulse

        msg      = StringMsg()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Command field list (drag source)
# ─────────────────────────────────────────────────────────────────────────────

class CommandFieldList(QListWidget):
    """Static list of assignable command fields — drag onto a SliderSlot."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setDragDropMode(QListWidget.DragOnly)
        self.setMaximumWidth(150)
        self.setMinimumWidth(110)

        for key, label in COMMAND_FIELDS:
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, key)
            self.addItem(item)

    def mimeData(self, items):
        mime = QMimeData()
        if items:
            key = items[0].data(Qt.UserRole) or ""
            mime.setData(CMD_MIME, QByteArray(key.encode()))
        return mime


# ─────────────────────────────────────────────────────────────────────────────
# Slider slot (drop target)
# ─────────────────────────────────────────────────────────────────────────────

class SliderSlot(QGroupBox):
    """
    Generic vertical slider, initially unassigned.
    Drop a command field from CommandFieldList to assign what it controls.
    Right-click to unassign.
    """

    _PLACEHOLDER = "— drop field here —"

    def __init__(self, on_drop=None, parent=None):
        super().__init__(SliderSlot._PLACEHOLDER, parent)
        self._field: str | None = None
        self._on_drop = on_drop   # callable(field_key) -> (min, max) | None
        self.setAcceptDrops(True)
        self.setMinimumWidth(110)

        _SPIN_LO = -(2 ** 30)
        _SPIN_HI =  (2 ** 30) - 1

        # Max spinbox
        self._max_spin = QSpinBox()
        self._max_spin.setRange(_SPIN_LO, _SPIN_HI)
        self._max_spin.setValue(1000)
        self._max_spin.setPrefix("Max: ")

        # Vertical slider
        self._slider = QSlider(Qt.Vertical)
        self._slider.setRange(-1000, 1000)
        self._slider.setValue(0)
        self._slider.setTickInterval(500)
        self._slider.setTickPosition(QSlider.TicksRight)
        self._slider.setMinimumHeight(180)

        # Min spinbox
        self._min_spin = QSpinBox()
        self._min_spin.setRange(_SPIN_LO, _SPIN_HI)
        self._min_spin.setValue(-1000)
        self._min_spin.setPrefix("Min: ")

        # Exact entry spinbox
        self._exact_spin = QSpinBox()
        self._exact_spin.setRange(-1000, 1000)
        self._exact_spin.setValue(0)

        # Value display
        self._val_label = QLabel("0")
        self._val_label.setAlignment(Qt.AlignCenter)

        col = QVBoxLayout(self)
        col.addWidget(self._max_spin)
        col.addWidget(self._slider, 1, Qt.AlignHCenter)
        col.addWidget(self._min_spin)
        col.addWidget(QLabel("Exact:"))
        col.addWidget(self._exact_spin)
        col.addWidget(self._val_label)

        # Signal wiring
        self._max_spin.valueChanged.connect(self._on_max_changed)
        self._min_spin.valueChanged.connect(self._on_min_changed)
        self._slider.valueChanged.connect(self._on_slider_moved)
        self._exact_spin.valueChanged.connect(self._on_exact_changed)

        self._set_controls_enabled(False)

    # ── range changes ─────────────────────────────────────────────────────────

    def _on_max_changed(self, v: int):
        if v < self._min_spin.value():
            self._min_spin.blockSignals(True)
            self._min_spin.setValue(v)
            self._min_spin.blockSignals(False)
        self._slider.setMaximum(v)
        self._exact_spin.setMaximum(v)

    def _on_min_changed(self, v: int):
        if v > self._max_spin.value():
            self._max_spin.blockSignals(True)
            self._max_spin.setValue(v)
            self._max_spin.blockSignals(False)
        self._slider.setMinimum(v)
        self._exact_spin.setMinimum(v)

    # ── value sync ────────────────────────────────────────────────────────────

    def _on_slider_moved(self, v: int):
        self._val_label.setText(str(v))
        self._exact_spin.blockSignals(True)
        self._exact_spin.setValue(v)
        self._exact_spin.blockSignals(False)

    def _on_exact_changed(self, v: int):
        self._slider.blockSignals(True)
        self._slider.setValue(v)
        self._slider.blockSignals(False)
        self._val_label.setText(str(v))

    # ── drag / drop ───────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(CMD_MIME):
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        key = bytes(ev.mimeData().data(CMD_MIME)).decode()
        self._assign(key)
        if self._on_drop:
            limits = self._on_drop(key)
            if limits is not None:
                lo, hi = limits
                # Block signals to avoid cross-clamping during bulk update.
                for w in (self._min_spin, self._max_spin,
                          self._slider, self._exact_spin):
                    w.blockSignals(True)
                self._min_spin.setValue(lo)
                self._max_spin.setValue(hi)
                self._slider.setMinimum(lo)
                self._slider.setMaximum(hi)
                self._exact_spin.setMinimum(lo)
                self._exact_spin.setMaximum(hi)
                self._slider.setValue(0)
                self._exact_spin.setValue(0)
                self._val_label.setText("0")
                for w in (self._min_spin, self._max_spin,
                          self._slider, self._exact_spin):
                    w.blockSignals(False)
        ev.acceptProposedAction()

    def contextMenuEvent(self, ev):
        if self._field is None:
            return
        menu = QMenu(self)
        act  = QAction("Unassign", self)
        act.triggered.connect(self._unassign)
        menu.addAction(act)
        menu.exec_(ev.globalPos())

    # ── assignment ────────────────────────────────────────────────────────────

    def _assign(self, key: str):
        self._field = key
        label = next((l for k, l in COMMAND_FIELDS if k == key), key)
        self.setTitle(label)
        self._set_controls_enabled(True)

    def _unassign(self):
        self._field = None
        self.setTitle(SliderSlot._PLACEHOLDER)
        self._slider.setValue(0)
        self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled: bool):
        self._slider.setEnabled(enabled)
        self._max_spin.setEnabled(enabled)
        self._min_spin.setEnabled(enabled)
        self._exact_spin.setEnabled(enabled)

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def field(self) -> str | None:
        return self._field

    @property
    def value(self) -> int:
        return self._slider.value()

    def zero(self):
        self._slider.setValue(0)


# ─────────────────────────────────────────────────────────────────────────────
# Qt main window
# ─────────────────────────────────────────────────────────────────────────────

class DynoWindow(QMainWindow):

    def __init__(self, commander: DynoCommander):
        super().__init__()
        self._cmd          = commander
        self._main_enabled = False
        self._dut_enabled  = False
        self._hold_output1 = False

        self.setWindowTitle("Dyno Control")
        self._build_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._push_command)
        self._timer.start(5)   # 200 Hz

    def _build_ui(self):
        # ── Left: command field list ───────────────────────────────────────────
        self._field_list = CommandFieldList()

        # ── Centre: slider slots ───────────────────────────────────────────────
        self._slots = [SliderSlot(on_drop=self._cmd.get_limits) for _ in range(NUM_SLOTS)]
        slots_w   = QWidget()
        slots_lay = QHBoxLayout(slots_w)
        slots_lay.setSpacing(6)
        for slot in self._slots:
            slots_lay.addWidget(slot)

        # ── Right: hardcoded buttons ───────────────────────────────────────────
        btn_w   = QWidget()
        btn_lay = QVBoxLayout(btn_w)
        btn_lay.setSpacing(8)
        btn_w.setFixedWidth(130)

        self._main_enable_btn = QPushButton("Main Enable")
        self._main_enable_btn.setCheckable(True)
        self._main_enable_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44cc44; font-weight: bold; }")
        self._main_enable_btn.clicked.connect(self._toggle_main)

        main_zero_btn = QPushButton("Main Zero")
        main_zero_btn.clicked.connect(self._main_zero)

        self._main_mode_combo = QComboBox()
        for label, _ in DS402_MODES:
            self._main_mode_combo.addItem(label)
        self._main_mode_combo.setCurrentIndex(
            next(i for i, (_, v) in enumerate(DS402_MODES) if v == DS402_DEFAULT_MODE))

        self._dut_enable_btn = QPushButton("DUT Enable")
        self._dut_enable_btn.setCheckable(True)
        self._dut_enable_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44cc44; font-weight: bold; }")
        self._dut_enable_btn.clicked.connect(self._toggle_dut)

        dut_zero_btn = QPushButton("DUT Zero")
        dut_zero_btn.clicked.connect(self._dut_zero)

        self._dut_mode_combo = QComboBox()
        for label, _ in DS402_MODES:
            self._dut_mode_combo.addItem(label)
        self._dut_mode_combo.setCurrentIndex(
            next(i for i, (_, v) in enumerate(DS402_MODES) if v == DS402_DEFAULT_MODE))

        self._output1_btn = QPushButton("Hold Output 1")
        self._output1_btn.setCheckable(True)
        self._output1_btn.setStyleSheet(
            "QPushButton:checked { background-color: #44aaff; font-weight: bold; }")
        self._output1_btn.clicked.connect(self._toggle_output1)

        fault_btn = QPushButton("Fault Reset")
        fault_btn.setStyleSheet("background-color: #f0a000; font-weight: bold;")
        fault_btn.clicked.connect(self._cmd.pulse_fault_reset)

        btn_lay.addWidget(self._main_enable_btn)
        btn_lay.addWidget(main_zero_btn)
        btn_lay.addWidget(QLabel("Main Mode:"))
        btn_lay.addWidget(self._main_mode_combo)
        btn_lay.addSpacing(12)
        btn_lay.addWidget(self._dut_enable_btn)
        btn_lay.addWidget(dut_zero_btn)
        btn_lay.addWidget(QLabel("DUT Mode:"))
        btn_lay.addWidget(self._dut_mode_combo)
        btn_lay.addSpacing(12)
        btn_lay.addWidget(self._output1_btn)
        btn_lay.addWidget(fault_btn)
        btn_lay.addStretch()

        # ── Splitter ───────────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._field_list)
        splitter.addWidget(slots_w)
        splitter.addWidget(btn_w)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)

        # ── Status label ───────────────────────────────────────────────────────
        self._status_label = QLabel("bridge_ros2 starting…")
        self._status_label.setAlignment(Qt.AlignCenter)

        # ── Central widget ─────────────────────────────────────────────────────
        central = QWidget()
        vlay    = QVBoxLayout(central)
        vlay.addWidget(splitter, 1)
        vlay.addWidget(self._status_label)
        self.setCentralWidget(central)

    # ── button callbacks ──────────────────────────────────────────────────────

    def _toggle_main(self):
        self._main_enabled = self._main_enable_btn.isChecked()
        self._main_enable_btn.setText(
            "Main Enabled" if self._main_enabled else "Main Enable")

    def _toggle_dut(self):
        self._dut_enabled = self._dut_enable_btn.isChecked()
        self._dut_enable_btn.setText(
            "DUT Enabled" if self._dut_enabled else "DUT Enable")

    def _toggle_output1(self):
        self._hold_output1 = self._output1_btn.isChecked()
        self._output1_btn.setText(
            "Output 1 ON" if self._hold_output1 else "Hold Output 1")

    def _main_zero(self):
        for slot in self._slots:
            if slot.field in MAIN_ZERO_FIELDS:
                slot.zero()

    def _dut_zero(self):
        for slot in self._slots:
            if slot.field in DUT_ZERO_FIELDS:
                slot.zero()

    # ── publish ───────────────────────────────────────────────────────────────

    def _push_command(self):
        # Start with all command fields at zero; overwrite with assigned slots.
        numeric = {k: 0 for k in ALL_CMD_KEYS}
        for slot in self._slots:
            if slot.field is not None:
                numeric[slot.field] = slot.value
        self._cmd.set_command(
            numeric      = numeric,
            main_enable  = self._main_enabled,
            dut_enable   = self._dut_enabled,
            hold_output1 = self._hold_output1,
            main_mode    = DS402_MODES[self._main_mode_combo.currentIndex()][1],
            dut_mode     = DS402_MODES[self._dut_mode_combo.currentIndex()][1],
        )

    def set_status(self, text: str):
        self._status_label.setText(text)


# ─────────────────────────────────────────────────────────────────────────────
# Bridge subprocess management
# ─────────────────────────────────────────────────────────────────────────────

def launch_bridge(bridge_path: str, topology: str, fault_reset_s: float,
                  pub_hz: float, debug: bool = False):
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
                   help="Pass debug:=1 to bridge_ros2")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Launch bridge subprocess ──────────────────────────────────────────────
    bridge_proc = None
    if not args.no_bridge:
        if not os.path.isfile(args.bridge):
            print(f"ERROR: bridge binary not found at '{args.bridge}'\n"
                  f"       Build it first:  bash src/interface_bridges/ros2/build.sh",
                  file=sys.stderr)
            sys.exit(1)
        bridge_proc = launch_bridge(
            args.bridge, args.topology, args.fault_reset_s, args.pub_hz, args.debug)

    # ── ROS2 init ─────────────────────────────────────────────────────────────
    rclpy.init()
    commander = DynoCommander(pub_hz=args.pub_hz)

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(commander,), daemon=True)
    ros_thread.start()

    # ── Qt GUI ────────────────────────────────────────────────────────────────
    app    = QApplication(sys.argv)
    window = DynoWindow(commander)
    window.resize(700, 420)

    if bridge_proc is not None:
        window.set_status(f"bridge_ros2 PID {bridge_proc.pid}")
    else:
        window.set_status("--no-bridge: connecting to existing bridge")

    window.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    exit_code = app.exec_()

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print("[dyno_gui] Window closed — shutting down.")
    commander.set_command(
        numeric={k: 0 for k in ALL_CMD_KEYS},
        main_enable=False, dut_enable=False, hold_output1=False)
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
