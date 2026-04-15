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
import importlib.util
import inspect
import json
import os
import signal
import subprocess
import sys
import threading
import time
import traceback

# ── Qt ────────────────────────────────────────────────────────────────────────
try:
    from PyQt5.QtCore    import Qt, QTimer, QMimeData, QByteArray
    from PyQt5.QtGui     import QFont
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget,
        QVBoxLayout, QHBoxLayout, QSplitter,
        QSlider, QPushButton, QLabel, QGroupBox,
        QSpinBox, QDoubleSpinBox, QListWidget, QListWidgetItem,
        QMenu, QAction, QComboBox, QTextEdit, QFormLayout,
        QScrollArea, QSizePolicy,
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

DEFAULT_BRIDGE      = "build/dyno_ros2_bridge/bridge_ros2"
DEFAULT_TOPOLOGY    = "config/topology.dyno2.template6.json"
DEFAULT_PUB_HZ      = 200.0
TEST_SCRIPTS_DIR    = "src/dyno_test_scripts"  # scanned for *.py test scripts
DEFAULT_FAULT_S  = 2.0

CMD_MIME  = "application/x-dyno-command-field"
NUM_SLOTS = 9   # number of slider slots shown

# All drag-assignable command fields: (json_key, display_label)
COMMAND_FIELDS = [
    ("main_velocity",   "Main Velocity"),
    ("main_position",   "Main Position"),
    ("main_torque",     "Main Torque"),
    ("main_current",    "Main Current"),
    ("dut_velocity",    "DUT Velocity"),
    ("dut_position",    "DUT Position"),
    ("dut_torque",      "DUT Torque"),
    ("dut_current",     "DUT Current"),
    # Control gains
    ("main_torque_kp",  "Main Torque Kp"),
    ("main_torque_max", "Main Torque Max"),
    ("main_torque_min", "Main Torque Min"),
    ("main_vel_kp",     "Main Vel Kp"),
    ("main_vel_ki",     "Main Vel Ki"),
    ("main_vel_kd",     "Main Vel Kd"),
    ("main_pos_kp",     "Main Pos Kp"),
    ("main_pos_ki",     "Main Pos Ki"),
    ("main_pos_kd",     "Main Pos Kd"),
    ("dut_torque_kp",   "DUT Torque Kp"),
    ("dut_torque_max",  "DUT Torque Max"),
    ("dut_torque_min",  "DUT Torque Min"),
    ("dut_vel_kp",      "DUT Vel Kp"),
    ("dut_vel_ki",      "DUT Vel Ki"),
    ("dut_vel_kd",      "DUT Vel Kd"),
    ("dut_pos_kp",      "DUT Pos Kp"),
    ("dut_pos_ki",      "DUT Pos Ki"),
    ("dut_pos_kd",      "DUT Pos Kd"),
]

# Fields that use a float spinbox instead of the integer slider
GAIN_FIELDS = {
    "main_torque_kp", "main_torque_max", "main_torque_min",
    "main_vel_kp",    "main_vel_ki",     "main_vel_kd",
    "main_pos_kp",    "main_pos_ki",     "main_pos_kd",
    "dut_torque_kp",  "dut_torque_max",  "dut_torque_min",
    "dut_vel_kp",     "dut_vel_ki",      "dut_vel_kd",
    "dut_pos_kp",     "dut_pos_ki",      "dut_pos_kd",
}

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

# Non-gain setpoint fields zeroed during script preamble/epilogue
_ZERO_CMD = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}


def _interruptible_sleep(secs: float, stop_event, granularity: float = 0.01) -> bool:
    """Sleep for ``secs`` seconds, waking every ``granularity`` seconds to check
    stop_event.  Returns True if interrupted (stop_event set), False if the full
    duration elapsed."""
    deadline = time.monotonic() + secs
    while time.monotonic() < deadline:
        if stop_event.is_set():
            return True
        time.sleep(min(granularity, deadline - time.monotonic()))
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Test script discovery and execution
# ─────────────────────────────────────────────────────────────────────────────

def _discover_scripts(scripts_dir: str) -> list[str]:
    """Return sorted list of *.py paths in scripts_dir, newest first."""
    if not os.path.isdir(scripts_dir):
        return []
    paths = [
        os.path.join(scripts_dir, f)
        for f in os.listdir(scripts_dir)
        if f.endswith(".py") and not f.startswith("_")
    ]
    return sorted(paths, key=lambda p: os.path.getmtime(p), reverse=True)


def _load_script_module(path: str):
    """
    Dynamically load a test script module.
    Scripts must define:
        PARAMS: dict[str, float | int | str]  — editable parameters with defaults
        run(params: dict, commander) -> None  — called in a background thread
    """
    spec = importlib.util.spec_from_file_location("_dyno_test_script", path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ScriptRunner:
    """Manages loading and running one test script at a time in a background thread."""

    def __init__(self):
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock       = threading.Lock()

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def run(self, path: str, params: dict, commander, on_done, on_log,
            main_mode: int = 9, dut_mode: int = 9,
            pre_settle_s: float = 0.1, post_settle_s: float = 0.1):
        """
        Start script in a background thread with preamble and epilogue.

        Preamble (runs before mod.run()):
          zero+disable → sleep(pre_settle_s) → enable → sleep(pre_settle_s)
          → set mode → sleep(pre_settle_s) → mod.run()

        Epilogue (runs after mod.run() regardless of success/abort):
          zero+keep_enabled → sleep(post_settle_s) → disable+mode=0
          → sleep(post_settle_s) → on_done()

        on_done(success: bool, msg: str) — called once, after epilogue completes
        on_log(line: str)                — called for preamble/epilogue status lines
        """
        if self.is_running:
            return
        self._stop_event.clear()
        mod = _load_script_module(path)

        def _target():
            zero = _ZERO_CMD.copy()

            # ── PREAMBLE ─────────────────────────────────────────────────────
            # Step 0: caller already sent zero+disable; wait for it to reach drive.
            on_log("[preamble] waiting for zeros to propagate…")
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                on_done(False, "Aborted during pre-zero settle.")
                return

            # Step 1: enable both drives, mode still 0
            on_log("[preamble] enabling drives…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=0, dut_mode=0)
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                commander.set_command(
                    numeric=zero, main_enable=False, dut_enable=False,
                    main_mode=0, dut_mode=0)
                on_done(False, "Aborted during pre-enable settle.")
                return

            # Step 2: set target mode
            on_log(f"[preamble] setting mode (main={main_mode}, dut={dut_mode})…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=main_mode, dut_mode=dut_mode)
            if _interruptible_sleep(pre_settle_s, self._stop_event):
                commander.set_command(
                    numeric=zero, main_enable=False, dut_enable=False,
                    main_mode=0, dut_mode=0)
                on_done(False, "Aborted during pre-mode settle.")
                return

            # ── SCRIPT ───────────────────────────────────────────────────────
            on_log("[preamble] starting script…")
            try:
                mod.run(params, commander, self._stop_event)
                success, msg = True, "Completed successfully."
            except Exception:
                success, msg = False, traceback.format_exc()

            # ── EPILOGUE ─────────────────────────────────────────────────────
            # Zero setpoints; keep drives briefly enabled so motion can settle.
            on_log("[epilogue] zeroing commands…")
            commander.set_command(
                numeric=zero, main_enable=True, dut_enable=True,
                main_mode=0, dut_mode=0)
            time.sleep(post_settle_s)   # unconditional — always clean up

            # Disable drives and clear mode.
            on_log("[epilogue] disabling drives…")
            commander.set_command(
                numeric=zero, main_enable=False, dut_enable=False,
                main_mode=0, dut_mode=0)
            time.sleep(post_settle_s)

            on_done(success, msg)

        self._thread = threading.Thread(target=_target, daemon=True)
        self._thread.start()

    def abort(self):
        """Signal the running script to stop (sets stop_event)."""
        self._stop_event.set()


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 commander node (runs on a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DynoCommander(Node):
    """Publishes /dyno/command at a fixed rate from the current GUI state."""

    def __init__(self, pub_hz: float):
        super().__init__("dyno_gui_commander")
        self._pub = self.create_publisher(StringMsg, "/dyno/command", 10)

        self._lock         = threading.Lock()
        # Gain fields are intentionally excluded — they are only added to the
        # payload when a slider is explicitly assigned to them, so the bridge
        # keeps its SDO-seeded values by default.
        self._numeric      = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}
        self._main_enable  = False
        self._dut_enable   = False
        self._fault_reset  = False
        self._hold_output1 = False
        self._main_mode    = DS402_DEFAULT_MODE
        self._dut_mode     = DS402_DEFAULT_MODE

        # Drive limits — updated from status topics, used to auto-set slider ranges.
        _empty_limits = {
            "max_velocity_abs": 0.0, "min_position": 0.0, "max_position": 0.0,
            "torque_kp": 0.0, "torque_max": 0.0, "torque_min": 0.0,
            "vel_kp": 0.0, "vel_ki": 0.0, "vel_kd": 0.0,
            "pos_kp": 0.0, "pos_ki": 0.0, "pos_kd": 0.0,
        }
        self._limits_lock     = threading.Lock()
        self._main_limits     = dict(_empty_limits)
        self._dut_limits      = dict(_empty_limits)
        self._limits_callback = None   # callable(drive: str), set by DynoWindow

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
        # Prefer natural-unit fields (_rad_s / _rad); fall back to raw for compat
        limits = {
            "max_velocity_abs": float(data.get("max_velocity_abs_rad_s",
                                      data.get("max_velocity_abs", 0.0))),
            "min_position":     float(data.get("min_position_rad",
                                      data.get("min_position", 0))),
            "max_position":     float(data.get("max_position_rad",
                                      data.get("max_position", 0))),
            "torque_kp":  float(data.get("torque_kp",  0.0)),
            "torque_max": float(data.get("torque_max", 0.0)),
            "torque_min": float(data.get("torque_min", 0.0)),
            "vel_kp":     float(data.get("vel_kp",     0.0)),
            "vel_ki":     float(data.get("vel_ki",     0.0)),
            "vel_kd":     float(data.get("vel_kd",     0.0)),
            "pos_kp":     float(data.get("pos_kp",     0.0)),
            "pos_ki":     float(data.get("pos_ki",     0.0)),
            "pos_kd":     float(data.get("pos_kd",     0.0)),
        }
        with self._limits_lock:
            if drive == "main":
                self._main_limits = limits
            else:
                self._dut_limits = limits
        if self._limits_callback:
            self._limits_callback(drive)

    def set_limits_callback(self, cb) -> None:
        self._limits_callback = cb

    def get_limits(self, field_key: str):
        """Return (min, max, default) for a command field, or None if unavailable.
        Gain fields return floats; velocity/position return floats in natural units."""
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
            max_vel = limits["max_velocity_abs"]
            if max_vel > 0:
                return (-max_vel, max_vel, 0.0)
        elif field_type == "position":
            lo = limits["min_position"]
            hi = limits["max_position"]
            if lo == 0.0 and hi == 0.0:
                return None
            _NO_LIMIT = 1000.0   # rad — ~159 revolutions, usable default
            if lo <= -1e9:
                lo = -_NO_LIMIT
            if hi >= 1e9:
                hi = _NO_LIMIT
            return (lo, hi, 0.0)
        elif field_type == "torque_min":
            current = limits.get(field_type, 0.0)
            return (-20.0, 0.0, current)   # negative clamp — must allow negative
        elif field_type in ("torque_kp", "torque_max",
                            "vel_kp", "vel_ki", "vel_kd",
                            "pos_kp", "pos_ki", "pos_kd"):
            current = limits.get(field_type, 0.0)
            return (0.0, 20.0, current)   # float range; default = value from drive
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

    def __init__(self, on_drop=None, is_field_allowed=None,
                 on_assigned=None, on_unassigned=None, parent=None):
        super().__init__(SliderSlot._PLACEHOLDER, parent)
        self._field: str | None = None
        self._float_mode        = False
        self._on_drop          = on_drop           # (key) -> (min,max,default) | None
        self._is_field_allowed = is_field_allowed  # (key) -> bool
        self._on_assigned      = on_assigned       # (key) -> None
        self._on_unassigned    = on_unassigned     # (key) -> None
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

        # Exact entry spinbox (integer mode)
        self._exact_spin = QSpinBox()
        self._exact_spin.setRange(-1000, 1000)
        self._exact_spin.setValue(0)

        # Float spinbox (gain mode — replaces slider+exact_spin)
        self._float_spin = QDoubleSpinBox()
        self._float_spin.setRange(0.0, 20.0)
        self._float_spin.setSingleStep(0.001)
        self._float_spin.setDecimals(3)
        self._float_spin.setValue(0.0)
        self._float_spin.hide()

        # Value display
        self._val_label = QLabel("0")
        self._val_label.setAlignment(Qt.AlignCenter)

        # Clear button
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._unassign)

        col = QVBoxLayout(self)
        col.addWidget(self._max_spin)
        col.addWidget(self._slider, 1, Qt.AlignHCenter)
        col.addWidget(self._float_spin)
        col.addWidget(self._min_spin)
        col.addWidget(QLabel("Exact:"))
        col.addWidget(self._exact_spin)
        col.addWidget(self._val_label)
        col.addWidget(self._clear_btn)

        # Signal wiring
        self._max_spin.valueChanged.connect(self._on_max_changed)
        self._min_spin.valueChanged.connect(self._on_min_changed)
        self._slider.valueChanged.connect(self._on_slider_moved)
        self._exact_spin.valueChanged.connect(self._on_exact_changed)
        self._float_spin.valueChanged.connect(self._on_float_changed)

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

    def _on_float_changed(self, v: float):
        self._val_label.setText(f"{v:.3f}")

    # ── float mode ────────────────────────────────────────────────────────────

    def set_float_mode(self, lo: float, hi: float, default: float):
        """Switch to float spinbox mode (for gain fields)."""
        self._float_mode = True
        self._slider.hide()
        self._max_spin.hide()
        self._min_spin.hide()
        self._exact_spin.hide()
        self._float_spin.blockSignals(True)
        self._float_spin.setRange(lo, hi)
        self._float_spin.setValue(default)
        self._float_spin.blockSignals(False)
        self._val_label.setText(f"{default:.3f}")
        self._float_spin.show()

    def clear_float_mode(self):
        """Switch back to integer slider mode."""
        self._float_mode = False
        self._float_spin.hide()
        self._slider.show()
        self._max_spin.show()
        self._min_spin.show()
        self._exact_spin.show()

    # ── drag / drop ───────────────────────────────────────────────────────────

    def dragEnterEvent(self, ev):
        if ev.mimeData().hasFormat(CMD_MIME):
            key = bytes(ev.mimeData().data(CMD_MIME)).decode()
            # Allow if: this slot already has the field, OR it's not taken elsewhere.
            if key != self._field and self._is_field_allowed and \
                    not self._is_field_allowed(key):
                ev.ignore()
                return
            ev.acceptProposedAction()
        else:
            ev.ignore()

    def dragMoveEvent(self, ev):
        ev.acceptProposedAction()

    def dropEvent(self, ev):
        key = bytes(ev.mimeData().data(CMD_MIME)).decode()
        self._assign(key)
        if self._on_drop:
            result = self._on_drop(key)
            if result is not None:
                lo, hi, default = result
                if key in GAIN_FIELDS:
                    self.set_float_mode(lo, hi, default)
                else:
                    self.clear_float_mode()
                    # Block signals to avoid cross-clamping during bulk update.
                    for w in (self._min_spin, self._max_spin,
                              self._slider, self._exact_spin):
                        w.blockSignals(True)
                    lo_i, hi_i, def_i = int(lo), int(hi), int(default)
                    self._min_spin.setValue(lo_i)
                    self._max_spin.setValue(hi_i)
                    self._slider.setMinimum(lo_i)
                    self._slider.setMaximum(hi_i)
                    self._exact_spin.setMinimum(lo_i)
                    self._exact_spin.setMaximum(hi_i)
                    self._slider.setValue(def_i)
                    self._exact_spin.setValue(def_i)
                    self._val_label.setText(str(def_i))
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
        if self._field == key:
            return  # No change — same field re-dropped
        if self._field is not None and self._on_unassigned:
            self._on_unassigned(self._field)   # Release previous field
        self._field = key
        label = next((l for k, l in COMMAND_FIELDS if k == key), key)
        self.setTitle(label)
        self._set_controls_enabled(True)
        if self._on_assigned:
            self._on_assigned(key)

    def _unassign(self):
        old = self._field
        self._field = None
        self.setTitle(SliderSlot._PLACEHOLDER)
        self.clear_float_mode()
        self._slider.setValue(0)
        self._set_controls_enabled(False)
        if old is not None and self._on_unassigned:
            self._on_unassigned(old)

    def _set_controls_enabled(self, enabled: bool):
        self._slider.setEnabled(enabled)
        self._max_spin.setEnabled(enabled)
        self._min_spin.setEnabled(enabled)
        self._exact_spin.setEnabled(enabled)
        self._float_spin.setEnabled(enabled)
        self._clear_btn.setEnabled(enabled)

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def field(self) -> str | None:
        return self._field

    @property
    def value(self):
        """Return float if in gain mode, int otherwise."""
        if self._float_mode:
            return self._float_spin.value()
        return self._slider.value()

    def zero(self):
        if self._float_mode:
            self._float_spin.setValue(0.0)
        else:
            self._slider.setValue(0)

    def apply_limits(self, lo: float, hi: float):
        """Update slider range without resetting the current value (integer mode only)."""
        if self._float_mode:
            self._float_spin.blockSignals(True)
            self._float_spin.setRange(lo, hi)
            self._float_spin.blockSignals(False)
            return
        lo_i, hi_i = int(lo), int(hi)
        for w in (self._min_spin, self._max_spin, self._slider, self._exact_spin):
            w.blockSignals(True)
        self._min_spin.setValue(lo_i)
        self._max_spin.setValue(hi_i)
        self._slider.setMinimum(lo_i)
        self._slider.setMaximum(hi_i)
        self._exact_spin.setMinimum(lo_i)
        self._exact_spin.setMaximum(hi_i)
        for w in (self._min_spin, self._max_spin, self._slider, self._exact_spin):
            w.blockSignals(False)


# ─────────────────────────────────────────────────────────────────────────────
# Scripting panel
# ─────────────────────────────────────────────────────────────────────────────

class ScriptingPanel(QGroupBox):
    """
    Right-side panel for selecting and running Python test scripts.

    Script protocol — each script file must define:
        PARAMS = {"speed": 100, "duration_s": 5.0, ...}  # param name → default
        def run(params: dict, commander: DynoCommander, stop_event: threading.Event):
            ...  # check stop_event.is_set() regularly to support abort
    """

    def __init__(self, commander, scripts_dir: str,
                 on_script_active=None, get_modes=None, parent=None):
        super().__init__("Test Scripts", parent)
        self._commander        = commander
        self._scripts_dir      = scripts_dir
        self._runner           = ScriptRunner()
        self._param_widgets: dict[str, QDoubleSpinBox | QSpinBox] = {}
        self._current_mod      = None
        self._on_script_active = on_script_active  # callable(bool) | None
        self._get_modes        = get_modes          # callable() → (main_mode, dut_mode) | None

        # ── Script selector ───────────────────────────────────────────────────
        self._script_combo = QComboBox()
        self._script_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._script_combo.currentIndexChanged.connect(self._on_script_selected)

        refresh_btn = QPushButton("↻")
        refresh_btn.setFixedWidth(28)
        refresh_btn.setToolTip("Rescan scripts folder")
        refresh_btn.clicked.connect(self._scan_scripts)

        selector_row = QHBoxLayout()
        selector_row.addWidget(self._script_combo, 1)
        selector_row.addWidget(refresh_btn)

        # ── Parameters area ───────────────────────────────────────────────────
        self._params_form   = QFormLayout()
        self._params_form.setSpacing(4)
        params_widget = QWidget()
        params_widget.setLayout(self._params_form)

        self._params_scroll = QScrollArea()
        self._params_scroll.setWidget(params_widget)
        self._params_scroll.setWidgetResizable(True)
        self._params_scroll.setMinimumHeight(80)
        self._params_scroll.setMaximumHeight(200)

        # ── Timing ────────────────────────────────────────────────────────────
        self._pre_settle_spin = QDoubleSpinBox()
        self._pre_settle_spin.setRange(0, 5000)
        self._pre_settle_spin.setSingleStep(10)
        self._pre_settle_spin.setSuffix(" ms")
        self._pre_settle_spin.setValue(100)
        self._pre_settle_spin.setToolTip(
            "Wait applied after each preamble step:\n"
            "  zero→enable, enable→mode, mode→script start")

        self._post_settle_spin = QDoubleSpinBox()
        self._post_settle_spin.setRange(0, 5000)
        self._post_settle_spin.setSingleStep(10)
        self._post_settle_spin.setSuffix(" ms")
        self._post_settle_spin.setValue(100)
        self._post_settle_spin.setToolTip(
            "Wait applied after each epilogue step:\n"
            "  zero setpoints, then disable drives")

        timing_row = QHBoxLayout()
        timing_row.addWidget(QLabel("Pre:"))
        timing_row.addWidget(self._pre_settle_spin)
        timing_row.addSpacing(6)
        timing_row.addWidget(QLabel("Post:"))
        timing_row.addWidget(self._post_settle_spin)

        # ── Buttons ───────────────────────────────────────────────────────────
        self._run_btn = QPushButton("Run Test")
        self._run_btn.setStyleSheet("background-color: #44cc44; font-weight: bold;")
        self._run_btn.clicked.connect(self._run_script)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setStyleSheet("background-color: #cc4444; font-weight: bold;")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._abort_script)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self._run_btn)
        btn_row.addWidget(self._abort_btn)

        # ── Output log ────────────────────────────────────────────────────────
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumHeight(80)
        self._log.setFont(QFont("Monospace", 8))
        self._log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ── Layout ────────────────────────────────────────────────────────────
        lay = QVBoxLayout(self)
        lay.addLayout(selector_row)
        lay.addWidget(QLabel("Parameters:"))
        lay.addWidget(self._params_scroll)
        lay.addLayout(timing_row)
        lay.addLayout(btn_row)
        lay.addWidget(QLabel("Output:"))
        lay.addWidget(self._log, 1)

        self._scan_scripts()

    # ── Script discovery ──────────────────────────────────────────────────────

    def _scan_scripts(self):
        paths = _discover_scripts(self._scripts_dir)
        prev  = self._script_combo.currentData()
        self._script_combo.blockSignals(True)
        self._script_combo.clear()
        if not paths:
            self._script_combo.addItem(f"(no scripts in {self._scripts_dir})", None)
        for p in paths:
            self._script_combo.addItem(os.path.basename(p), p)
        # Restore selection if still present
        idx = next((i for i in range(self._script_combo.count())
                    if self._script_combo.itemData(i) == prev), 0)
        self._script_combo.setCurrentIndex(idx)
        self._script_combo.blockSignals(False)
        self._on_script_selected(self._script_combo.currentIndex())

    def _on_script_selected(self, idx: int):
        path = self._script_combo.itemData(idx)
        self._current_mod = None
        self._clear_params()
        if not path:
            return
        try:
            mod = _load_script_module(path)
            self._current_mod = mod
            params = getattr(mod, "PARAMS", {})
            self._build_params(params)
        except Exception as e:
            self._log_line(f"[load error] {e}")

    # ── Parameter form ────────────────────────────────────────────────────────

    def _clear_params(self):
        while self._params_form.rowCount():
            self._params_form.removeRow(0)
        self._param_widgets.clear()

    def _build_params(self, params: dict):
        self._clear_params()
        for name, default in params.items():
            if isinstance(default, float):
                w = QDoubleSpinBox()
                w.setDecimals(4)
                w.setRange(-1e9, 1e9)
                w.setSingleStep(0.1)
                w.setValue(default)
            else:
                w = QSpinBox()
                w.setRange(-2**30, 2**30 - 1)
                w.setValue(int(default))
            self._param_widgets[name] = w
            self._params_form.addRow(name + ":", w)

    def _collect_params(self) -> dict:
        return {name: w.value() for name, w in self._param_widgets.items()}

    # ── Run / abort ───────────────────────────────────────────────────────────

    def _run_script(self):
        if self._current_mod is None or self._runner.is_running:
            return

        # Zero drives and set mode_of_operation to 0 before handing off to script
        self._commander.set_command(
            numeric     = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS},
            main_enable = False,
            dut_enable  = False,
            main_mode   = 0,
            dut_mode    = 0,
        )
        # Pause GUI push — script now has exclusive control of set_command()
        if self._on_script_active:
            self._on_script_active(True)

        params = self._collect_params()
        self._log.clear()
        self._log_line(f"[run] {self._script_combo.currentText()}")
        self._log_line(f"[params] {params}")

        self._run_btn.setEnabled(False)
        self._abort_btn.setEnabled(True)

        main_mode, dut_mode = self._get_modes() if self._get_modes else (9, 9)
        pre_s  = self._pre_settle_spin.value()  / 1000.0
        post_s = self._post_settle_spin.value() / 1000.0

        def _on_done(success: bool, msg: str):
            QTimer.singleShot(0, lambda: self._on_script_done(success, msg))

        try:
            self._runner.run(
                path          = self._script_combo.currentData(),
                params        = params,
                commander     = self._commander,
                on_done       = _on_done,
                on_log        = self._log_line,
                main_mode     = main_mode,
                dut_mode      = dut_mode,
                pre_settle_s  = pre_s,
                post_settle_s = post_s,
            )
        except Exception as exc:
            if self._on_script_active:
                self._on_script_active(False)
            self._run_btn.setEnabled(True)
            self._abort_btn.setEnabled(False)
            self._log_line(f"[error] Could not start script: {exc}")
            return

        # Thread started — poll for completion so UI is restored reliably
        # regardless of whether QTimer.singleShot from the background thread fires.
        QTimer.singleShot(100, self._poll_thread_done)

    def _abort_script(self):
        self._runner.abort()
        self._log_line("[abort] stop requested")
        # Immediately restore GUI control (Main Enable, sliders) without waiting
        # for the script thread to finish sleeping.
        # _poll_thread_done (started by _run_script) will re-enable Run Test
        # once the thread actually exits.
        if self._on_script_active:
            self._on_script_active(False)
        self._abort_btn.setEnabled(False)

    def _poll_thread_done(self) -> None:
        """Poll every 100 ms until the background script thread has exited.
        Authoritative UI restorer for both natural completion and abort."""
        if self._runner.is_running:
            QTimer.singleShot(100, self._poll_thread_done)
        else:
            # Idempotent — _on_script_active(False) may already be False (abort path).
            if self._on_script_active:
                self._on_script_active(False)
            self._run_btn.setEnabled(True)
            self._abort_btn.setEnabled(False)

    def _on_script_done(self, success: bool, msg: str):
        """Log-only — UI state is managed by _poll_thread_done."""
        status = "OK" if success else "ERROR"
        self._log_line(f"[{status}] {msg.strip()}")

    # ── Log helper ────────────────────────────────────────────────────────────

    def _log_line(self, text: str):
        # Safe to call from any thread via QTimer.singleShot
        def _append():
            self._log.append(text)
            self._log.verticalScrollBar().setValue(
                self._log.verticalScrollBar().maximum())
        QTimer.singleShot(0, _append)


# ─────────────────────────────────────────────────────────────────────────────
# Qt main window
# ─────────────────────────────────────────────────────────────────────────────

class DynoWindow(QMainWindow):

    def __init__(self, commander: DynoCommander, scripts_dir: str = TEST_SCRIPTS_DIR):
        super().__init__()
        self._cmd            = commander
        self._scripts_dir    = scripts_dir
        self._main_enabled   = False
        self._dut_enabled    = False
        self._hold_output1   = False
        self._script_running = False

        self.setWindowTitle("Dyno Control")
        self._build_ui()
        commander.set_limits_callback(self._on_limits_updated)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._push_command)
        self._timer.start(5)   # 200 Hz

    def _build_ui(self):
        # ── Left: command field list ───────────────────────────────────────────
        self._field_list = CommandFieldList()

        # ── Centre: slider slots ───────────────────────────────────────────────
        self._assigned_fields: set[str] = set()
        self._slots = [
            SliderSlot(
                on_drop          = self._cmd.get_limits,
                is_field_allowed = lambda key: key not in self._assigned_fields,
                on_assigned      = lambda key: self._assigned_fields.add(key),
                on_unassigned    = lambda key: self._assigned_fields.discard(key),
            )
            for _ in range(NUM_SLOTS)
        ]
        slots_w   = QWidget()
        slots_lay = QHBoxLayout(slots_w)
        slots_lay.setSpacing(6)
        for slot in self._slots:
            slots_lay.addWidget(slot)

        # ── Right panel: buttons + scripting side by side ─────────────────────
        right_w   = QWidget()
        right_lay = QHBoxLayout(right_w)
        right_lay.setSpacing(6)
        right_lay.setContentsMargins(0, 0, 0, 0)

        # ── Buttons column ────────────────────────────────────────────────────
        btn_w   = QWidget()
        btn_lay = QVBoxLayout(btn_w)
        btn_lay.setSpacing(8)
        btn_w.setFixedWidth(140)

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
        fault_btn.clicked.connect(self._fault_reset_pressed)

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

        # ── Scripting panel ───────────────────────────────────────────────────
        self._script_panel = ScriptingPanel(
            self._cmd, self._scripts_dir,
            on_script_active = self._on_script_active,
            get_modes        = lambda: (
                DS402_MODES[self._main_mode_combo.currentIndex()][1],
                DS402_MODES[self._dut_mode_combo.currentIndex()][1],
            ),
        )
        self._script_panel.setMinimumWidth(280)

        right_lay.addWidget(btn_w)
        right_lay.addWidget(self._script_panel, 1)

        # ── Splitter ───────────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._field_list)
        splitter.addWidget(slots_w)
        splitter.addWidget(right_w)
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

    # ── script handoff ────────────────────────────────────────────────────────

    def _on_script_active(self, active: bool) -> None:
        """Called by ScriptingPanel when a script starts (True) or finishes (False)."""
        self._script_running = active
        # Always reset mode combos to "No Mode (0)":
        #   • on start  — ensures _get_modes() returns (0,0) so the preamble doesn't
        #                 briefly re-apply whatever manual mode the GUI combo showed.
        #   • on finish — reflects the epilogue's mode=0 disable state.
        _none_idx = next(i for i, (_, v) in enumerate(DS402_MODES) if v == 0)
        self._main_mode_combo.setCurrentIndex(_none_idx)
        self._dut_mode_combo.setCurrentIndex(_none_idx)
        if not active:
            # Uncheck both enable buttons so the drive is not automatically
            # re-enabled when GUI push resumes after the script.
            self._main_enabled = False
            self._dut_enabled  = False
            self._main_enable_btn.setChecked(False)
            self._main_enable_btn.setText("Main Enable")
            self._dut_enable_btn.setChecked(False)
            self._dut_enable_btn.setText("DUT Enable")
            # Push the now-disabled state once so the drive receives it immediately.
            self._push_command()

    def _fault_reset_pressed(self) -> None:
        """Fault Reset button handler — blocked while a script is running."""
        if self._script_running:
            return   # must abort script before using fault reset
        self._cmd.pulse_fault_reset()

    # ── publish ───────────────────────────────────────────────────────────────

    def _push_command(self):
        if self._script_running:
            return
        # Setpoint fields default to 0; gain fields are omitted unless a slider
        # is actively assigned to them (so the bridge keeps its SDO-seeded values).
        numeric = {k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS}
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

    def _on_limits_updated(self, drive: str) -> None:
        """Called from the ROS spin thread — schedule GUI update on the Qt thread."""
        QTimer.singleShot(0, lambda: self._refresh_slot_limits(drive))

    def _refresh_slot_limits(self, drive: str) -> None:
        prefix = "main_" if drive == "main" else "dut_"
        for slot in self._slots:
            if slot.field and slot.field.startswith(prefix):
                result = self._cmd.get_limits(slot.field)
                if result is not None:
                    lo, hi, _ = result
                    slot.apply_limits(lo, hi)

    def set_status(self, text: str):
        self._status_label.setText(text)

    def closeEvent(self, event):
        """Abort any running script before closing so the cleanup zero command wins."""
        if self._script_running:
            self._script_panel._abort_script()
            # Give the script thread a moment to see stop_event and stop sending
            # enable=True before main() sends its zero command.
            time.sleep(0.15)
        event.accept()


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
    window.resize(1400, 680)

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
        numeric={k: 0 for k in ALL_CMD_KEYS if k not in GAIN_FIELDS},
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
