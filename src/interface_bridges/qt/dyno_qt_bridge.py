#!/usr/bin/env python3
"""
Dyno PySide6 / Qt Bridge

Live dashboard for the dyno testbench.  Receives telemetry from bridge_udp
over UDP and sends commands back.

Install dependencies:
    pip install PySide6 pyqtgraph

Usage:
    python3 dyno_qt_bridge.py [--telem-port 7600] [--cmd-port 7601]
"""

import argparse
import json
import socket
import threading
import time
from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QPushButton, QSpinBox,
    QVBoxLayout, QWidget,
)

TELEM_PORT   = 7600
COMMAND_PORT = 7601
BUFFER_SIZE  = 4096
HISTORY_LEN  = 500   # number of samples kept in plot buffers


# ── UDP receiver (runs on background thread, emits signals) ───────────────────

class TelemetryReceiver(QObject):
    received = Signal(dict)

    def __init__(self, port: int):
        super().__init__()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('127.0.0.1', port))
        self._sock.settimeout(1.0)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(BUFFER_SIZE)
                t = json.loads(data.decode())
                self.received.emit(t)
            except socket.timeout:
                continue
            except Exception:
                continue

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._sock.close()


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):

    def __init__(self, telem_port: int, cmd_port: int, bridge_host: str):
        super().__init__()
        self.setWindowTitle('Dyno Testbench Dashboard')
        self.resize(1100, 700)

        self._cmd_sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_addr   = (bridge_host, cmd_port)
        self._cmd_state  = {
            'main_speed':   0,
            'dut_speed':    0,
            'main_enable':  False,
            'dut_enable':   False,
            'fault_reset':  False,
            'hold_output1': False,
        }

        # Plot buffers
        self._t          = deque(maxlen=HISTORY_LEN)
        self._main_vel   = deque(maxlen=HISTORY_LEN)
        self._dut_vel    = deque(maxlen=HISTORY_LEN)
        self._ch1_torque = deque(maxlen=HISTORY_LEN)
        self._ch2_torque = deque(maxlen=HISTORY_LEN)
        self._t0         = time.monotonic()

        self._build_ui()

        # Telemetry receiver
        self._receiver = TelemetryReceiver(telem_port)
        self._receiver.received.connect(self._on_telemetry)

        # Command send timer (20 Hz) — keeps bridge alive even without UI events
        self._cmd_timer = QTimer(self)
        self._cmd_timer.timeout.connect(self._send_command)
        self._cmd_timer.start(50)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # ── Left panel: controls ──────────────────────────────────────────────
        left = QVBoxLayout()
        root.addLayout(left, stretch=1)

        # Main drive controls
        main_grp = QGroupBox('Main Drive')
        main_layout = QVBoxLayout(main_grp)

        self._main_speed = QSpinBox()
        self._main_speed.setRange(-10000, 10000)
        self._main_speed.setSingleStep(100)
        self._main_speed.setValue(0)
        self._main_speed.valueChanged.connect(
            lambda v: self._set_cmd('main_speed', v))
        main_layout.addWidget(QLabel('Speed (rad/s × scale):'))
        main_layout.addWidget(self._main_speed)

        self._main_enable = QCheckBox('Enable')
        self._main_enable.stateChanged.connect(
            lambda s: self._set_cmd('main_enable', s == Qt.CheckState.Checked.value))
        main_layout.addWidget(self._main_enable)

        self._main_status = QLabel('state: —')
        main_layout.addWidget(self._main_status)
        left.addWidget(main_grp)

        # DUT controls
        dut_grp = QGroupBox('DUT Drive')
        dut_layout = QVBoxLayout(dut_grp)

        self._dut_speed = QSpinBox()
        self._dut_speed.setRange(-10000, 10000)
        self._dut_speed.setSingleStep(100)
        self._dut_speed.setValue(0)
        self._dut_speed.valueChanged.connect(
            lambda v: self._set_cmd('dut_speed', v))
        dut_layout.addWidget(QLabel('Speed (rad/s × scale):'))
        dut_layout.addWidget(self._dut_speed)

        self._dut_enable = QCheckBox('Enable')
        self._dut_enable.stateChanged.connect(
            lambda s: self._set_cmd('dut_enable', s == Qt.CheckState.Checked.value))
        dut_layout.addWidget(self._dut_enable)

        self._dut_status = QLabel('state: —')
        dut_layout.addWidget(self._dut_status)
        left.addWidget(dut_grp)

        # IO / actions
        io_grp = QGroupBox('I/O & Actions')
        io_layout = QVBoxLayout(io_grp)

        self._out1_check = QCheckBox('Hold EL2004 Output 1')
        self._out1_check.stateChanged.connect(
            lambda s: self._set_cmd('hold_output1', s == Qt.CheckState.Checked.value))
        io_layout.addWidget(self._out1_check)

        fault_btn = QPushButton('Send Fault Reset')
        fault_btn.pressed.connect(lambda: self._set_cmd('fault_reset', True))
        fault_btn.released.connect(lambda: self._set_cmd('fault_reset', False))
        io_layout.addWidget(fault_btn)

        self._stats_label = QLabel('cycle: —  wkc: —  t_us: —')
        io_layout.addWidget(self._stats_label)

        left.addWidget(io_grp)
        left.addStretch()

        # ── Right panel: plots ────────────────────────────────────────────────
        right = QVBoxLayout()
        root.addLayout(right, stretch=3)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        # Velocity plot
        vel_plot = pg.PlotWidget(title='Drive Velocity')
        vel_plot.setLabel('left',   'velocity')
        vel_plot.setLabel('bottom', 'time (s)')
        vel_plot.addLegend()
        self._curve_main_vel = vel_plot.plot(pen='b', name='main_drive')
        self._curve_dut_vel  = vel_plot.plot(pen='r', name='dut')
        right.addWidget(vel_plot)

        # Torque plot
        torque_plot = pg.PlotWidget(title='Torque Sensor (ELM3002)')
        torque_plot.setLabel('left',   'torque (Nm)')
        torque_plot.setLabel('bottom', 'time (s)')
        torque_plot.addLegend()
        self._curve_ch1_t = torque_plot.plot(pen='g', name='ch1')
        self._curve_ch2_t = torque_plot.plot(pen='m', name='ch2')
        right.addWidget(torque_plot)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _set_cmd(self, key: str, value):
        self._cmd_state[key] = value

    def _send_command(self):
        try:
            payload = json.dumps(self._cmd_state).encode()
            self._cmd_sock.sendto(payload, self._cmd_addr)
        except Exception:
            pass

    def _on_telemetry(self, t: dict):
        now = time.monotonic() - self._t0
        self._t.append(now)

        main = t.get('main', {})
        dut  = t.get('dut',  {})

        self._main_vel.append(main.get('fb_vel', 0))
        self._dut_vel.append(dut.get('fb_vel', 0))
        self._ch1_torque.append(t.get('ch1_t', 0.0))
        self._ch2_torque.append(t.get('ch2_t', 0.0))

        ts = list(self._t)
        self._curve_main_vel.setData(ts, list(self._main_vel))
        self._curve_dut_vel.setData(ts, list(self._dut_vel))
        self._curve_ch1_t.setData(ts, list(self._ch1_torque))
        self._curve_ch2_t.setData(ts, list(self._ch2_torque))

        self._main_status.setText(
            f"state: {main.get('state','—')}  sw: 0x{main.get('sw', 0):04X}")
        self._dut_status.setText(
            f"state: {dut.get('state','—')}  sw: 0x{dut.get('sw', 0):04X}")
        self._stats_label.setText(
            f"cycle: {t.get('cycle',0)}  wkc: {t.get('wkc',0)}  t_us: {t.get('t_us',0.0):.1f}")

    def closeEvent(self, event):
        self._cmd_timer.stop()
        self._receiver.stop()
        self._cmd_sock.close()
        super().closeEvent(event)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Dyno Qt/PySide6 dashboard')
    parser.add_argument('--telem-port',   type=int, default=TELEM_PORT)
    parser.add_argument('--cmd-port',     type=int, default=COMMAND_PORT)
    parser.add_argument('--bridge-host',  type=str, default='127.0.0.1')
    args = parser.parse_args()

    app = QApplication([])
    win = MainWindow(args.telem_port, args.cmd_port, args.bridge_host)
    win.show()
    app.exec()


if __name__ == '__main__':
    main()
