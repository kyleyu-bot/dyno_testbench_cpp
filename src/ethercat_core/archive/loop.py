"""Cyclic EtherCAT process-data loop over configured slave adapters."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Set

from .data_types import SystemCommand, SystemStatus
from .master import MasterRuntime

# ---------------------------------------------------------------------------
# clock_nanosleep via librt — more precise than time.sleep on RT kernels.
# Falls back to time.sleep if librt is unavailable.
# ---------------------------------------------------------------------------

_CLOCK_MONOTONIC = 1
_TIMER_ABSTIME = 1


class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


def _load_librt() -> ctypes.CDLL | None:
    for name in ("librt.so.1", "librt.so", ctypes.util.find_library("rt") or ""):
        if not name:
            continue
        try:
            return ctypes.CDLL(name, use_errno=True)
        except OSError:
            continue
    return None


_librt = _load_librt()


def _clock_nanosleep_abstime(abs_ns: int) -> None:
    """Sleep until abs_ns (CLOCK_MONOTONIC, absolute).  Falls back to time.sleep."""
    if _librt is not None:
        ts = _Timespec(abs_ns // 1_000_000_000, abs_ns % 1_000_000_000)
        _librt.clock_nanosleep(_CLOCK_MONOTONIC, _TIMER_ABSTIME, ctypes.byref(ts), None)
    else:
        remaining = abs_ns - time.monotonic_ns()
        if remaining > 0:
            time.sleep(remaining / 1_000_000_000)


# ---------------------------------------------------------------------------


@dataclass(slots=True)
class LoopStats:
    """Basic loop timing and bus statistics."""

    cycle_count: int = 0
    last_wkc: int = 0
    last_cycle_time_ns: int = 0
    last_dc_error_ns: int = 0
    last_period_ns: int = 0
    last_wakeup_latency_ns: int = 0


@dataclass(slots=True)
class LoopConfig:
    """Optional real-time configuration for the cyclic loop thread.

    rt_priority:
        SCHED_FIFO priority (1–99).  0 = keep default SCHED_OTHER.
        Requires CAP_SYS_NICE or running as root.
    cpu_affinity:
        Set of CPU indices to pin the loop thread to.
        Empty set = no affinity change.
    """

    rt_priority: int = 0
    cpu_affinity: Set[int] = field(default_factory=set)


class EthercatLoop:
    """Cyclic loop using master runtime and per-slave adapters.

    Pass a LoopConfig to enable real-time scheduling and/or CPU affinity
    on the loop thread for reduced jitter on RT-PREEMPT kernels.
    """

    def __init__(
        self,
        runtime: MasterRuntime,
        cycle_hz: int = 1000,
        rt_config: LoopConfig | None = None,
    ):
        if cycle_hz <= 0:
            raise ValueError("cycle_hz must be > 0")

        self._runtime = runtime
        self._cycle_hz = cycle_hz
        self._cycle_ns = int(1_000_000_000 / cycle_hz)
        self._rt_config = rt_config or LoopConfig()

        self._lock = threading.Lock()
        self._pending_command = SystemCommand()
        self._latest_status = SystemStatus()
        self._stats = LoopStats()

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def stats(self) -> LoopStats:
        with self._lock:
            return LoopStats(
                cycle_count=self._stats.cycle_count,
                last_wkc=self._stats.last_wkc,
                last_cycle_time_ns=self._stats.last_cycle_time_ns,
                last_dc_error_ns=self._stats.last_dc_error_ns,
                last_period_ns=self._stats.last_period_ns,
                last_wakeup_latency_ns=self._stats.last_wakeup_latency_ns,
            )

    def set_command(self, command: SystemCommand) -> None:
        with self._lock:
            self._pending_command = command

    def get_status(self) -> SystemStatus:
        with self._lock:
            return SystemStatus(
                by_slave=dict(self._latest_status.by_slave),
                seq=self._latest_status.seq,
                stamp_ns=self._latest_status.stamp_ns,
            )

    def run_once(self) -> SystemStatus:
        start_ns = time.monotonic_ns()
        command = self._snapshot_command(start_ns)

        # Encode command payload for each configured slave adapter.
        for name, adapter in self._runtime.adapters.items():
            slave = self._runtime.slaves_by_name[name]
            payload = self._encode_payload(adapter, command.by_slave.get(name))
            slave.output = payload

        self._runtime.master.send_processdata()
        wkc = int(self._runtime.master.receive_processdata(2000))

        end_ns = time.monotonic_ns()
        cycle_time_ns = end_ns - start_ns
        dc_error_ns = cycle_time_ns - self._cycle_ns

        status_by_slave: Dict[str, Any] = {}
        for name, adapter in self._runtime.adapters.items():
            slave = self._runtime.slaves_by_name[name]
            status_by_slave[name] = adapter.unpack_tx_pdo(
                bytes(slave.input),
                seq=command.seq,
                stamp_ns=end_ns,
                cycle_time_ns=cycle_time_ns,
                dc_time_error_ns=dc_error_ns,
            )

        status = SystemStatus(by_slave=status_by_slave, seq=command.seq, stamp_ns=end_ns)
        with self._lock:
            self._stats.cycle_count += 1
            self._stats.last_wkc = wkc
            self._stats.last_cycle_time_ns = cycle_time_ns
            self._stats.last_dc_error_ns = dc_error_ns
            # Only publish status when the bus confirmed delivery (wkc > 0).
            # A zero wkc means receive_processdata got no fresh frame and
            # slave.input still holds the previous cycle's bytes — publishing
            # that with a new stamp_ns would silently look fresh to readers.
            if wkc > 0:
                self._latest_status = status
        return status

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_forever, daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout_s)

    def _apply_rt_config(self) -> None:
        cfg = self._rt_config

        if cfg.cpu_affinity:
            try:
                os.sched_setaffinity(0, cfg.cpu_affinity)
            except OSError as exc:
                print(f"[EthercatLoop] WARNING: cpu_affinity failed: {exc}")

        if cfg.rt_priority > 0:
            try:
                param = os.sched_param(cfg.rt_priority)
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError as exc:
                print(f"[EthercatLoop] WARNING: SCHED_FIFO priority={cfg.rt_priority} failed: {exc}")

    def _run_forever(self) -> None:
        self._apply_rt_config()

        next_tick = time.monotonic_ns()
        prev_start_ns = 0
        while not self._stop_event.is_set():
            start_ns = time.monotonic_ns()
            period_ns = 0 if prev_start_ns == 0 else start_ns - prev_start_ns
            wakeup_latency_ns = start_ns - next_tick
            self.run_once()
            with self._lock:
                self._stats.last_period_ns = period_ns
                self._stats.last_wakeup_latency_ns = wakeup_latency_ns
            prev_start_ns = start_ns
            next_tick += self._cycle_ns
            now = time.monotonic_ns()
            if now >= next_tick:
                # Missed deadline — reset to avoid chasing accumulated lag.
                next_tick = now
            else:
                _clock_nanosleep_abstime(next_tick)

    def _snapshot_command(self, stamp_ns: int) -> SystemCommand:
        with self._lock:
            seq = self._pending_command.seq + 1
            by_slave = dict(self._pending_command.by_slave)
            self._pending_command = SystemCommand(
                by_slave=by_slave, seq=seq, stamp_ns=stamp_ns
            )
            return self._pending_command

    @staticmethod
    def _encode_payload(adapter: Any, command: Any) -> bytes:
        if command is None:
            return bytes(adapter.rx_pdo_size)
        payload = adapter.pack_rx_pdo(command)
        if len(payload) != adapter.rx_pdo_size:
            raise ValueError(
                f"Encoded payload size mismatch: expected={adapter.rx_pdo_size} got={len(payload)}"
            )
        return payload
