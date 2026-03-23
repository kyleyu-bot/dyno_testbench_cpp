#!/usr/bin/env python3
"""Read-only DS402 TxPDO monitor for Novanta Volcano with interactive field selection."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import fields
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.loop import EthercatLoop, LoopConfig
from ethercat_core.master import (
    EthercatMaster,
    al_state_name,
    load_topology,
    resolve_slave_position,
)
from ethercat_core.devices.motor_drives.Novanta.Volcano.data_types import DriveStatus

MAX_FIELDS = 6
DEFAULT_FIELDS = [
    "status_word",
    "cia402_state",
    "bus_voltage",
    "error_code",
    "measured_velocity_rad_s",
    "measured_position_rad",
]


def _parse_cpu_affinity(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        token = item.strip()
        if not token:
            continue
        cpu = int(token, 10)
        if cpu < 0:
            raise argparse.ArgumentTypeError("CPU indices must be >= 0.")
        cpus.add(cpu)
    if not cpus:
        raise argparse.ArgumentTypeError("CPU affinity must include at least one CPU.")
    return cpus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read Volcano TxPDO data in read-only mode and print selected fields."
    )
    parser.add_argument(
        "--topology",
        default="config/topology.dyno2.template5.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="dut",
        help="Configured slave name to observe.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Monitor duration in seconds.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=5.0,
        help="Terminal update rate.",
    )
    parser.add_argument(
        "--fields",
        default="",
        help="Comma-separated field names (or indices) to display, max 6.",
    )
    parser.add_argument(
        "--rt-priority",
        type=int,
        default=0,
        help="Loop thread SCHED_FIFO priority (1-99). 0 keeps default scheduler.",
    )
    parser.add_argument(
        "--cpu-affinity",
        type=_parse_cpu_affinity,
        default=set(),
        help="Comma-separated CPU indices for the loop thread, e.g. '2' or '2,3'.",
    )
    return parser.parse_args()


def _available_fields() -> list[str]:
    return [f.name for f in fields(DriveStatus)]


def _parse_selected_fields(raw: str, available: list[str]) -> list[str]:
    items = [x.strip() for x in raw.split(",") if x.strip()]
    if not items:
        return []

    selected: list[str] = []
    for item in items:
        if item.isdigit():
            idx = int(item) - 1
            if idx < 0 or idx >= len(available):
                raise ValueError(f"Field index out of range: {item}")
            name = available[idx]
        else:
            name = item
            if name not in available:
                raise ValueError(f"Unknown field: {name}")

        if name not in selected:
            selected.append(name)

    if len(selected) > MAX_FIELDS:
        raise ValueError(f"Select at most {MAX_FIELDS} fields.")
    return selected


def _prompt_for_fields(available: list[str]) -> list[str]:
    print("Available DriveStatus fields:")
    for i, name in enumerate(available, start=1):
        print(f"  {i:2d}. {name}")
    print(
        f"Select up to {MAX_FIELDS} fields by name or index (comma-separated). "
        "Press Enter for default."
    )
    raw = input("> ").strip()
    if not raw:
        return DEFAULT_FIELDS
    return _parse_selected_fields(raw, available)


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    resolved_position = resolve_slave_position(cfg, args.slave)
    for slave_cfg in cfg.slaves:
        if slave_cfg.name == args.slave:
            slave_cfg.position = resolved_position
            break
    master = EthercatMaster(cfg)

    available = _available_fields()
    try:
        selected = (
            _parse_selected_fields(args.fields, available)
            if args.fields
            else _prompt_for_fields(available)
        )
    except ValueError as exc:
        print(f"Field selection error: {exc}")
        return 2

    if not selected:
        selected = DEFAULT_FIELDS

    try:
        runtime = master.initialize()
        if args.slave not in runtime.adapters:
            raise RuntimeError(
                f"Unknown slave '{args.slave}'. Available: {list(runtime.adapters.keys())}"
            )

        rt_priority = max(0, min(args.rt_priority, 99))
        loop = EthercatLoop(
            runtime,
            cycle_hz=cfg.cycle_hz,
            rt_config=LoopConfig(
                rt_priority=rt_priority,
                cpu_affinity=args.cpu_affinity,
            ),
        )
        loop.start()

        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"for {args.duration_s:.1f}s | "
            f"rt_priority={rt_priority} cpu_affinity={sorted(args.cpu_affinity) or 'none'}"
        )
        print("Fields:", ", ".join(selected))

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                status = loop.get_status()
                stats = loop.stats
                ds = status.by_slave.get(args.slave)
                slave = runtime.slaves_by_name[args.slave]

                if ds is None:
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} "
                        f"al={al_state_name(int(slave.state))} status=unavailable"
                    )
                else:
                    values = [f"{k}={getattr(ds, k)}" for k in selected]
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} "
                        f"al={al_state_name(int(slave.state))} "
                        + " ".join(values)
                    )
                next_print = now + print_period

            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
