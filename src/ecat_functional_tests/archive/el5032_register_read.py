#!/usr/bin/env python3
"""Read Beckhoff EL5032 register 0x6000:11 in a simple polling loop."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.master import (
    MasterConfigError,
    load_topology,
    require_pysoem,
    resolve_slave_position,
)


REGISTER_INDEX = 0x6000
REGISTER_SUBINDEX = 0x11
LOW_25BIT_MASK = (1 << 25) - 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Poll Beckhoff EL5032 register 0x6000:11."
    )
    parser.add_argument(
        "--topology",
        default="config/ethercat_device_config/topology.dyno2.template2.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="encoder_interface",
        help="Configured EL5032 slave name to observe.",
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
        help="Register print rate.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pysoem = require_pysoem()

    cfg = load_topology(args.topology)
    resolved_position = resolve_slave_position(cfg, args.slave)

    master = pysoem.Master()
    master.open(cfg.iface)
    try:
        slave_count = master.config_init()
        if slave_count <= 0:
            raise RuntimeError("No EtherCAT slaves detected.")

        if resolved_position >= len(master.slaves):
            raise MasterConfigError(
                f"Resolved position {resolved_position} out of range; detected {len(master.slaves)} slaves."
            )

        slave = master.slaves[resolved_position]
        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"register 0x{REGISTER_INDEX:04X}:{REGISTER_SUBINDEX:02X} for {args.duration_s:.1f}s"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                raw = slave.sdo_read(REGISTER_INDEX, REGISTER_SUBINDEX)
                if isinstance(raw, int):
                    value = int(raw)
                elif isinstance(raw, (bytes, bytearray)):
                    payload = bytes(raw)
                    if len(payload) != 8:
                        raise ValueError(
                            f"Expected 8 bytes for 0x{REGISTER_INDEX:04X}:{REGISTER_SUBINDEX:02X}, got {len(payload)}"
                        )
                    value = int.from_bytes(payload, "little", signed=True)
                else:
                    raise TypeError(f"Unexpected SDO payload type: {type(raw)}")
                low_25_bits = value & LOW_25BIT_MASK
                print(
                    f"register_0x{REGISTER_INDEX:04X}_{REGISTER_SUBINDEX:02X}={value} "
                    f"lsb_25={low_25_bits}"
                )
                next_print = now + print_period
            time.sleep(0.005)

        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
