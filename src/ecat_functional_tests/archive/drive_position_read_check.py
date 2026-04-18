#!/usr/bin/env python3
"""Read-only DS402 position monitor for TxPDO 0x1A00:3 (0x6064)."""

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

from ethercat_core.loop import EthercatLoop
from ethercat_core.master import EthercatMaster, load_topology, resolve_slave_position


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and print DS402 position from TxPDO 0x1A00:3 (0x6064)."
    )
    parser.add_argument(
        "--topology",
        default="config/ethercat_device_config/topology.debug.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="main_drive",
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
        default=10.0,
        help="Terminal update rate.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    resolved_position = resolve_slave_position(cfg, args.slave)
    for slave_cfg in cfg.slaves:
        if slave_cfg.name == args.slave:
            slave_cfg.position = resolved_position
            break
    master = EthercatMaster(cfg)

    try:
        runtime = master.initialize()
        if args.slave not in runtime.adapters:
            raise RuntimeError(
                f"Unknown slave '{args.slave}'. Available: {list(runtime.adapters.keys())}"
            )

        loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
        loop.start()

        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"(TxPDO 0x1A00:3, object 0x6064) for {args.duration_s:.1f}s"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                status = loop.get_status()
                ds = status.by_slave.get(args.slave)
                if ds is None:
                    print("position_0x6064=unavailable")
                else:
                    # `measured_position_rad` currently carries raw 0x6064 typed value.
                    print(f"position_0x6064={int(ds.measured_position_rad)}")
                next_print = now + print_period
            time.sleep(0.002)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
