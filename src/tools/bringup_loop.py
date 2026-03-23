#!/usr/bin/env python3
"""Minimal bring-up runner: init master, run loop, and print status snapshots."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow running this script directly before packaging/install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.loop import EthercatLoop
from ethercat_core.master import EthercatMaster, al_state_name, load_topology


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run EtherCAT bring-up loop.")
    parser.add_argument(
        "--topology",
        default="config/topology.debug.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=5.0,
        help="How long to run the loop before exiting.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=2.0,
        help="Status print rate while loop is running.",
    )
    parser.add_argument(
        "--typed-loop",
        action="store_true",
        help="Use typed adapter loop (requires matching PDO map).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    master = EthercatMaster(cfg)

    try:
        runtime = master.initialize()
        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period_s = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()
        cycle_count = 0
        wkc = 0

        loop = None
        if args.typed_loop:
            loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
            loop.start()

        while time.monotonic() < deadline:
            if loop is None:
                runtime.master.send_processdata()
                wkc = int(runtime.master.receive_processdata(2000))
                cycle_count += 1

            now = time.monotonic()
            if now >= next_print:
                if loop is None:
                    print(f"cycle={cycle_count} wkc={wkc} mode=raw")
                else:
                    status = loop.get_status()
                    stats = loop.stats
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} "
                        f"cycle_ns={stats.last_cycle_time_ns} dc_err_ns={stats.last_dc_error_ns} mode=typed"
                    )
                for name, slave in runtime.slaves_by_name.items():
                    configured = True if loop is None else (name in status.by_slave)
                    print(f"  {name}: al={al_state_name(int(slave.state))} configured={configured}")
                next_print = now + print_period_s
            time.sleep(0.001)

        if loop is not None:
            loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
