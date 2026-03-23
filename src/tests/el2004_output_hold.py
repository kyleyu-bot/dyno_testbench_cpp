#!/usr/bin/env python3
"""Hold EL2004 output channel 1 high, then clear it on timeout or user input."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.data_types import SystemCommand
from ethercat_core.loop import EthercatLoop
from ethercat_core.master import EthercatMaster, load_topology
from ethercat_core.devices.beckhoff.el2004.data_types import El2004Command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Set EL2004 output 1 high, then clear it after a hold period or user input."
    )
    parser.add_argument(
        "--topology",
        default="config/topology.dyno2.template1.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="digital_IO",
        help="Configured EL2004 slave name to command.",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=60.0,
        help="Seconds to hold output 1 high before clearing it.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=2.0,
        help="Status print rate while output 1 is held high.",
    )
    return parser.parse_args()


def _watch_for_enter(stop_event: threading.Event) -> None:
    try:
        input("Press Enter to clear EL2004 output 1 early.\n")
    except EOFError:
        return
    stop_event.set()


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    master = EthercatMaster(cfg)
    loop: EthercatLoop | None = None

    try:
        runtime = master.initialize()
        if args.slave not in runtime.adapters:
            raise RuntimeError(
                f"Unknown slave '{args.slave}'. Available: {list(runtime.adapters.keys())}"
            )

        loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
        loop.start()

        stop_event = threading.Event()
        input_thread = threading.Thread(
            target=_watch_for_enter, args=(stop_event,), daemon=True
        )
        input_thread.start()

        high_cmd = El2004Command(output_1=True)
        low_cmd = El2004Command(output_1=False)

        loop.set_command(SystemCommand(by_slave={args.slave: high_cmd}))
        print(
            "Set EL2004 output 1 high "
            "(PDO object 1 / 0x7010:01 in the configured mapping)."
        )

        deadline = time.monotonic() + max(0.0, args.hold_s)
        print_period_s = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()
        cleared_by = "timeout"

        while time.monotonic() < deadline:
            if stop_event.is_set():
                cleared_by = "keyboard"
                break

            now = time.monotonic()
            if now >= next_print:
                remaining_s = max(0.0, deadline - now)
                status = loop.get_status().by_slave.get(args.slave)
                output_byte = getattr(status, "output_byte", 0)
                print(
                    f"holding output_1=1 output_byte=0x{output_byte:02X} "
                    f"remaining_s={remaining_s:.1f}"
                )
                next_print = now + print_period_s

            time.sleep(0.05)

        loop.set_command(SystemCommand(by_slave={args.slave: low_cmd}))
        time.sleep(max(2.0 / max(cfg.cycle_hz, 1), 0.01))
        status = loop.get_status().by_slave.get(args.slave)
        output_byte = getattr(status, "output_byte", 0)
        print(f"Cleared EL2004 output 1 via {cleared_by}. output_byte=0x{output_byte:02X}")

        loop.stop()
        master.close()
        return 0
    except KeyboardInterrupt:
        try:
            if loop is not None:
                loop.set_command(SystemCommand(by_slave={args.slave: El2004Command()}))
                time.sleep(max(2.0 / max(cfg.cycle_hz, 1), 0.01))
                loop.stop()
                loop = None
            time.sleep(max(2.0 / max(cfg.cycle_hz, 1), 0.01))
        except Exception:
            pass
        master.close()
        print("Interrupted. Requested EL2004 output 1 clear before exit.")
        return 130
    finally:
        if loop is not None:
            loop.stop()
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
