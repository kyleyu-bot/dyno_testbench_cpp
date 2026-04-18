#!/usr/bin/env python3
"""Read and print Beckhoff EL5032 TxPDO data in the cyclic loop."""

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
from ethercat_core.devices.beckhoff.el5032.adapter import El5032SlaveAdapter
from ethercat_core.devices.beckhoff.el5032.data_types import El5032Data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and print EL5032 TxPDO data using the cyclic loop."
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
        adapter = runtime.adapters.get(args.slave)
        if not isinstance(adapter, El5032SlaveAdapter):
            raise RuntimeError(
                f"Slave '{args.slave}' is not an EL5032. Adapter={type(adapter).__name__}"
            )

        loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
        loop.start()

        deadline = time.monotonic() + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = time.monotonic()

        print(
            f"Monitoring '{args.slave}' at position {resolved_position} "
            f"for {args.duration_s:.1f}s"
        )

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_print:
                status = loop.get_status()
                data = status.by_slave.get(args.slave)
                if not isinstance(data, El5032Data):
                    print(
                        "raw_pdo=unavailable "
                        "encoder_value_raw=unavailable "
                        "encoder_count_25bit=unavailable"
                    )
                else:
                    print(
                        # f"raw_pdo={data.raw_pdo.hex()} "
                        # f"encoder_value_raw_first10={int(data.encoder_value_raw).to_bytes(10, 'little', signed=True).hex()} "
                        # f"encoder_value_raw={data.encoder_value_raw} "
                        f"encoder_count_25bit={adapter.get_encoder_count_25bit(data)}"
                    )
                next_print = now + print_period
            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
