#!/usr/bin/env python3
"""DS402 static torque-ramp exercise (safe low-level command profile)."""

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

from ethercat_core.data_types import SystemCommand
from ethercat_core.loop import EthercatLoop
from ethercat_core.master import EthercatMaster, load_topology
from ethercat_core.devices.ds402.data_types import Command, ModeOfOperation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a safe DS402 static torque-ramp test."
    )
    parser.add_argument(
        "--topology",
        default="config/topology.debug.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="main_drive",
        help="Configured slave name to command.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=12.0,
        help="Total test duration.",
    )
    parser.add_argument(
        "--ramp-time-s",
        type=float,
        default=4.0,
        help="Time to ramp command from 0 to max torque.",
    )
    parser.add_argument(
        "--hold-time-s",
        type=float,
        default=4.0,
        help="Time to hold at max torque before returning to 0.",
    )
    parser.add_argument(
        "--max-torque",
        type=float,
        default=150.0,
        help="Peak target torque command (raw typed units).",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="Status print rate.",
    )
    return parser.parse_args()


def _torque_profile(
    t_s: float,
    *,
    ramp_time_s: float,
    hold_time_s: float,
    max_torque: float,
) -> float:
    if t_s <= 0.0:
        return 0.0

    ramp = max(ramp_time_s, 1e-3)
    hold = max(hold_time_s, 0.0)

    if t_s < ramp:
        return max_torque * (t_s / ramp)
    if t_s < ramp + hold:
        return max_torque
    if t_s < (2.0 * ramp + hold):
        return max_torque * (1.0 - (t_s - ramp - hold) / ramp)
    return 0.0


def main() -> int:
    args = parse_args()
    cfg = load_topology(args.topology)
    master = EthercatMaster(cfg)

    try:
        runtime = master.initialize()
        if args.slave not in runtime.adapters:
            raise RuntimeError(
                f"Unknown slave '{args.slave}'. Available: {list(runtime.adapters.keys())}"
            )
        startup_params = runtime.startup_params.get(args.slave, {})
        if startup_params:
            print(
                "Loaded startup gains: "
                + ", ".join(f"{k}={v}" for k, v in startup_params.items())
            )
        kt = float(startup_params.get("motor_kt", 0.0))
        torque_kp = 0.0 if abs(kt) < 1e-9 else 1.0 / kt
        vel_qr = float(startup_params.get("torque_loop_max_output", 0.0))
        vel_is = float(startup_params.get("torque_loop_min_output", 0.0))
        vel_kp = float(startup_params.get("velocity_loop_kp", 0.0))
        vel_ki = float(startup_params.get("velocity_loop_ki", 0.0))
        vel_kd = float(startup_params.get("velocity_loop_kd", 0.0))
        pos_kp = float(startup_params.get("position_loop_kp", 0.0))
        pos_ki = float(startup_params.get("position_loop_ki", 0.0))
        pos_kd = float(startup_params.get("position_loop_kd", 0.0))

        loop = EthercatLoop(runtime, cycle_hz=cfg.cycle_hz)
        loop.start()

        start = time.monotonic()
        deadline = start + max(0.0, args.duration_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = start

        while time.monotonic() < deadline:
            now = time.monotonic()
            elapsed_s = now - start
            target_torque = _torque_profile(
                elapsed_s,
                ramp_time_s=args.ramp_time_s,
                hold_time_s=args.hold_time_s,
                max_torque=args.max_torque,
            )

            loop.set_command(
                SystemCommand(
                    by_slave={
                        args.slave: Command(
                            mode_of_operation=ModeOfOperation.CYCLIC_SYNC_TORQUE,
                            target_torque_nm=target_torque,
                            target_velocity_rad_s=0.0,
                            target_position_rad=0.0,
                            torque_kp=torque_kp,
                            torque_loop_max_output=vel_qr,
                            torque_loop_min_output=vel_is,
                            velocity_loop_kp=vel_kp,
                            velocity_loop_ki=vel_ki,
                            velocity_loop_kd=vel_kd,
                            position_loop_kp=pos_kp,
                            position_loop_ki=pos_ki,
                            position_loop_kd=pos_kd,
                            enable_drive=True,
                            clear_fault=False,
                        )
                    }
                )
            )

            if now >= next_print:
                status = loop.get_status()
                stats = loop.stats
                ds = status.by_slave.get(args.slave)
                if ds is None:
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} command={target_torque:.3f} no status yet"
                    )
                else:
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} "
                        f"command={target_torque:.3f} measured={ds.measured_torque_nm:.3f} "
                        f"vel={ds.measured_velocity_rad_s:.3f} state={ds.cia402_state.name}"
                    )
                next_print = now + print_period

            time.sleep(0.002)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
