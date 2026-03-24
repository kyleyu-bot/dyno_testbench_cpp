#!/usr/bin/env python3
"""Simple DS402 velocity command/readback test for Novanta Volcano."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

# Allow direct execution before install.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ethercat_core.data_types import SystemCommand
from ethercat_core.loop import EthercatLoop, LoopConfig
from ethercat_core.master import EthercatMaster, load_topology, resolve_slave_position
from ethercat_core.devices.base import SdoReadSpec
from ethercat_core.archive.devices.motor_drives.novanta.Volcano.data_types import Command, DriveCiA402States, ModeOfOperation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send simple speed command and monitor Volcano drive speed feedback."
    )
    parser.add_argument(
        "--topology",
        default="config/topology.dyno2.template5.json",
        help="Path to topology JSON file.",
    )
    parser.add_argument(
        "--slave",
        default="dut",
        help="Configured slave name to command.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=1000,
        help="Speed command as int32 (maps to RxPDO 0x1600:04 / 0x60FF).",
    )
    parser.add_argument(
        "--mode",
        type=int,
        default=int(ModeOfOperation.CYCLIC_SYNC_VELOCITY),
        help="Commanded mode-of-operation value for 0x6060 (default: 9 / CSV).",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Total test duration in seconds.",
    )
    parser.add_argument(
        "--fault-reset-s",
        type=float,
        default=0.5,
        help="Fault-reset phase duration at test start.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=5.0,
        help="Terminal status print rate.",
    )
    parser.add_argument(
        "--write-startup-sdos",
        action="store_true",
        help="Write loaded startup SDO values back to the drive before loop start.",
    )
    parser.add_argument(
        "--force-sdo-mode",
        action="store_true",
        help="Force-write 0x6060 mode once via SDO before loop start.",
    )
    parser.add_argument(
        "--debug-gain-sdos",
        action="store_true",
        help="Print raw + decoded SDO values for 0x250A/0x250B at startup.",
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


def _clamp_i32(value: int) -> int:
    return max(-2147483648, min(2147483647, value))


def _torque_kp_from_startup_params(params: dict[str, object]) -> float:
    kt = float(params.get("motor_kt", 0.0))
    if abs(kt) < 1e-9:
        return 0.0
    return 1.0 / kt


def _debug_gain_registers(runtime: object, slave_name: str) -> None:
    slave = runtime.slaves_by_name[slave_name]
    gain_regs = [
        ("velocity_loop_kp", 0x250A, 0x00),
        ("velocity_loop_ki", 0x250B, 0x00),
    ]
    for name, index, subindex in gain_regs:
        try:
            raw = bytes(slave.sdo_read(index, subindex))
        except Exception as exc:
            print(f"{name} sdo_read failed at 0x{index:04X}:{subindex:02X}: {exc}")
            continue

        raw_hex = raw[:8].hex()
        f32 = struct.unpack("<f", raw[:4])[0] if len(raw) >= 4 else float("nan")
        u32 = int.from_bytes(raw[:4], "little", signed=False) if len(raw) >= 4 else 0
        s32 = int.from_bytes(raw[:4], "little", signed=True) if len(raw) >= 4 else 0
        print(
            f"{name} 0x{index:04X}:{subindex:02X} raw={raw_hex} f32={f32} u32={u32} s32={s32}"
        )


def _encode_sdo_value(value: object, spec: SdoReadSpec) -> bytes:
    dtype = spec.data_type
    if dtype == "bytes":
        if not isinstance(value, (bytes, bytearray)):
            raise ValueError(f"{spec.name} expects bytes payload.")
        return bytes(value)
    if dtype == "u8":
        return int(value).to_bytes(1, "little", signed=False)
    if dtype == "s8":
        return int(value).to_bytes(1, "little", signed=True)
    if dtype == "u16":
        return int(value).to_bytes(2, "little", signed=False)
    if dtype == "s16":
        return int(value).to_bytes(2, "little", signed=True)
    if dtype == "u32":
        return int(value).to_bytes(4, "little", signed=False)
    if dtype == "s32":
        return int(value).to_bytes(4, "little", signed=True)
    if dtype == "f32":
        return struct.pack("<f", float(value))
    raise ValueError(f"Unsupported SDO type '{dtype}' for {spec.name}.")


def _apply_startup_params_to_drive(runtime: object, slave_name: str) -> dict[str, object]:
    params = dict(getattr(runtime, "startup_params", {}).get(slave_name, {}))
    if not params:
        return params

    adapter = runtime.adapters[slave_name]
    slave = runtime.slaves_by_name[slave_name]
    specs = getattr(adapter, "startup_read_specs", lambda: {})()
    for key, value in params.items():
        spec = specs.get(key)
        if spec is None:
            continue
        payload = _encode_sdo_value(value, spec)
        last_exc: Exception | None = None
        for attempt in range(5):
            try:
                slave.sdo_write(int(spec.index), int(spec.subindex), payload)
                last_exc = None
                break
            except Exception as exc:
                last_exc = exc
                if attempt < 4:
                    time.sleep(0.02)
        if last_exc is not None:
            raise RuntimeError(
                f"Failed writing startup SDO '{key}' at "
                f"0x{int(spec.index):04X}:{int(spec.subindex):02X}: {last_exc}"
            ) from last_exc
    return params


def _write_mode_sdo(runtime: object, slave_name: str, mode_value: int) -> None:
    slave = runtime.slaves_by_name[slave_name]
    payload = int(mode_value).to_bytes(1, "little", signed=True)
    last_exc: Exception | None = None
    for attempt in range(5):
        try:
            slave.sdo_write(0x6060, 0x00, payload)
            last_exc = None
            return
        except Exception as exc:
            last_exc = exc
            if attempt < 4:
                time.sleep(0.02)
    raise RuntimeError(f"Failed writing 0x6060 mode={mode_value}: {last_exc}") from last_exc


def main() -> int:
    args = parse_args()
    try:
        cmd_mode = ModeOfOperation(args.mode)
    except ValueError as exc:
        raise SystemExit(
            f"Unsupported --mode value {args.mode}. Use one of: {[int(m) for m in ModeOfOperation]}"
        ) from exc

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
        startup_params = dict(runtime.startup_params.get(args.slave, {}))
        if startup_params:
            print(
                "Loaded startup gains: "
                + ", ".join(f"{k}={v}" for k, v in startup_params.items())
            )
        if args.debug_gain_sdos:
            _debug_gain_registers(runtime, args.slave)
        torque_kp = _torque_kp_from_startup_params(startup_params)
        vel_qr = float(startup_params.get("torque_loop_max_output", 0.0))
        vel_is = float(startup_params.get("torque_loop_min_output", 0.0))
        vel_kp = float(startup_params.get("velocity_loop_kp", 0.0))
        vel_ki = float(startup_params.get("velocity_loop_ki", 0.0))
        vel_kd = float(startup_params.get("velocity_loop_kd", 0.0))
        pos_kp = float(startup_params.get("position_loop_kp", 0.0))
        pos_ki = float(startup_params.get("position_loop_ki", 0.0))
        pos_kd = float(startup_params.get("position_loop_kd", 0.0))
        if args.write_startup_sdos and startup_params:
            _apply_startup_params_to_drive(runtime, args.slave)
            print("Wrote startup gains back via SDO.")
        if args.force_sdo_mode:
            _write_mode_sdo(runtime, args.slave, args.mode)
            print(f"Forced SDO mode write: 0x6060={args.mode}")
        print(f"Using '{args.slave}' at position {resolved_position}")

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

        t0 = time.monotonic()
        deadline = t0 + max(0.0, args.duration_s)
        reset_deadline = t0 + max(0.0, args.fault_reset_s)
        print_period = 1.0 / max(args.print_hz, 0.1)
        next_print = t0

        speed_cmd_i32 = _clamp_i32(int(args.speed))

        while time.monotonic() < deadline:
            now = time.monotonic()
            in_reset = now < reset_deadline

            status = loop.get_status()
            ds = status.by_slave.get(args.slave)
            drive_enabled = (
                ds is not None
                and ds.cia402_state == DriveCiA402States.OPERATION_ENABLED
            )

            cmd = Command(
                mode_of_operation=cmd_mode,
                target_torque_nm=0.0,
                target_velocity_rad_s=float(speed_cmd_i32) if drive_enabled else 0.0,
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
                enable_drive=not in_reset,
                clear_fault=in_reset,
            )
            loop.set_command(SystemCommand(by_slave={args.slave: cmd}))

            if now >= next_print:
                stats = loop.stats
                if ds is None:
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} cmd_60FF={speed_cmd_i32} speed_606C=unavailable"
                    )
                else:
                    print(
                        f"cycle={stats.cycle_count} wkc={stats.last_wkc} "
                        f"torque_max_out={vel_qr:.6f} torque_min_out={vel_is:.6f} "
                        f"vel_kp={vel_kp:.6f} vel_ki={vel_ki:.6f} torque_kp={torque_kp:.6f} "
                        f"state={ds.cia402_state.name} "
                        f"cmd_6060={args.mode} "
                        f"mode_6061={ds.mode_of_operation_display} "
                        f"cmd_60FF={speed_cmd_i32} speed_606C={int(ds.measured_velocity_rad_s)} "
                        f"rx_cmd_2079={ds.velocity_command_received:.3f} "
                        f"bus_v_2060={ds.bus_voltage:.3f} "
                        f"status=0x{ds.status_word:04X} err=0x{ds.error_code:04X}"
                    )
                next_print = now + print_period

            time.sleep(0.005)

        loop.stop()
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
