"""DS402 PDO packing/unpacking helpers."""

from __future__ import annotations

from dataclasses import dataclass
from struct import Struct

from .data_types import DriveCiA402States, DriveCommandBase

# Master -> drive cyclic command payload for mapped 0x1600 + 0x1601.
# 0x1600: 0x6040(U16), 0x6060(S8), 0x607A(S32), 0x60FF(S32), 0x2022(F32), 0x2523(F32)
# 0x1601: 0x2527(F32), 0x2528(F32), 0x250A(F32), 0x250B(F32), 0x250C(F32),
#         0x2511(F32), 0x2512(F32), 0x2513(F32)
# RX_PDO_STRUCT = Struct("<Hbiiffffffffff")

# Drive -> master cyclic status payload for mapped 0x1A00 + 0x1A01 + 0x1A02.
# 0x1A00: 0x6041(U16), 0x6061(S8), 0x6064(S32), 0x2060(F32), 0x6077(S16), 0x2063(F32), 0x603F(U16)
# 0x1A01: 0x606C(S32), 0x204A(S32), 0x2078(S32), 0x2079(F32), 0x203B(F32), 0x203C(F32), 0x2076(F32)
# 0x1A02: 0x2072(F32), 0x2073(F32)
# TX_PDO_STRUCT = Struct("<HbifhfHiiiffffff")

# Previous example/baseline layout retained as fallback for legacy maps.
# LEGACY_TX_PDO_STRUCT = Struct("<HbHhiiB")
AL_STATE_OPERATIONAL = 0x08


# @dataclass(slots=True)
# class PdoScaling:
#     """
#     Conversion factors between engineering units and raw PDO integer units.

#     These defaults are placeholders and should be tuned per drive/PDO map.
#     """

#     torque_lsb_per_nm: float = 10.0
#     velocity_lsb_per_rad_s: float = 1000.0
#     position_lsb_per_rad: float = 10000.0


def _clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


def _clamp_i32(value: int) -> int:
    return max(-2147483648, min(2147483647, value))


def decode_cia402_state(status_word: int) -> DriveCiA402States:
    """Decode CiA 402 logical state from statusword (0x6041)."""

    state_004f = status_word & 0x004F
    state_006f = status_word & 0x006F

    if state_004f == 0x0000:
        return DriveCiA402States.NOT_READY_TO_SWITCH_ON
    if state_004f == 0x0040:
        return DriveCiA402States.SWITCH_ON_DISABLED
    if state_006f == 0x0021:
        return DriveCiA402States.READY_TO_SWITCH_ON
    if state_006f == 0x0023:
        return DriveCiA402States.SWITCHED_ON
    if state_006f == 0x0027:
        return DriveCiA402States.OPERATION_ENABLED
    if state_006f == 0x0007:
        return DriveCiA402States.QUICK_STOP_ACTIVE
    if state_004f == 0x000F:
        return DriveCiA402States.FAULT_REACTION_ACTIVE
    if state_004f == 0x0008:
        return DriveCiA402States.FAULT
    return DriveCiA402States.NOT_READY_TO_SWITCH_ON


def _decode_statusword_bits(status_word: int) -> dict[str, bool]:
    return {
        "ready_to_switch_on": bool(status_word & (1 << 0)),
        "switched_on": bool(status_word & (1 << 1)),
        "operation_enabled": bool(status_word & (1 << 2)),
        "fault": bool(status_word & (1 << 3)),
        "voltage_enabled": bool(status_word & (1 << 4)),
        "quick_stop_active": not bool(status_word & (1 << 5)),
        "switch_on_disabled": bool(status_word & (1 << 6)),
        "warning": bool(status_word & (1 << 7)),
        "remote": bool(status_word & (1 << 9)),
        "target_reached": bool(status_word & (1 << 10)),
    }


def _controlword_from_command(
    command: DriveCommandBase, current_state: DriveCiA402States
) -> int:
    if command.clear_fault and current_state == DriveCiA402States.FAULT:
        return 0x0080
    if not command.enable_drive:
        if current_state in (
            DriveCiA402States.OPERATION_ENABLED,
            DriveCiA402States.SWITCHED_ON,
            DriveCiA402States.READY_TO_SWITCH_ON,
            DriveCiA402States.SWITCH_ON_DISABLED,
            DriveCiA402States.QUICK_STOP_ACTIVE,
        ):
            return 0x0006  # Shutdown
        return 0x0000  # Disable voltage

    if current_state == DriveCiA402States.SWITCH_ON_DISABLED:
        return 0x0006  # Shutdown -> Ready to switch on
    if current_state == DriveCiA402States.READY_TO_SWITCH_ON:
        return 0x0007  # Switch on
    if current_state == DriveCiA402States.SWITCHED_ON:
        return 0x000F  # Enable operation
    if current_state == DriveCiA402States.OPERATION_ENABLED:
        return 0x000F  # Hold operation enabled
    if current_state == DriveCiA402States.QUICK_STOP_ACTIVE:
        return 0x000F  # Recover to operation enabled path
    if current_state == DriveCiA402States.FAULT:
        return 0x0080 if command.clear_fault else 0x0000
    return 0x0000


# def pack_command(
#     command: Command, scaling: PdoScaling | None = None, current_status_word: int = 0
# ) -> bytes:
#     """Pack application `Command` into DS402 RX PDO bytes."""

#     _ = scaling  # kept for API compatibility while scale factors are disabled
#     current_state = decode_cia402_state(current_status_word)
#     controlword = _controlword_from_command(command, current_state)
#     mode = int(command.mode_of_operation)
#     # Scaling disabled intentionally: use direct typed raw values.
#     # target_torque = _clamp_i16(round(command.target_torque_nm * cfg.torque_lsb_per_nm))
#     # target_velocity = _clamp_i32(
#     #     round(command.target_velocity_rad_s * cfg.velocity_lsb_per_rad_s)
#     # )
#     # target_position = _clamp_i32(
#     #     round(command.target_position_rad * cfg.position_lsb_per_rad)
#     # )
#     target_velocity = _clamp_i32(round(command.target_velocity_rad_s))
#     target_position = _clamp_i32(round(command.target_position_rad))
#     torque_command_2022 = (
#         float(command.torque_command_2022)
#         if command.torque_command_2022 != 0.0
#         else float(command.target_torque_nm)
#     )
#     torque_kp = float(command.torque_kp)
#     torque_loop_max_output = float(command.torque_loop_max_output)
#     torque_loop_min_output = float(command.torque_loop_min_output)
#     velocity_loop_kp = float(command.velocity_loop_kp)
#     velocity_loop_ki = float(command.velocity_loop_ki)
#     velocity_loop_kd = float(command.velocity_loop_kd)
#     position_loop_kp = float(command.position_loop_kp)
#     position_loop_ki = float(command.position_loop_ki)
#     position_loop_kd = float(command.position_loop_kd)

#     return RX_PDO_STRUCT.pack(
#         controlword,
#         mode,
#         target_position,
#         target_velocity,
#         torque_command_2022,
#         torque_kp,
#         torque_loop_max_output,
#         torque_loop_min_output,
#         velocity_loop_kp,
#         velocity_loop_ki,
#         velocity_loop_kd,
#         position_loop_kp,
#         position_loop_ki,
#         position_loop_kd,
#     )


# def unpack_status(
#     pdo: bytes,
#     scaling: PdoScaling | None = None,
#     *,
#     seq: int = 0,
#     stamp_ns: int = 0,
#     cycle_time_ns: int = 0,
#     dc_time_error_ns: int = 0,
# ) -> DriveStatus:
#     """Unpack DS402 TX PDO bytes into `DriveStatus`."""

#     if len(pdo) < LEGACY_TX_PDO_STRUCT.size:
#         raise ValueError(
#             f"TX PDO payload too small: got={len(pdo)} expected_at_least={LEGACY_TX_PDO_STRUCT.size}"
#         )

#     _ = scaling  # kept for API compatibility while scale factors are disabled
#     if len(pdo) >= TX_PDO_STRUCT.size:
#         (
#             status_word,
#             mode_display,
#             measured_input_encoder_position_raw,
#             measured_bus_voltage_v,
#             estimated_torque_raw,
#             measured_motor_temp_c,
#             error_code,
#             measured_input_motor_velocity_raw,
#             raw_output_encoder_position,
#             position_setpoint_raw,
#             velocity_setpoint_raw,
#             iq_actual,
#             id_actual,
#             idc_actual,
#             iq_command,
#             id_command,
#         ) = TX_PDO_STRUCT.unpack_from(pdo, 0)
#         measured_torque_raw = estimated_torque_raw
#         measured_velocity_raw = measured_input_motor_velocity_raw
#         measured_position_raw = measured_input_encoder_position_raw
#         received_velocity_command_raw = velocity_setpoint_raw
#         al_state_code = AL_STATE_OPERATIONAL if status_word != 0 else 0
#         _ = (
#             measured_bus_voltage_v,
#             measured_motor_temp_c,
#             raw_output_encoder_position,
#             position_setpoint_raw,
#             iq_actual,
#             id_actual,
#             idc_actual,
#             iq_command,
#             id_command,
#         )
#     else:
#         (
#             status_word,
#             mode_display,
#             error_code,
#             measured_torque_raw,
#             measured_velocity_raw,
#             measured_position_raw,
#             al_state_code,
#         ) = LEGACY_TX_PDO_STRUCT.unpack_from(pdo, 0)
#         measured_bus_voltage_v = 0.0
#         received_velocity_command_raw = 0.0

#     cia402_state = decode_cia402_state(status_word)
#     status_bits = _decode_statusword_bits(status_word)
#     al_state_base = al_state_code & 0x0F

#     return DriveStatus(
        # online=al_state_base != 0,
        # operational=al_state_base == AL_STATE_OPERATIONAL,
        # faulted=cia402_state
        # in (DriveCiA402States.FAULT, DriveCiA402States.FAULT_REACTION_ACTIVE),
        # al_state_code=al_state_code,
        # cia402_state=cia402_state,
        # status_word=status_word,
        # mode_of_operation_display=mode_display,
        # error_code=error_code,
        # ready_to_switch_on=status_bits["ready_to_switch_on"],
        # switched_on=status_bits["switched_on"],
        # operation_enabled=status_bits["operation_enabled"],
        # fault=status_bits["fault"],
        # voltage_enabled=status_bits["voltage_enabled"],
        # quick_stop_active=status_bits["quick_stop_active"],
        # switch_on_disabled=status_bits["switch_on_disabled"],
        # warning=status_bits["warning"],
        # remote=status_bits["remote"],
        # target_reached=status_bits["target_reached"],
        # # Scaling disabled intentionally: values are kept in direct raw typed units.
        # # measured_torque_nm=measured_torque_raw / cfg.torque_lsb_per_nm,
        # # measured_velocity_rad_s=measured_velocity_raw / cfg.velocity_lsb_per_rad_s,
        # # measured_position_rad=measured_position_raw / cfg.position_lsb_per_rad,
        # # Java reference:
        # # - TPDO 0x1A01 measuredInputMotorVelocity is Signed32
        # # - TPDO 0x1A00 estimatedTorque is Signed16
        # measured_torque_nm=float(measured_torque_raw),
        # measured_velocity_rad_s=float(measured_velocity_raw),
        # measured_position_rad=float(measured_position_raw),
        # velocity_command_received=float(received_velocity_command_raw),
        # bus_voltage=float(measured_bus_voltage_v),
        # dc_time_error_ns=dc_time_error_ns,
        # cycle_time_ns=cycle_time_ns,
        # seq=seq,
        # stamp_ns=stamp_ns,
    # )
