"""Novanta Volcano slave adapter implementation."""

from __future__ import annotations

from dataclasses import dataclass, field

from ....base import SdoReadSpec
from ...drive_bases.ds402.adapter import Ds402SlaveAdapter
from .data_types import Command, DriveStatus
from .pdo import PdoScaling, RX_PDO_STRUCT, TX_PDO_STRUCT, pack_command, unpack_status


@dataclass(slots=True)
class NovantaVolcanoSlaveAdapter(Ds402SlaveAdapter):
    """Adapter that encapsulates DS402-specific cyclic PDO mapping for the Novanta Volcano."""

    scaling: PdoScaling = field(default_factory=PdoScaling)
    _startup_read_specs: dict[str, SdoReadSpec] = field(
        default_factory=lambda: {
            "torque_loop_max_output": SdoReadSpec(
                name="torque_loop_max_output", index=0x2527, subindex=0x00, data_type="f32"
            ),
            "torque_loop_min_output": SdoReadSpec(
                name="torque_loop_min_output", index=0x2528, subindex=0x00, data_type="f32"
            ),
            "velocity_loop_kp": SdoReadSpec(
                name="velocity_loop_kp", index=0x250A, subindex=0x00, data_type="f32"
            ),
            "velocity_loop_ki": SdoReadSpec(
                name="velocity_loop_ki", index=0x250B, subindex=0x00, data_type="f32"
            ),
            "velocity_loop_kd": SdoReadSpec(
                name="velocity_loop_kd", index=0x250C, subindex=0x00, data_type="f32"
            ),
            "position_loop_kp": SdoReadSpec(
                name="position_loop_kp", index=0x2511, subindex=0x00, data_type="f32"
            ),
            "position_loop_ki": SdoReadSpec(
                name="position_loop_ki", index=0x2512, subindex=0x00, data_type="f32"
            ),
            "position_loop_kd": SdoReadSpec(
                name="position_loop_kd", index=0x2513, subindex=0x00, data_type="f32"
            ),
            "motor_kt": SdoReadSpec(
                name="motor_kt", index=0x243B, subindex=0x00, data_type="f32"
            ),
        }
    )

    @property
    def rx_pdo_size(self) -> int:
        return RX_PDO_STRUCT.size

    @property
    def tx_pdo_size(self) -> int:
        return TX_PDO_STRUCT.size

    def pack_rx_pdo(self, command: Command) -> bytes:
        return pack_command(command, self.scaling, self._last_status_word)

    def unpack_tx_pdo(
        self,
        pdo: bytes,
        *,
        seq: int = 0,
        stamp_ns: int = 0,
        cycle_time_ns: int = 0,
        dc_time_error_ns: int = 0,
    ) -> DriveStatus:
        status = unpack_status(
            pdo,
            self.scaling,
            seq=seq,
            stamp_ns=stamp_ns,
            cycle_time_ns=cycle_time_ns,
            dc_time_error_ns=dc_time_error_ns,
        )
        self._last_status_word = status.status_word
        return status

    def startup_read_specs(self) -> dict[str, SdoReadSpec]:
        """Named NovantaVolcano startup SDOs available for pre-remap readout."""
        return dict(self._startup_read_specs)
