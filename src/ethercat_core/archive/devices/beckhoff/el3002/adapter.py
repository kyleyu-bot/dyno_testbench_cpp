"""Beckhoff ELM3002 slave adapter implementation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ...base import SlaveIdentity
from .data_types import (
    EL3002_TX_PDO_FIELDS,
    EL3002_TX_PDO_SIZE,
    El3002Command,
    El3002Data,
    Elm3002PaiStatus,
    decode_pai_status,
)

EL3002_ALLOWED_TORQUE_SCALES = (20.0, 200.0, 500.0)


@dataclass(slots=True)
class El3002SlaveAdapter:
    """Adapter for Beckhoff ELM3002 2-channel analog input terminal (AI Oversampling)."""

    identity: SlaveIdentity
    pai_samples_1_torque_scale: float = 200.0
    pai_samples_2_torque_scale: float = 20.0

    def __post_init__(self) -> None:
        self.pai_samples_1_torque_scale = self._validate_torque_scale(
            self.pai_samples_1_torque_scale
        )
        self.pai_samples_2_torque_scale = self._validate_torque_scale(
            self.pai_samples_2_torque_scale
        )

    @property
    def rx_pdo_size(self) -> int:
        # ELM3002 is input-only.
        return 0

    @property
    def tx_pdo_size(self) -> int:
        return EL3002_TX_PDO_SIZE

    def pack_rx_pdo(self, command: El3002Command) -> bytes:
        del command
        return b""

    def unpack_tx_pdo(
        self,
        pdo: bytes,
        *,
        seq: int = 0,
        stamp_ns: int = 0,
        cycle_time_ns: int = 0,
        dc_time_error_ns: int = 0,
    ) -> El3002Data:
        del seq, stamp_ns, cycle_time_ns, dc_time_error_ns

        values: dict[str, Any] = {field.name: 0 for field in EL3002_TX_PDO_FIELDS}
        for field in EL3002_TX_PDO_FIELDS:
            field_end = field.offset + field.size
            if len(pdo) < field_end:
                continue
            values[field.name] = int.from_bytes(
                pdo[field.offset:field_end],
                byteorder="little",
                signed=field.signed,
            )

        return El3002Data(raw_pdo=bytes(pdo), **values)

    def _get_field_bytes(self, data: El3002Data, field_name: str) -> bytes:
        field = next(field for field in EL3002_TX_PDO_FIELDS if field.name == field_name)
        field_end = field.offset + field.size
        if len(data.raw_pdo) < field_end:
            return b""
        return data.raw_pdo[field.offset:field_end]

    # --- Channel 1 ---

    def get_pai_status_1_raw(self, data: El3002Data) -> int:
        return data.pai_status_1

    def get_pai_status_1(self, data: El3002Data) -> Elm3002PaiStatus:
        return decode_pai_status(data.pai_status_1)

    def get_pai_samples_1(self, data: El3002Data) -> int:
        return data.pai_samples_1

    def get_pai_samples_1_raw(self, data: El3002Data) -> bytes:
        return self._get_field_bytes(data, "pai_samples_1")

    def get_pai_samples_1_scaled_voltage(self, data: El3002Data) -> float:
        return self._scale_adc_to_voltage(data.pai_samples_1)

    def get_pai_samples_1_scaled_torque(self, data: El3002Data) -> float:
        return self._scale_adc(data.pai_samples_1) * self.pai_samples_1_torque_scale

    def set_pai_samples_1_torque_scale(self, scale: float) -> None:
        self.pai_samples_1_torque_scale = self._validate_torque_scale(scale)

    # --- Channel 2 ---

    def get_pai_status_2_raw(self, data: El3002Data) -> int:
        return data.pai_status_2

    def get_pai_status_2(self, data: El3002Data) -> Elm3002PaiStatus:
        return decode_pai_status(data.pai_status_2)

    def get_pai_samples_2(self, data: El3002Data) -> int:
        return data.pai_samples_2

    def get_pai_samples_2_raw(self, data: El3002Data) -> bytes:
        return self._get_field_bytes(data, "pai_samples_2")

    def get_pai_samples_2_scaled_voltage(self, data: El3002Data) -> float:
        return self._scale_adc_to_voltage(data.pai_samples_2)

    def get_pai_samples_2_scaled_torque(self, data: El3002Data) -> float:
        return self._scale_adc(data.pai_samples_2) * self.pai_samples_2_torque_scale

    def set_pai_samples_2_torque_scale(self, scale: float) -> None:
        self.pai_samples_2_torque_scale = self._validate_torque_scale(scale)

    # --- Timestamp ---

    def get_timestamp(self, data: El3002Data) -> int:
        return data.timestamp

    # --- Helpers ---

    @staticmethod
    def _scale_adc(sample: int) -> float:
        """Normalize a 24-bit signed ADC INT32 value to [-1.0, 1.0]."""
        return sample / float(1 << 23)

    @staticmethod
    def _scale_adc_to_voltage(sample: int) -> float:
        """Convert a 24-bit signed ADC INT32 value to volts (±5 V full scale)."""
        return (sample / float(1 << 23)) * 5.0

    @staticmethod
    def _validate_torque_scale(scale: float) -> float:
        scale = float(scale)
        if scale not in EL3002_ALLOWED_TORQUE_SCALES:
            raise ValueError(
                f"Unsupported ELM3002 torque scale {scale}. "
                f"Allowed values: {EL3002_ALLOWED_TORQUE_SCALES}"
            )
        return scale
