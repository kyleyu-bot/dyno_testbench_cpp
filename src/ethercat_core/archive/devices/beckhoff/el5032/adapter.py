"""Beckhoff EL5032 slave adapter implementation."""

from __future__ import annotations

from dataclasses import dataclass

from ...base import SlaveIdentity
from .data_types import (
    EL5032_ENCODER_VALUE_MASK_25BIT,
    EL5032_TX_PDO_SIZE,
    El5032Command,
    El5032Data,
)


@dataclass(slots=True)
class El5032SlaveAdapter:
    """Adapter for Beckhoff EL5032 encoder data mapped from 0x6000:11."""

    identity: SlaveIdentity

    @property
    def rx_pdo_size(self) -> int:
        return 0

    @property
    def tx_pdo_size(self) -> int:
        return EL5032_TX_PDO_SIZE

    def pack_rx_pdo(self, command: El5032Command) -> bytes:
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
    ) -> El5032Data:
        del seq, stamp_ns, cycle_time_ns, dc_time_error_ns

        encoder_value_raw = 0
        encoder_count_25bit = 0
        if len(pdo) >= EL5032_TX_PDO_SIZE:
            encoder_value_raw = int.from_bytes(
                pdo[:EL5032_TX_PDO_SIZE], "little", signed=True
            )
            unsigned_encoder_value = int.from_bytes(
                pdo[:EL5032_TX_PDO_SIZE], "little", signed=False
            )
            encoder_count_25bit = (
                # unsigned_encoder_value >> (EL5032_TX_PDO_SIZE * 8 - 25)
                unsigned_encoder_value >> (2 * 8)
            ) & EL5032_ENCODER_VALUE_MASK_25BIT

        return El5032Data(
            encoder_value_raw=encoder_value_raw,
            encoder_count_25bit=encoder_count_25bit,
            raw_pdo=bytes(pdo),
        )

    def get_encoder_count_25bit(self, data: El5032Data) -> int:
        return data.encoder_count_25bit
