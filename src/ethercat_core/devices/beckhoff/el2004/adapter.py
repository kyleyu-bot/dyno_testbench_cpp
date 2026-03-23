"""Beckhoff EL2004 slave adapter implementation."""

from __future__ import annotations

from dataclasses import dataclass

from ...base import SlaveIdentity
from .data_types import El2004Command, El2004Status


@dataclass(slots=True)
class El2004SlaveAdapter:
    """Adapter for Beckhoff EL2004 4-channel digital output terminal."""

    identity: SlaveIdentity

    @property
    def rx_pdo_size(self) -> int:
        # Four output channels are packed into one byte.
        return 1

    @property
    def tx_pdo_size(self) -> int:
        # This terminal is output-only in the current model.
        return 0

    def pack_rx_pdo(self, command: El2004Command) -> bytes:
        value = 0
        if command.output_1:
            value |= 0x01
        if command.output_2:
            value |= 0x02
        if command.output_3:
            value |= 0x04
        if command.output_4:
            value |= 0x08
        return bytes((value,))

    def unpack_tx_pdo(
        self,
        pdo: bytes,
        *,
        seq: int = 0,
        stamp_ns: int = 0,
        cycle_time_ns: int = 0,
        dc_time_error_ns: int = 0,
    ) -> El2004Status:
        del seq, stamp_ns, cycle_time_ns, dc_time_error_ns
        return El2004Status(output_byte=pdo[0] if pdo else 0)
