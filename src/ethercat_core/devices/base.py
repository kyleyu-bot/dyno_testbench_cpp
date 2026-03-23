"""Base interfaces for EtherCAT slave-specific adapters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Protocol, TypeVar

CommandT = TypeVar("CommandT")
StatusT = TypeVar("StatusT")


@dataclass(slots=True)
class SlaveIdentity:
    """Static identity and addressing info for one configured slave."""

    name: str
    position: int
    vendor_id: int = 0
    product_code: int = 0


@dataclass(slots=True)
class SdoReadSpec:
    """Typed SDO object read specification."""

    name: str
    index: int
    subindex: int = 0
    data_type: Literal["u8", "s8", "u16", "s16", "u32", "s32", "f32", "bytes"] = "bytes"


class SlaveAdapter(Protocol[CommandT, StatusT]):
    """Contract each slave type implements for cyclic PDO exchange."""

    identity: SlaveIdentity

    @property
    def rx_pdo_size(self) -> int:
        """Byte size for master->slave cyclic payload."""

    @property
    def tx_pdo_size(self) -> int:
        """Byte size for slave->master cyclic payload."""

    def pack_rx_pdo(self, command: CommandT) -> bytes:
        """Encode application command into slave-specific RX PDO bytes."""

    def unpack_tx_pdo(
        self,
        pdo: bytes,
        *,
        seq: int = 0,
        stamp_ns: int = 0,
        cycle_time_ns: int = 0,
        dc_time_error_ns: int = 0,
    ) -> StatusT:
        """Decode slave-specific TX PDO bytes into typed status."""
