"""DS402 slave adapter base class."""

from __future__ import annotations

from dataclasses import dataclass

from ....base import SdoReadSpec, SlaveIdentity


@dataclass(slots=True)
class Ds402SlaveAdapter:
    """Base class for DS402 cyclic PDO adapters. Subclasses implement the PDO layout."""

    identity: SlaveIdentity
    _last_status_word: int = 0

    @property
    def rx_pdo_size(self) -> int:
        raise NotImplementedError

    @property
    def tx_pdo_size(self) -> int:
        raise NotImplementedError

    def pack_rx_pdo(self, command: object) -> bytes:
        del command
        raise NotImplementedError

    def unpack_tx_pdo(
        self,
        pdo: bytes,
        *,
        seq: int = 0,
        stamp_ns: int = 0,
        cycle_time_ns: int = 0,
        dc_time_error_ns: int = 0,
    ) -> object:
        del pdo, seq, stamp_ns, cycle_time_ns, dc_time_error_ns
        raise NotImplementedError

    def startup_read_specs(self) -> dict[str, SdoReadSpec]:
        return {}
