"""Compatibility wrapper for DS402 PDO helpers.

For new multi-slave code, import from `ethercat_core.devices.<family>.pdo`.
"""

from .devices.ds402.pdo import (
    PdoScaling,
    RX_PDO_STRUCT,
    TX_PDO_STRUCT,
    decode_cia402_state,
    pack_command,
    unpack_status,
)

__all__ = [
    "PdoScaling",
    "RX_PDO_STRUCT",
    "TX_PDO_STRUCT",
    "decode_cia402_state",
    "pack_command",
    "unpack_status",
]
