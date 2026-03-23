"""Core-level shared types and compatibility exports."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict

from .devices.motor_drives.Novanta.Everest.data_types import (
    Command,
    DriveCiA402States,
    DriveStatus,
    ModeOfOperation,
)


class EthercatAlStates(IntEnum):
    """EtherCAT Application Layer (AL) states and AL error flag."""

    INIT = 0x01
    PRE_OPERATIONAL = 0x02
    BOOTSTRAP = 0x03
    SAFE_OPERATIONAL = 0x04
    OPERATIONAL = 0x08
    ERROR_FLAG = 0x10


@dataclass(slots=True)
class SystemCommand:
    """Per-cycle multi-slave command container keyed by configured slave name."""

    by_slave: Dict[str, Any] = field(default_factory=dict)
    seq: int = 0
    stamp_ns: int = 0


@dataclass(slots=True)
class SystemStatus:
    """Per-cycle multi-slave status container keyed by configured slave name."""

    by_slave: Dict[str, Any] = field(default_factory=dict)
    seq: int = 0
    stamp_ns: int = 0
