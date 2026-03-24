"""CiA 402 (DS402) command/status data model."""

from __future__ import annotations

from enum import IntEnum
from typing import Protocol


class ModeOfOperation(IntEnum):
    """CiA 402 mode of operation (0x6060) values."""

    NO_MODE = 0
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 2
    PROFILE_TORQUE = 4
    CYCLIC_SYNC_POSITION = 8
    CYCLIC_SYNC_VELOCITY = 9
    CYCLIC_SYNC_TORQUE = 10


class DriveCiA402States(IntEnum):
    """
    CiA 402 drive-side state machine logical states.

    These are symbolic logical states, not raw statusword bitmasks.
    """

    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7


class DriveCommandBase(Protocol):
    """Minimal interface that DS402 controlword logic requires from any command type."""

    enable_drive: bool
    clear_fault: bool
