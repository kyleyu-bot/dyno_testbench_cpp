"""Beckhoff EL5032 command/data model."""

from __future__ import annotations

from dataclasses import dataclass, field


EL5032_ENCODER_VALUE_PDO_INDEX = 0x1A00
EL5032_ENCODER_VALUE_REGISTER_INDEX = 0x6000
EL5032_ENCODER_VALUE_SUBINDEX = 0x11
EL5032_ENCODER_VALUE_MASK_25BIT = (1 << 25) - 1
EL5032_TX_PDO_SIZE = 10


@dataclass(slots=True)
class El5032Command:
    """Placeholder command model for Beckhoff EL5032."""


@dataclass(slots=True)
class El5032Data:
    """Observed EL5032 TX PDO data for object 0x6000:11."""

    encoder_value_raw: int = 0
    encoder_count_25bit: int = 0
    raw_pdo: bytes = field(default_factory=bytes)
