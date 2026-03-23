"""Beckhoff EL2004 command/status data model."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class El2004Command:
    """Command for a 4-channel digital output terminal."""

    output_1: bool = False
    output_2: bool = False
    output_3: bool = False
    output_4: bool = False


@dataclass(slots=True)
class El2004Status:
    """Observed EL2004 process-data state."""

    output_byte: int = 0

