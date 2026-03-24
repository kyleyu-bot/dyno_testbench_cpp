"""Beckhoff EL2004 slave module."""

from .adapter import El2004SlaveAdapter
from .data_types import El2004Command, El2004Status

__all__ = ["El2004Command", "El2004Status", "El2004SlaveAdapter"]

