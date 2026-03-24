"""Beckhoff EL5032 slave module."""

from .adapter import El5032SlaveAdapter
from .data_types import El5032Command, El5032Data

__all__ = ["El5032Command", "El5032Data", "El5032SlaveAdapter"]
