"""Slave adapter modules for different EtherCAT device families."""

from .base import SdoReadSpec, SlaveAdapter, SlaveIdentity

__all__ = ["SlaveIdentity", "SdoReadSpec", "SlaveAdapter"]
