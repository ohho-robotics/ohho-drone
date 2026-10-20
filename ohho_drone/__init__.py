"""OhhO Drone — Experimental Drone & MAVLink Adapter Prototype.

==============================================================================
WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SOFTWARE SIMULATION ONLY
DO NOT USE FOR REAL AIRCRAFT FLIGHT OR PHYSICAL HARDWARE CONTROL.
==============================================================================

This package provides an experimental sketch and prototype adapter connecting
drone flight controllers (such as PX4 running in SITL simulation) via the
MAVLink protocol to OhhO OS transport and telemetry interfaces.
"""

from __future__ import annotations

from .adapter import MavlinkDroneTransport
from .schema import (
    DronePose,
    DroneTelemetry,
    DroneVelocity,
    FlightMode,
    GlobalCoordinates,
)
from .sitl.mock_server import MockPx4SitlServer

__version__ = "0.0.0-experimental"
__all__ = [
    "DronePose",
    "DroneTelemetry",
    "DroneVelocity",
    "FlightMode",
    "GlobalCoordinates",
    "MavlinkDroneTransport",
    "MockPx4SitlServer",
    "__version__",
]
