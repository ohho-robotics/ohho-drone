"""Data schemas for drone kinematics, flight modes, and telemetry.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
Compatible with OhhO OS core data types (ohho.schema).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum

try:
    from ohho.schema import (
        ConnectionState,
        Odometry,
        Telemetry,
        TransportStatus,
        Velocity,
        clamp,
    )
except ImportError:
    # Standalone fallbacks if ohho-os is not installed in the environment
    class ConnectionState(str, Enum):  # type: ignore[no-redef]
        IDLE = "idle"
        CONNECTING = "connecting"
        CONNECTED = "connected"
        RECONNECTING = "reconnecting"
        ERROR = "error"
        DISCONNECTED = "disconnected"

    @dataclass
    class Velocity:  # type: ignore[no-redef]
        linear_x: float = 0.0
        linear_y: float = 0.0
        angular_z: float = 0.0

    @dataclass
    class Odometry:  # type: ignore[no-redef]
        x: float = 0.0
        y: float = 0.0
        theta: float = 0.0
        vx: float = 0.0
        vy: float = 0.0
        omega: float = 0.0

    @dataclass
    class Telemetry:  # type: ignore[no-redef]
        odom: Odometry | None = None
        joints: list = field(default_factory=list)
        battery: float | None = None
        scan: object | None = None
        custom: dict = field(default_factory=dict)
        timestamp: float = field(default_factory=time.time)

    @dataclass
    class TransportStatus:  # type: ignore[no-redef]
        protocol: str
        state: ConnectionState
        label: str = ""
        latency_ms: float = 0.0
        msg_rate: float = 0.0
        error: str | None = None
        connected_since: float | None = None

    def clamp(value: float, lo: float, hi: float) -> float:  # type: ignore[no-redef]
        return max(lo, min(hi, value))


class FlightMode(str, Enum):
    """Drone flight modes (aligned with PX4 autopilot modes)."""

    DISARMED = "disarmed"
    MANUAL = "manual"
    STABILIZED = "stabilized"
    ALTITUDE = "altitude"
    POSITION = "position"
    OFFBOARD = "offboard"
    TAKEOFF = "takeoff"
    LAND = "land"
    RTL = "rtl"
    HOLD = "hold"
    UNKNOWN = "unknown"


@dataclass
class DroneVelocity:
    """3D body/local velocity command for aerial platforms.

    linear_x: forward velocity (m/s)
    linear_y: rightward velocity (m/s)
    linear_z: upward climb rate (m/s, positive = climb, negative = descent)
    yaw_rate: angular rate about vertical axis (rad/s, positive = clockwise / right)
    """

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    yaw_rate: float = 0.0

    def to_ohho_velocity(self) -> Velocity:
        """Project onto planar OhhO base velocity."""
        return Velocity(
            linear_x=self.linear_x,
            linear_y=self.linear_y,
            angular_z=self.yaw_rate,
        )

    @classmethod
    def from_ohho_velocity(cls, vel: Velocity, linear_z: float = 0.0) -> DroneVelocity:
        return cls(
            linear_x=vel.linear_x,
            linear_y=vel.linear_y,
            linear_z=linear_z,
            yaw_rate=vel.angular_z,
        )


@dataclass
class DronePose:
    """6-DOF pose in local coordinate frame (NED or ENU)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class GlobalCoordinates:
    """WGS84 global coordinates from GNSS."""

    lat: float = 0.0  # degrees
    lon: float = 0.0  # degrees
    alt_amsl: float = 0.0  # metres above mean sea level
    relative_alt: float = 0.0  # metres above takeoff/home point


@dataclass
class DroneTelemetry:
    """Comprehensive drone state snapshot.

    Extends and translates into OhhO OS core Telemetry.
    """

    armed: bool = False
    flight_mode: FlightMode = FlightMode.DISARMED
    # MAV_LANDED_STATE, or None when EXTENDED_SYS_STATE has not been seen.
    landed_state: int | None = None
    odom: Odometry | None = None
    pose: DronePose | None = None
    velocity: DroneVelocity = field(default_factory=DroneVelocity)
    global_pos: GlobalCoordinates | None = None
    battery: float | None = None  # 0.0 .. 1.0 fraction
    voltage_v: float = 0.0
    current_a: float = 0.0
    satellites_visible: int = 0
    gps_fix_type: int = 0  # 0=None, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
    custom: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_ohho_telemetry(self) -> Telemetry:
        """Convert into standard OhhO OS Telemetry dataclass."""
        custom_data = dict(self.custom)
        custom_data.update(
            {
                "drone": {
                    "armed": self.armed,
                    "flight_mode": self.flight_mode.value,
                    "voltage_v": self.voltage_v,
                    "current_a": self.current_a,
                    "satellites": self.satellites_visible,
                    "gps_fix_type": self.gps_fix_type,
                    "landed_state": self.landed_state,
                    "experimental": True,
                    "flight_ready": False,
                }
            }
        )
        if self.global_pos:
            custom_data["drone"]["global_pos"] = {
                "lat": self.global_pos.lat,
                "lon": self.global_pos.lon,
                "alt_amsl": self.global_pos.alt_amsl,
                "relative_alt": self.global_pos.relative_alt,
            }
        if self.pose:
            custom_data["drone"]["pose_6dof"] = {
                "x": self.pose.x,
                "y": self.pose.y,
                "z": self.pose.z,
                "roll": self.pose.roll,
                "pitch": self.pose.pitch,
                "yaw": self.pose.yaw,
            }

        return Telemetry(
            odom=self.odom,
            battery=self.battery,
            custom=custom_data,
            timestamp=self.timestamp,
        )


__all__ = [
    "ConnectionState",
    "DronePose",
    "DroneTelemetry",
    "DroneVelocity",
    "FlightMode",
    "GlobalCoordinates",
    "Odometry",
    "Telemetry",
    "TransportStatus",
    "Velocity",
    "clamp",
]
