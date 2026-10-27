"""Typed MAVLink message definitions and binary serialization.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar


def _pad(data: bytes, size: int) -> bytes:
    """Zero-fill a MAVLink v2 payload that omitted trailing zero bytes."""
    if len(data) >= size:
        return data
    return data + bytes(size - len(data))

# Standard MAVLink Commands
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400

# PX4 treats param2=21196 on MAV_CMD_COMPONENT_ARM_DISARM as force-disarm.
# This stack must never transmit that value.
MAV_FORCE_DISARM_MAGIC = 21196

# MAV_LANDED_STATE values carried by EXTENDED_SYS_STATE.landed_state
MAV_LANDED_STATE_UNDEFINED = 0
MAV_LANDED_STATE_ON_GROUND = 1
MAV_LANDED_STATE_IN_AIR = 2
MAV_LANDED_STATE_TAKEOFF = 3
MAV_LANDED_STATE_LANDING = 4

# PX4 custom_mode packing: (sub_mode << 24) | (main_mode << 16)
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3  # HOLD
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6


def px4_custom_mode(main_mode: int, sub_mode: int) -> int:
    """Pack a PX4 main mode and sub mode into custom_mode."""
    return ((sub_mode & 0xFF) << 24) | ((main_mode & 0xFF) << 16)


PX4_MODE_AUTO_LOITER = px4_custom_mode(
    PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER
)
PX4_MODE_AUTO_LAND = px4_custom_mode(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND)

# Command Ack Results
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4

# Base Mode Flags
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# Frames
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_NED = 8

# Autopilot Types
MAV_AUTOPILOT_GENERIC = 0
MAV_AUTOPILOT_PX4 = 12

# Vehicle Types
MAV_TYPE_GENERIC = 0
MAV_TYPE_QUADROTOR = 2


@dataclass
class HeartbeatMessage:
    """HEARTBEAT (msg ID 0). Indicates system presence and state."""

    msg_id: ClassVar[int] = 0
    crc_extra: ClassVar[int] = 50

    custom_mode: int = 0
    type: int = MAV_TYPE_QUADROTOR
    autopilot: int = MAV_AUTOPILOT_PX4
    base_mode: int = 0
    system_status: int = 4  # MAV_STATE_ACTIVE
    mavlink_version: int = 3

    @property
    def is_armed(self) -> bool:
        return bool(self.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

    def pack(self) -> bytes:
        return struct.pack(
            "<IBBBBB",
            self.custom_mode,
            self.type,
            self.autopilot,
            self.base_mode,
            self.system_status,
            self.mavlink_version,
        )

    @classmethod
    def unpack(cls, data: bytes) -> HeartbeatMessage:
        data = _pad(data, 9)
        (custom_mode, type_, autopilot, base_mode, system_status, mavlink_version) = (
            struct.unpack("<IBBBBB", data[:9])
        )
        return cls(
            custom_mode=custom_mode,
            type=type_,
            autopilot=autopilot,
            base_mode=base_mode,
            system_status=system_status,
            mavlink_version=mavlink_version,
        )


@dataclass
class AttitudeMessage:
    """ATTITUDE (msg ID 30). Vehicle Euler angles and angular rates."""

    msg_id: ClassVar[int] = 30
    crc_extra: ClassVar[int] = 39

    time_boot_ms: int = 0
    roll: float = 0.0  # radians
    pitch: float = 0.0  # radians
    yaw: float = 0.0  # radians
    rollspeed: float = 0.0  # rad/s
    pitchspeed: float = 0.0  # rad/s
    yawspeed: float = 0.0  # rad/s

    def pack(self) -> bytes:
        return struct.pack(
            "<Iffffff",
            self.time_boot_ms,
            self.roll,
            self.pitch,
            self.yaw,
            self.rollspeed,
            self.pitchspeed,
            self.yawspeed,
        )

    @classmethod
    def unpack(cls, data: bytes) -> AttitudeMessage:
        data = _pad(data, 28)
        fields = struct.unpack("<Iffffff", data[:28])
        return cls(*fields)


@dataclass
class LocalPositionNedMessage:
    """LOCAL_POSITION_NED (msg ID 32). Filtered local position in NED frame."""

    msg_id: ClassVar[int] = 32
    crc_extra: ClassVar[int] = 185

    time_boot_ms: int = 0
    x: float = 0.0  # North, metres
    y: float = 0.0  # East, metres
    z: float = 0.0  # Down, metres (negative = altitude)
    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s
    vz: float = 0.0  # m/s (positive = descent)

    def pack(self) -> bytes:
        return struct.pack(
            "<Iffffff",
            self.time_boot_ms,
            self.x,
            self.y,
            self.z,
            self.vx,
            self.vy,
            self.vz,
        )

    @classmethod
    def unpack(cls, data: bytes) -> LocalPositionNedMessage:
        data = _pad(data, 28)
        fields = struct.unpack("<Iffffff", data[:28])
        return cls(*fields)


@dataclass
class GlobalPositionIntMessage:
    """GLOBAL_POSITION_INT (msg ID 33). Fused GNSS global position."""

    msg_id: ClassVar[int] = 33
    crc_extra: ClassVar[int] = 104

    time_boot_ms: int = 0
    lat: int = 0  # Latitude, degE7
    lon: int = 0  # Longitude, degE7
    alt: int = 0  # Altitude AMSL, mm
    relative_alt: int = 0  # Altitude above ground, mm
    vx: int = 0  # Ground X velocity, cm/s
    vy: int = 0  # Ground Y velocity, cm/s
    vz: int = 0  # Ground Z velocity, cm/s
    hdg: int = 0  # Heading, cdeg (0..36000)

    @property
    def lat_deg(self) -> float:
        return self.lat / 1e7

    @property
    def lon_deg(self) -> float:
        return self.lon / 1e7

    @property
    def alt_m(self) -> float:
        return self.alt / 1e3

    @property
    def relative_alt_m(self) -> float:
        return self.relative_alt / 1e3

    def pack(self) -> bytes:
        return struct.pack(
            "<IiiiihhhH",
            self.time_boot_ms,
            self.lat,
            self.lon,
            self.alt,
            self.relative_alt,
            self.vx,
            self.vy,
            self.vz,
            self.hdg,
        )

    @classmethod
    def unpack(cls, data: bytes) -> GlobalPositionIntMessage:
        data = _pad(data, 28)
        fields = struct.unpack("<IiiiihhhH", data[:28])
        return cls(*fields)


@dataclass
class SysStatusMessage:
    """SYS_STATUS (msg ID 1). Battery and onboard hardware health."""

    msg_id: ClassVar[int] = 1
    crc_extra: ClassVar[int] = 124

    onboard_control_sensors_present: int = 0
    onboard_control_sensors_enabled: int = 0
    onboard_control_sensors_health: int = 0
    load: int = 0
    voltage_battery: int = 12600  # mV (3S LiPo ~ 12.6V)
    current_battery: int = 500  # cA (5.0A)
    battery_remaining: int = 95  # % (0..100); -1 means unknown
    drop_rate_comm: int = 0
    errors_comm: int = 0
    errors_count1: int = 0
    errors_count2: int = 0
    errors_count3: int = 0
    errors_count4: int = 0

    @property
    def voltage_v(self) -> float:
        return self.voltage_battery / 1000.0

    @property
    def current_a(self) -> float:
        return self.current_battery / 100.0

    def pack(self) -> bytes:
        # MAVLink v2 orders fields by size. int8 battery_remaining is last.
        return struct.pack(
            "<IIIHHhHHHHHHb",
            self.onboard_control_sensors_present,
            self.onboard_control_sensors_enabled,
            self.onboard_control_sensors_health,
            self.load,
            self.voltage_battery,
            self.current_battery,
            self.drop_rate_comm,
            self.errors_comm,
            self.errors_count1,
            self.errors_count2,
            self.errors_count3,
            self.errors_count4,
            self.battery_remaining,
        )

    @classmethod
    def unpack(cls, data: bytes) -> SysStatusMessage:
        data = _pad(data, 31)
        (
            present,
            enabled,
            health,
            load,
            voltage,
            current,
            drop,
            errs,
            e1,
            e2,
            e3,
            e4,
            remaining,
        ) = struct.unpack("<IIIHHhHHHHHHb", data[:31])
        return cls(
            onboard_control_sensors_present=present,
            onboard_control_sensors_enabled=enabled,
            onboard_control_sensors_health=health,
            load=load,
            voltage_battery=voltage,
            current_battery=current,
            battery_remaining=remaining,
            drop_rate_comm=drop,
            errors_comm=errs,
            errors_count1=e1,
            errors_count2=e2,
            errors_count3=e3,
            errors_count4=e4,
        )


@dataclass
class CommandLongMessage:
    """COMMAND_LONG (msg ID 76). Send commands to flight controller."""

    msg_id: ClassVar[int] = 76
    crc_extra: ClassVar[int] = 152

    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0
    param6: float = 0.0
    param7: float = 0.0
    command: int = 0
    target_system: int = 1
    target_component: int = 1
    confirmation: int = 0

    def pack(self) -> bytes:
        return struct.pack(
            "<fffffffHBBB",
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.param5,
            self.param6,
            self.param7,
            self.command,
            self.target_system,
            self.target_component,
            self.confirmation,
        )

    @classmethod
    def unpack(cls, data: bytes) -> CommandLongMessage:
        data = _pad(data, 33)
        fields = struct.unpack("<fffffffHBBB", data[:33])
        return cls(*fields)


@dataclass
class CommandAckMessage:
    """COMMAND_ACK (msg ID 77). Acknowledgment of a command."""

    msg_id: ClassVar[int] = 77
    crc_extra: ClassVar[int] = 143

    command: int = 0
    result: int = MAV_RESULT_ACCEPTED

    def pack(self) -> bytes:
        return struct.pack("<HB", self.command, self.result)

    @classmethod
    def unpack(cls, data: bytes) -> CommandAckMessage:
        data = _pad(data, 3)
        (command, result) = struct.unpack("<HB", data[:3])
        return cls(command=command, result=result)


@dataclass
class ExtendedSysStateMessage:
    """EXTENDED_SYS_STATE (msg ID 245). Reports landed state."""

    msg_id: ClassVar[int] = 245
    crc_extra: ClassVar[int] = 130

    vtol_state: int = 0  # MAV_VTOL_STATE_UNDEFINED
    landed_state: int = MAV_LANDED_STATE_UNDEFINED

    def pack(self) -> bytes:
        return struct.pack("<BB", self.vtol_state & 0xFF, self.landed_state & 0xFF)

    @classmethod
    def unpack(cls, data: bytes) -> ExtendedSysStateMessage:
        data = _pad(data, 2)
        vtol_state, landed_state = struct.unpack("<BB", data[:2])
        return cls(vtol_state=vtol_state, landed_state=landed_state)


@dataclass
class SetPositionTargetLocalNedMessage:
    """SET_POSITION_TARGET_LOCAL_NED (msg ID 84).

    Offboard setpoint (velocity, position, or acceleration).
    """

    msg_id: ClassVar[int] = 84
    crc_extra: ClassVar[int] = 143

    time_boot_ms: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    afx: float = 0.0
    afy: float = 0.0
    afz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    # Bitmask to ignore position & acceleration:
    # ignore x,y,z (bits 0,1,2), afx,afy,afz (bits 6,7,8), yaw (bit 10)
    # Mask = 0b0000_0101_1100_0111 = 0x05C7 (or 0x0FA7)
    type_mask: int = 0x05C7
    target_system: int = 1
    target_component: int = 1
    coordinate_frame: int = MAV_FRAME_LOCAL_NED

    def pack(self) -> bytes:
        return struct.pack(
            "<IfffffffffffHBBB",
            self.time_boot_ms,
            self.x,
            self.y,
            self.z,
            self.vx,
            self.vy,
            self.vz,
            self.afx,
            self.afy,
            self.afz,
            self.yaw,
            self.yaw_rate,
            self.type_mask,
            self.target_system,
            self.target_component,
            self.coordinate_frame,
        )

    @classmethod
    def unpack(cls, data: bytes) -> SetPositionTargetLocalNedMessage:
        data = _pad(data, 53)
        fields = struct.unpack("<IfffffffffffHBBB", data[:53])
        return cls(*fields)


# Mapping from msg_id to CRC extra byte
CRC_EXTRA_MAP: dict[int, int] = {
    HeartbeatMessage.msg_id: HeartbeatMessage.crc_extra,
    SysStatusMessage.msg_id: SysStatusMessage.crc_extra,
    AttitudeMessage.msg_id: AttitudeMessage.crc_extra,
    LocalPositionNedMessage.msg_id: LocalPositionNedMessage.crc_extra,
    GlobalPositionIntMessage.msg_id: GlobalPositionIntMessage.crc_extra,
    CommandLongMessage.msg_id: CommandLongMessage.crc_extra,
    CommandAckMessage.msg_id: CommandAckMessage.crc_extra,
    SetPositionTargetLocalNedMessage.msg_id: SetPositionTargetLocalNedMessage.crc_extra,
    ExtendedSysStateMessage.msg_id: ExtendedSysStateMessage.crc_extra,
}

__all__ = [
    "CRC_EXTRA_MAP",
    "AttitudeMessage",
    "CommandAckMessage",
    "CommandLongMessage",
    "ExtendedSysStateMessage",
    "GlobalPositionIntMessage",
    "HeartbeatMessage",
    "LocalPositionNedMessage",
    "MAV_AUTOPILOT_GENERIC",
    "MAV_AUTOPILOT_PX4",
    "MAV_CMD_COMPONENT_ARM_DISARM",
    "MAV_CMD_DO_SET_MODE",
    "MAV_CMD_NAV_LAND",
    "MAV_CMD_NAV_TAKEOFF",
    "MAV_FORCE_DISARM_MAGIC",
    "MAV_FRAME_BODY_NED",
    "MAV_FRAME_LOCAL_NED",
    "MAV_LANDED_STATE_IN_AIR",
    "MAV_LANDED_STATE_LANDING",
    "MAV_LANDED_STATE_ON_GROUND",
    "MAV_LANDED_STATE_TAKEOFF",
    "MAV_LANDED_STATE_UNDEFINED",
    "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
    "MAV_MODE_FLAG_SAFETY_ARMED",
    "MAV_RESULT_ACCEPTED",
    "MAV_RESULT_DENIED",
    "MAV_RESULT_FAILED",
    "MAV_RESULT_TEMPORARILY_REJECTED",
    "MAV_RESULT_UNSUPPORTED",
    "MAV_TYPE_GENERIC",
    "MAV_TYPE_QUADROTOR",
    "PX4_CUSTOM_MAIN_MODE_AUTO",
    "PX4_CUSTOM_SUB_MODE_AUTO_LAND",
    "PX4_CUSTOM_SUB_MODE_AUTO_LOITER",
    "PX4_MODE_AUTO_LAND",
    "PX4_MODE_AUTO_LOITER",
    "SetPositionTargetLocalNedMessage",
    "SysStatusMessage",
    "px4_custom_mode",
]
