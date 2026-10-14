"""MAVLink framing, messages, and codec subpackage.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
"""

from __future__ import annotations

from .codec import MavlinkCodec
from .messages import (
    CRC_EXTRA_MAP,
    MAV_AUTOPILOT_GENERIC,
    MAV_AUTOPILOT_PX4,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_BODY_NED,
    MAV_FRAME_LOCAL_NED,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_FAILED,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MAV_RESULT_UNSUPPORTED,
    MAV_TYPE_GENERIC,
    MAV_TYPE_QUADROTOR,
    AttitudeMessage,
    CommandAckMessage,
    CommandLongMessage,
    GlobalPositionIntMessage,
    HeartbeatMessage,
    LocalPositionNedMessage,
    SetPositionTargetLocalNedMessage,
    SysStatusMessage,
)
from .packet import MAVLINK_V1_STX, MAVLINK_V2_STX, MavlinkPacket, mavlink_x25_crc

__all__ = [
    "CRC_EXTRA_MAP",
    "MAVLINK_V1_STX",
    "MAVLINK_V2_STX",
    "MAV_AUTOPILOT_GENERIC",
    "MAV_AUTOPILOT_PX4",
    "MAV_CMD_COMPONENT_ARM_DISARM",
    "MAV_CMD_DO_SET_MODE",
    "MAV_CMD_NAV_LAND",
    "MAV_CMD_NAV_TAKEOFF",
    "MAV_FRAME_BODY_NED",
    "MAV_FRAME_LOCAL_NED",
    "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
    "MAV_MODE_FLAG_SAFETY_ARMED",
    "MAV_RESULT_ACCEPTED",
    "MAV_RESULT_DENIED",
    "MAV_RESULT_FAILED",
    "MAV_RESULT_TEMPORARILY_REJECTED",
    "MAV_RESULT_UNSUPPORTED",
    "MAV_TYPE_GENERIC",
    "MAV_TYPE_QUADROTOR",
    "AttitudeMessage",
    "CommandAckMessage",
    "CommandLongMessage",
    "GlobalPositionIntMessage",
    "HeartbeatMessage",
    "LocalPositionNedMessage",
    "MavlinkCodec",
    "MavlinkPacket",
    "SetPositionTargetLocalNedMessage",
    "SysStatusMessage",
    "mavlink_x25_crc",
]
