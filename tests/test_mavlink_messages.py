"""Unit tests for typed MAVLink messages and stream codec."""

from __future__ import annotations

import struct
import unittest

from ohho_drone.mavlink.codec import MavlinkCodec
from ohho_drone.mavlink.messages import (
    MAV_AUTOPILOT_PX4,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_BODY_NED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_RESULT_ACCEPTED,
    MAV_TYPE_QUADROTOR,
    AttitudeMessage,
    CommandAckMessage,
    CommandLongMessage,
    ExtendedSysStateMessage,
    GlobalPositionIntMessage,
    HeartbeatMessage,
    LocalPositionNedMessage,
    SetPositionTargetLocalNedMessage,
    SysStatusMessage,
)
from ohho_drone.mavlink.packet import MavlinkPacket


class TestMavlinkMessages(unittest.TestCase):
    def test_heartbeat_pack_unpack(self) -> None:
        hb = HeartbeatMessage(
            custom_mode=4,
            type=MAV_TYPE_QUADROTOR,
            autopilot=MAV_AUTOPILOT_PX4,
            base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
            system_status=4,
            mavlink_version=3,
        )
        self.assertTrue(hb.is_armed)

        packed = hb.pack()
        self.assertEqual(len(packed), 9)

        unpacked = HeartbeatMessage.unpack(packed)
        self.assertEqual(unpacked.custom_mode, 4)
        self.assertEqual(unpacked.type, MAV_TYPE_QUADROTOR)
        self.assertEqual(unpacked.autopilot, MAV_AUTOPILOT_PX4)
        self.assertTrue(unpacked.is_armed)
        self.assertEqual(unpacked.system_status, 4)

    def test_attitude_pack_unpack(self) -> None:
        att = AttitudeMessage(
            time_boot_ms=1024,
            roll=0.12,
            pitch=-0.05,
            yaw=1.57,
            rollspeed=0.01,
            pitchspeed=-0.02,
            yawspeed=0.03,
        )
        packed = att.pack()
        self.assertEqual(len(packed), 28)

        unpacked = AttitudeMessage.unpack(packed)
        self.assertEqual(unpacked.time_boot_ms, 1024)
        self.assertAlmostEqual(unpacked.roll, 0.12, places=5)
        self.assertAlmostEqual(unpacked.pitch, -0.05, places=5)
        self.assertAlmostEqual(unpacked.yaw, 1.57, places=5)
        self.assertAlmostEqual(unpacked.yawspeed, 0.03, places=5)

    def test_local_position_ned_pack_unpack(self) -> None:
        pos = LocalPositionNedMessage(
            time_boot_ms=500,
            x=12.5,
            y=-3.2,
            z=-2.5,  # 2.5m altitude
            vx=1.0,
            vy=0.5,
            vz=-0.2,
        )
        packed = pos.pack()
        self.assertEqual(len(packed), 28)

        unpacked = LocalPositionNedMessage.unpack(packed)
        self.assertAlmostEqual(unpacked.x, 12.5, places=5)
        self.assertAlmostEqual(unpacked.y, -3.2, places=5)
        self.assertAlmostEqual(unpacked.z, -2.5, places=5)
        self.assertAlmostEqual(unpacked.vx, 1.0, places=5)

    def test_global_position_int_pack_unpack(self) -> None:
        gps = GlobalPositionIntMessage(
            time_boot_ms=2000,
            lat=473977420,  # 47.3977420
            lon=85455940,  # 8.5455940
            alt=488000,  # 488.0m AMSL
            relative_alt=15000,  # 15.0m
            vx=100,  # 1.0 m/s
            vy=0,
            vz=-50,
            hdg=9000,  # 90.0 deg
        )
        self.assertAlmostEqual(gps.lat_deg, 47.397742, places=6)
        self.assertAlmostEqual(gps.lon_deg, 8.545594, places=6)
        self.assertAlmostEqual(gps.alt_m, 488.0, places=2)
        self.assertAlmostEqual(gps.relative_alt_m, 15.0, places=2)

        packed = gps.pack()
        self.assertEqual(len(packed), 28)
        unpacked = GlobalPositionIntMessage.unpack(packed)
        self.assertEqual(unpacked.lat, 473977420)
        self.assertEqual(unpacked.relative_alt, 15000)

    def test_sys_status_pack_unpack(self) -> None:
        status = SysStatusMessage(
            voltage_battery=12600,
            current_battery=450,
            battery_remaining=92,
        )
        self.assertEqual(status.voltage_v, 12.6)
        self.assertEqual(status.current_a, 4.5)

        packed = status.pack()
        self.assertEqual(len(packed), 31)
        unpacked = SysStatusMessage.unpack(packed)
        self.assertEqual(unpacked.voltage_battery, 12600)
        self.assertEqual(unpacked.current_battery, 450)
        self.assertEqual(unpacked.battery_remaining, 92)

    def test_sys_status_wire_order_puts_signed_battery_last(self) -> None:
        """battery_remaining is the final int8, after errors_count4."""
        status = SysStatusMessage(
            onboard_control_sensors_present=1,
            onboard_control_sensors_enabled=2,
            onboard_control_sensors_health=3,
            load=4,
            voltage_battery=12600,
            current_battery=450,
            battery_remaining=-1,
            drop_rate_comm=5,
            errors_comm=6,
            errors_count1=7,
            errors_count2=8,
            errors_count3=9,
            errors_count4=10,
        )
        packed = status.pack()
        fields = struct.unpack("<IIIHHhHHHHHHb", packed)
        self.assertEqual(fields[6], 5)
        self.assertEqual(fields[11], 10)
        self.assertEqual(fields[12], -1)
        unpacked = SysStatusMessage.unpack(packed)
        self.assertEqual(unpacked.drop_rate_comm, 5)
        self.assertEqual(unpacked.errors_count4, 10)
        self.assertEqual(unpacked.battery_remaining, -1)
        truncated = SysStatusMessage.unpack(packed[:-1])
        self.assertEqual(truncated.battery_remaining, 0)
        self.assertEqual(truncated.errors_count4, 10)

    def test_command_long_and_ack(self) -> None:
        cmd = CommandLongMessage(
            command=MAV_CMD_NAV_TAKEOFF,
            param7=5.0,
            target_system=1,
            target_component=1,
        )
        packed_cmd = cmd.pack()
        self.assertEqual(len(packed_cmd), 33)
        unpacked_cmd = CommandLongMessage.unpack(packed_cmd)
        self.assertEqual(unpacked_cmd.command, MAV_CMD_NAV_TAKEOFF)
        self.assertAlmostEqual(unpacked_cmd.param7, 5.0, places=4)

        ack = CommandAckMessage(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            result=MAV_RESULT_ACCEPTED,
        )
        packed_ack = ack.pack()
        self.assertEqual(len(packed_ack), 3)
        unpacked_ack = CommandAckMessage.unpack(packed_ack)
        self.assertEqual(unpacked_ack.command, MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(unpacked_ack.result, MAV_RESULT_ACCEPTED)

    def test_set_position_target_local_ned(self) -> None:
        target = SetPositionTargetLocalNedMessage(
            time_boot_ms=100,
            vx=1.5,
            vy=-0.5,
            vz=-1.0,
            yaw_rate=0.2,
            type_mask=0x05C7,
            target_system=1,
            target_component=1,
            coordinate_frame=MAV_FRAME_BODY_NED,
        )
        packed = target.pack()
        self.assertEqual(len(packed), 53)
        unpacked = SetPositionTargetLocalNedMessage.unpack(packed)
        self.assertAlmostEqual(unpacked.vx, 1.5, places=4)
        self.assertAlmostEqual(unpacked.vy, -0.5, places=4)
        self.assertAlmostEqual(unpacked.vz, -1.0, places=4)
        self.assertEqual(unpacked.type_mask, 0x05C7)
        self.assertEqual(unpacked.coordinate_frame, MAV_FRAME_BODY_NED)

    def test_codec_encode_and_feed(self) -> None:
        codec = MavlinkCodec(sysid=255, compid=191)
        hb = HeartbeatMessage(base_mode=MAV_MODE_FLAG_SAFETY_ARMED)
        encoded = codec.encode(hb)

        # Feed to codec
        stream_codec = MavlinkCodec()
        results = stream_codec.feed(encoded)
        self.assertEqual(len(results), 1)
        pkt, decoded = results[0]
        self.assertEqual(pkt.sysid, 255)
        self.assertEqual(pkt.compid, 191)
        self.assertIsInstance(decoded, HeartbeatMessage)
        assert isinstance(decoded, HeartbeatMessage)
        self.assertTrue(decoded.is_armed)

    def test_truncated_v2_payload_still_decodes(self) -> None:
        """Trailing zero bytes omitted by a v2 sender still unpack."""
        short = MavlinkPacket(
            msgid=ExtendedSysStateMessage.msg_id,
            payload=b"\x00",
        )
        raw = short.pack(crc_extra=ExtendedSysStateMessage.crc_extra)
        results = MavlinkCodec().feed(raw)
        self.assertEqual(len(results), 1)
        decoded = results[0][1]
        self.assertIsInstance(decoded, ExtendedSysStateMessage)
        assert isinstance(decoded, ExtendedSysStateMessage)
        self.assertEqual(decoded.vtol_state, 0)
        self.assertEqual(decoded.landed_state, 0)

        position = LocalPositionNedMessage(
            time_boot_ms=10, x=1.5, y=-2.0, z=0.0, vx=0.0, vy=0.0, vz=0.0
        )
        full = position.pack()
        trimmed = full[:12]
        unpacked = LocalPositionNedMessage.unpack(trimmed)
        self.assertAlmostEqual(unpacked.x, 1.5, places=5)
        self.assertAlmostEqual(unpacked.y, -2.0, places=5)
        self.assertEqual(unpacked.z, 0.0)
        self.assertEqual(unpacked.vz, 0.0)


if __name__ == "__main__":
    unittest.main()
