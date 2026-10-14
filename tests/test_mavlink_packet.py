"""Unit tests for MAVLink packet framing and CRC-16 calculation."""

from __future__ import annotations

import unittest

from ohho_drone.mavlink.packet import (
    MAVLINK_V1_STX,
    MAVLINK_V2_STX,
    MavlinkPacket,
    mavlink_x25_crc,
)


class TestMavlinkPacket(unittest.TestCase):
    def test_x25_crc_known_vector(self) -> None:
        """Verify MAVLink X.25 CRC-16 against known vector '123456789'."""
        crc = mavlink_x25_crc(b"123456789")
        self.assertEqual(crc, 0x6F91)

    def test_v2_pack_and_parse(self) -> None:
        """Test packing a MAVLink v2 packet and parsing it back."""
        payload = b"HelloPX4"
        pkt = MavlinkPacket(
            magic=MAVLINK_V2_STX,
            seq=42,
            sysid=1,
            compid=1,
            msgid=30,
            payload=payload,
        )
        raw = pkt.pack(crc_extra=39)

        # First byte must be 0xFD (MAVLink v2)
        self.assertEqual(raw[0], MAVLINK_V2_STX)
        self.assertEqual(raw[1], len(payload))
        self.assertEqual(raw[4], 42)
        self.assertEqual(raw[5], 1)
        self.assertEqual(raw[6], 1)

        # Parse back
        parsed, consumed = MavlinkPacket.parse_one(raw, crc_extra_map={30: 39})
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(consumed, len(raw))
        self.assertEqual(parsed.magic, MAVLINK_V2_STX)
        self.assertEqual(parsed.seq, 42)
        self.assertEqual(parsed.sysid, 1)
        self.assertEqual(parsed.compid, 1)
        self.assertEqual(parsed.msgid, 30)
        self.assertEqual(parsed.payload, payload)
        self.assertTrue(parsed.is_v2)

    def test_v1_pack_and_parse(self) -> None:
        """Test packing a MAVLink v1 packet and parsing it back."""
        payload = b"V1Frame"
        pkt = MavlinkPacket(
            magic=MAVLINK_V1_STX,
            seq=7,
            sysid=255,
            compid=191,
            msgid=0,
            payload=payload,
        )
        raw = pkt.pack(crc_extra=50)

        self.assertEqual(raw[0], MAVLINK_V1_STX)
        self.assertEqual(raw[1], len(payload))

        parsed, consumed = MavlinkPacket.parse_one(raw, crc_extra_map={0: 50})
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(consumed, len(raw))
        self.assertEqual(parsed.magic, MAVLINK_V1_STX)
        self.assertEqual(parsed.seq, 7)
        self.assertEqual(parsed.sysid, 255)
        self.assertEqual(parsed.compid, 191)
        self.assertEqual(parsed.msgid, 0)
        self.assertEqual(parsed.payload, payload)
        self.assertFalse(parsed.is_v2)

    def test_parse_incomplete_buffer(self) -> None:
        """Incomplete buffer should return None and 0 consumed."""
        pkt = MavlinkPacket(
            magic=MAVLINK_V2_STX,
            seq=1,
            sysid=1,
            compid=1,
            msgid=0,
            payload=b"123456789",
        )
        raw = pkt.pack(crc_extra=50)

        # Truncate by 3 bytes
        truncated = raw[:-3]
        parsed, consumed = MavlinkPacket.parse_one(truncated)
        self.assertIsNone(parsed)
        self.assertEqual(consumed, 0)

    def test_parse_garbage_preamble(self) -> None:
        """Garbage before a valid frame should skip 1 byte."""
        garbage = b"\x00\x12\x34"
        parsed, consumed = MavlinkPacket.parse_one(garbage)
        self.assertIsNone(parsed)
        self.assertEqual(consumed, 1)

    def test_corrupted_checksum(self) -> None:
        """Packet with invalid checksum should be rejected when crc_extra is provided."""
        pkt = MavlinkPacket(
            magic=MAVLINK_V2_STX,
            seq=1,
            sysid=1,
            compid=1,
            msgid=0,
            payload=b"\x00" * 9,
        )
        raw = bytearray(pkt.pack(crc_extra=50))
        # Corrupt last byte
        raw[-1] ^= 0xFF

        parsed, consumed = MavlinkPacket.parse_one(bytes(raw), crc_extra_map={0: 50})
        self.assertIsNone(parsed)
        self.assertEqual(consumed, 1)


if __name__ == "__main__":
    unittest.main()
