"""Pure-Python MAVLink v1/v2 packet framing, checksums, and stream parsing.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD

HEADER_LEN_V1 = 6
HEADER_LEN_V2 = 10
CHECKSUM_LEN = 2


def mavlink_x25_crc(data: bytes, init: int = 0xFFFF) -> int:
    """Compute standard MAVLink X.25 CRC-16."""
    crc = init
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


@dataclass
class MavlinkPacket:
    """A single framed MAVLink packet."""

    magic: int = MAVLINK_V2_STX
    payload_len: int = 0
    incompat_flags: int = 0
    compat_flags: int = 0
    seq: int = 0
    sysid: int = 1
    compid: int = 1
    msgid: int = 0
    payload: bytes = b""
    checksum: int = 0

    @property
    def is_v2(self) -> bool:
        return self.magic == MAVLINK_V2_STX

    def pack(self, crc_extra: int = 0) -> bytes:
        """Serialize packet into wire bytes including header, payload, and CRC."""
        payload_bytes = self.payload
        self.payload_len = len(payload_bytes)

        if self.is_v2:
            header = bytes(
                [
                    MAVLINK_V2_STX,
                    self.payload_len,
                    self.incompat_flags,
                    self.compat_flags,
                    self.seq & 0xFF,
                    self.sysid & 0xFF,
                    self.compid & 0xFF,
                    self.msgid & 0xFF,
                    (self.msgid >> 8) & 0xFF,
                    (self.msgid >> 16) & 0xFF,
                ]
            )
        else:
            header = bytes(
                [
                    MAVLINK_V1_STX,
                    self.payload_len,
                    self.seq & 0xFF,
                    self.sysid & 0xFF,
                    self.compid & 0xFF,
                    self.msgid & 0xFF,
                ]
            )

        crc_input = header[1:] + payload_bytes + bytes([crc_extra & 0xFF])
        self.checksum = mavlink_x25_crc(crc_input)
        return header + payload_bytes + struct.pack("<H", self.checksum)

    @classmethod
    def parse_one(
        cls, buffer: bytes, crc_extra_map: dict[int, int] | None = None
    ) -> tuple[MavlinkPacket | None, int]:
        """Attempt to parse one packet from buffer.

        Returns:
            (packet, consumed_bytes)
            If buffer is incomplete, returns (None, 0).
            If header magic is invalid, discards 1 byte and returns (None, 1).
        """
        if not buffer:
            return None, 0

        magic = buffer[0]
        if magic != MAVLINK_V1_STX and magic != MAVLINK_V2_STX:
            # Bad magic, advance 1 byte
            return None, 1

        if magic == MAVLINK_V2_STX:
            if len(buffer) < HEADER_LEN_V2:
                return None, 0
            payload_len = buffer[1]
            total_len = HEADER_LEN_V2 + payload_len + CHECKSUM_LEN
            if len(buffer) < total_len:
                return None, 0

            incompat_flags = buffer[2]
            compat_flags = buffer[3]
            seq = buffer[4]
            sysid = buffer[5]
            compid = buffer[6]
            msgid = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16)
            payload = buffer[HEADER_LEN_V2 : HEADER_LEN_V2 + payload_len]
            (checksum,) = struct.unpack(
                "<H", buffer[HEADER_LEN_V2 + payload_len : total_len]
            )

            # Check CRC if crc_extra is known
            if crc_extra_map is not None and msgid in crc_extra_map:
                crc_extra = crc_extra_map[msgid]
                expected_crc = mavlink_x25_crc(
                    buffer[1 : HEADER_LEN_V2 + payload_len] + bytes([crc_extra])
                )
                if checksum != expected_crc:
                    # Checksum mismatch, advance past magic byte
                    return None, 1

            pkt = cls(
                magic=magic,
                payload_len=payload_len,
                incompat_flags=incompat_flags,
                compat_flags=compat_flags,
                seq=seq,
                sysid=sysid,
                compid=compid,
                msgid=msgid,
                payload=payload,
                checksum=checksum,
            )
            return pkt, total_len

        elif magic == MAVLINK_V1_STX:
            if len(buffer) < HEADER_LEN_V1:
                return None, 0
            payload_len = buffer[1]
            total_len = HEADER_LEN_V1 + payload_len + CHECKSUM_LEN
            if len(buffer) < total_len:
                return None, 0

            seq = buffer[2]
            sysid = buffer[3]
            compid = buffer[4]
            msgid = buffer[5]
            payload = buffer[HEADER_LEN_V1 : HEADER_LEN_V1 + payload_len]
            (checksum,) = struct.unpack(
                "<H", buffer[HEADER_LEN_V1 + payload_len : total_len]
            )

            if crc_extra_map is not None and msgid in crc_extra_map:
                crc_extra = crc_extra_map[msgid]
                expected_crc = mavlink_x25_crc(
                    buffer[1 : HEADER_LEN_V1 + payload_len] + bytes([crc_extra])
                )
                if checksum != expected_crc:
                    return None, 1

            pkt = cls(
                magic=magic,
                payload_len=payload_len,
                incompat_flags=0,
                compat_flags=0,
                seq=seq,
                sysid=sysid,
                compid=compid,
                msgid=msgid,
                payload=payload,
                checksum=checksum,
            )
            return pkt, total_len

        else:
            # Bad magic, advance 1 byte
            return None, 1


__all__ = [
    "CHECKSUM_LEN",
    "HEADER_LEN_V1",
    "HEADER_LEN_V2",
    "MAVLINK_V1_STX",
    "MAVLINK_V2_STX",
    "MavlinkPacket",
    "mavlink_x25_crc",
]
