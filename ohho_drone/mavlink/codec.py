"""MAVLink stream encoder/decoder.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
"""

from __future__ import annotations

from typing import Any

from .messages import (
    CRC_EXTRA_MAP,
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
from .packet import MavlinkPacket

MESSAGE_PARSERS = {
    HeartbeatMessage.msg_id: HeartbeatMessage.unpack,
    SysStatusMessage.msg_id: SysStatusMessage.unpack,
    AttitudeMessage.msg_id: AttitudeMessage.unpack,
    LocalPositionNedMessage.msg_id: LocalPositionNedMessage.unpack,
    GlobalPositionIntMessage.msg_id: GlobalPositionIntMessage.unpack,
    CommandLongMessage.msg_id: CommandLongMessage.unpack,
    CommandAckMessage.msg_id: CommandAckMessage.unpack,
    SetPositionTargetLocalNedMessage.msg_id: SetPositionTargetLocalNedMessage.unpack,
    ExtendedSysStateMessage.msg_id: ExtendedSysStateMessage.unpack,
}


class MavlinkCodec:
    """Stream codec for MAVLink protocol."""

    def __init__(self, sysid: int = 255, compid: int = 191) -> None:
        self.sysid = sysid
        self.compid = compid
        self._seq = 0
        self._buffer = bytearray()

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def encode(self, msg: Any) -> bytes:
        """Encode a typed message object into framed MAVLink packet bytes."""
        msg_id = getattr(msg, "msg_id")
        crc_extra = getattr(msg, "crc_extra")
        payload = msg.pack()

        pkt = MavlinkPacket(
            seq=self._next_seq(),
            sysid=self.sysid,
            compid=self.compid,
            msgid=msg_id,
            payload=payload,
        )
        return pkt.pack(crc_extra=crc_extra)

    def feed(self, data: bytes) -> list[tuple[MavlinkPacket, Any]]:
        """Append incoming bytes to buffer and return list of (packet, decoded_msg).

        If a packet is known, decoded_msg is the typed message dataclass.
        If unknown, decoded_msg is None.
        """
        self._buffer.extend(data)
        results: list[tuple[MavlinkPacket, Any]] = []

        while True:
            pkt, consumed = MavlinkPacket.parse_one(
                bytes(self._buffer), crc_extra_map=CRC_EXTRA_MAP
            )
            if consumed == 0:
                break

            del self._buffer[:consumed]
            if pkt is not None:
                parser = MESSAGE_PARSERS.get(pkt.msgid)
                decoded = None
                if parser is not None:
                    try:
                        decoded = parser(pkt.payload)
                    except Exception:
                        decoded = None
                results.append((pkt, decoded))

        return results


__all__ = ["MavlinkCodec"]
