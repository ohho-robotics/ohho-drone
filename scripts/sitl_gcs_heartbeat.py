#!/usr/bin/env python3
"""Simulated ground station for headless PX4 SITL.

PX4 v1.17 ``RcAndDataLinkChecks`` blocks every nav mode while
``gcs_connection_lost`` is set and ``NAV_DLL_ACT > 0``. The console line is
``Preflight Fail: No connection to the GCS``
(``rcAndDataLinkCheck.cpp``). Commander starts with that flag set and clears
it only after a MAVLink link reports ``heartbeat_type_gcs``. The receiver
sets that from a ``HEARTBEAT`` whose type is ``MAV_TYPE_GCS``, and the flag
stays true for ``HEARTBEAT_TIMEOUT_US`` (2.5 s).

SITL instance 0 starts that link as ``mavlink start -x -u 18570``
(``px4-rc.mavlink``). ``-o`` defaults to remote port 14550 and ``init_udp``
aims it at 127.0.0.1. The first datagram from localhost is accepted.

This process binds 127.0.0.1:14550 and sends only that heartbeat to
127.0.0.1:18570 at 1 Hz. It does not arm, change parameters, or send any
other MAVLink message. ``NAV_DLL_ACT`` stays at the stock SITL value.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

# px4-rc.mavlink instance 0, and mavlink's default -o.
PX4_GCS_PORT = 18570
GCS_BIND_PORT = 14550
LOOPBACK = "127.0.0.1"
# Faster than the 2.5 s heartbeat_type_gcs timeout, and inside COM_DL_LOSS_T.
HEARTBEAT_PERIOD_S = 1.0
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190  # MAV_COMP_ID_MISSIONPLANNER


class GcsHeartbeatError(RuntimeError):
    """The simulated GCS cannot start. The process exits non-zero."""


def require_instance_zero(environ: dict[str, str]) -> None:
    """This harness only flies PX4 instance 0.

    Instance 0 listens on 18570 and sends to 14550. A later instance moves
    the local port and still uses remote 14550, which this bind would steal.
    """
    raw = environ.get("PX4_INSTANCE", "0").strip() or "0"
    if raw != "0":
        raise GcsHeartbeatError(
            f"PX4_INSTANCE is {raw!r}. The simulated GCS only talks to "
            f"instance 0 (PX4 udp {PX4_GCS_PORT}, remote {GCS_BIND_PORT})."
        )


def _common() -> Any:
    """MAVLink 2 common dialect. Importing v20 does not depend on MAVLINK20."""
    from pymavlink.dialects.v20 import common as mavlink2

    return mavlink2


def check_pymavlink_pin() -> str:
    """Return the installed version when it is the one requirements pin."""
    import pymavlink

    pin_path = Path(__file__).resolve().parents[1] / "requirements-sitl.txt"
    try:
        text = pin_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise GcsHeartbeatError(f"cannot read {pin_path.name}: {exc}") from exc
    version = str(pymavlink.__version__)
    if f"pymavlink=={version}" not in text:
        raise GcsHeartbeatError(
            f"pymavlink {version} is not the pin in {pin_path.name}"
        )
    return version


def gcs_heartbeat_bytes() -> bytes:
    """One MAVLink 2 HEARTBEAT of type MAV_TYPE_GCS. Nothing else is packed."""
    mavlink2 = _common()
    link = mavlink2.MAVLink(
        None, srcSystem=GCS_SYSTEM_ID, srcComponent=GCS_COMPONENT_ID
    )
    message = link.heartbeat_encode(
        mavlink2.MAV_TYPE_GCS,
        mavlink2.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )
    packet = message.pack(link)
    if not isinstance(packet, (bytes, bytearray)) or not packet:
        raise GcsHeartbeatError("pymavlink did not pack a heartbeat")
    if packet[0] != 0xFD:
        raise GcsHeartbeatError("heartbeat was not MAVLink 2")
    return bytes(packet)


def decode_packets(payload: bytes) -> list[Any]:
    """Messages in ``payload``. Used by tests to prove the datagram is one heartbeat."""
    link = _common().MAVLink(None)
    found: list[Any] = []
    for byte in payload:
        message = link.parse_char(bytes([byte]))
        if message is not None:
            found.append(message)
    return found


def _drain(sock: socket.socket) -> None:
    """Drop PX4's stream so the receive buffer does not sit full."""
    sock.setblocking(False)
    try:
        while True:
            try:
                sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError, ConnectionResetError):
                break
    finally:
        sock.setblocking(True)


def serve(
    px4_port: int = PX4_GCS_PORT,
    gcs_port: int = GCS_BIND_PORT,
    *,
    period_s: float = HEARTBEAT_PERIOD_S,
    max_heartbeats: Optional[int] = None,
    log: Callable[[str], None] = print,
) -> int:
    """Bind ``gcs_port`` on loopback and send heartbeats to ``px4_port``.

    Returns how many heartbeats were sent. ``max_heartbeats`` stops the loop
    for tests. The CI process leaves it unset and runs until it is killed.
    """
    if period_s <= 0.0:
        raise GcsHeartbeatError("heartbeat period must be positive")
    if not 1 <= px4_port <= 65535 or not 1 <= gcs_port <= 65535:
        raise GcsHeartbeatError("GCS ports must be in 1..65535")
    packet = gcs_heartbeat_bytes()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((LOOPBACK, gcs_port))
        sent = 0
        attempts = 0
        while max_heartbeats is None or attempts < max_heartbeats:
            attempts += 1
            try:
                sock.sendto(packet, (LOOPBACK, px4_port))
            except OSError as exc:
                log(f"gcs heartbeat send failed: {exc}")
            else:
                sent += 1
                if sent == 1 or sent % 30 == 0:
                    log(
                        f"gcs heartbeat {sent} {LOOPBACK}:{gcs_port} -> "
                        f"{LOOPBACK}:{px4_port}"
                    )
            if max_heartbeats is not None and attempts >= max_heartbeats:
                break
            _drain(sock)
            time.sleep(period_s)
        return sent
    finally:
        sock.close()


def _parse_args(argv: Sequence[str]) -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Send MAVLink GCS heartbeats to loopback PX4 SITL. "
            "There is no target argument and no arm command."
        )
    )
    parser.parse_args(list(argv))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = sys.argv[1:] if argv is None else list(argv)
    try:
        _parse_args(args)
        require_instance_zero(dict(os.environ))
        version = check_pymavlink_pin()
        print(
            f"gcs heartbeat pymavlink {version} listening on "
            f"{LOOPBACK}:{GCS_BIND_PORT} sending to {LOOPBACK}:{PX4_GCS_PORT}",
            flush=True,
        )
        serve()
    except GcsHeartbeatError as exc:
        print(f"gcs heartbeat failed: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"gcs heartbeat failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
