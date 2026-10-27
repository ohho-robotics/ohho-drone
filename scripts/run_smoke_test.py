#!/usr/bin/env python3
"""Automated end-to-end smoke test for PX4 SITL MAVLink adapter prototype.

==============================================================================
WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
==============================================================================

Exit 0 on success, exit 1 on any failure.
Suitable for headless CI workflows without external container dependencies.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from ohho_drone.adapter import MavlinkDroneTransport  # noqa: E402
from ohho_drone.mavlink.messages import (  # noqa: E402
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_FORCE_DISARM_MAGIC,
)
from ohho_drone.schema import ConnectionState, DroneVelocity, FlightMode  # noqa: E402
from ohho_drone.sitl.mock_server import MockPx4SitlServer  # noqa: E402


def _require(condition: object, message: str) -> None:
    """Raise on failure. Unlike assert, this stays active under python -O."""
    if not condition:
        raise RuntimeError(message)


def run_smoke_test() -> int:
    print("--- Starting OhhO Drone PX4 SITL MAVLink Smoke Test ---")

    # 1. Start mock server on ephemeral port
    server = MockPx4SitlServer(host="127.0.0.1", port=0)
    server.start()
    port = server.port
    print(f"[1] Mock SITL server started on 127.0.0.1:{port}")

    transport = MavlinkDroneTransport(
        host="127.0.0.1",
        port=port,
        sysid=255,
        compid=191,
    )

    received_frames = []

    def on_telemetry(t):
        received_frames.append(t)

    unsub = transport.on_telemetry(on_telemetry)

    try:
        # 2. Connect
        print("[2] Connecting to mock server...")
        status = transport.connect(timeout_s=3.0)
        _require(
            status.state == ConnectionState.CONNECTED,
            f"Expected CONNECTED, got {status.state}",
        )
        print(f"    Connected successfully: {status.label}")

        # 3. Wait for telemetry streaming
        print("[3] Waiting for telemetry stream...")
        start_wait = time.time()
        while len(received_frames) < 5 and (time.time() - start_wait) < 3.0:
            time.sleep(0.05)

        _require(
            len(received_frames) >= 5,
            f"Expected >= 5 telemetry frames, got {len(received_frames)}",
        )
        print(f"    Received {len(received_frames)} telemetry frames.")

        snap = transport.read_drone_telemetry()
        _require(snap.global_pos is not None, "Missing global GNSS position")
        _require(snap.pose is not None, "Missing 6-DOF pose")
        _require(
            snap.battery is not None and snap.battery > 0.0,
            "Missing battery status",
        )
        print(
            f"    Telemetry verified: battery={snap.battery*100:.0f}%, "
            f"lat={snap.global_pos.lat:.4f}"
        )

        # 4. Arming
        print("[4] Testing arm command...")
        armed = transport.arm(timeout_s=2.0)
        _require(armed, "Arm command failed or timed out")
        _require(server.armed, "Server does not report armed state")
        print("    Arm command verified.")

        # 5. Takeoff command. Climb well clear of the ground so the later
        #    emergency-stop check cannot be confused with touchdown.
        print("[5] Testing takeoff command...")
        transport.takeoff(altitude_m=2.0)
        climb_deadline = time.time() + 3.0
        while server.altitude < 1.0 and time.time() < climb_deadline:
            time.sleep(0.05)
        _require(
            server.altitude > 1.0,
            f"Expected climb, got altitude {server.altitude}",
        )
        print(f"    Takeoff verified (altitude: {server.altitude:.2f}m).")

        # 6. Velocity setpoint
        print("[6] Testing offboard velocity setpoint...")
        transport.send_drone_velocity(
            DroneVelocity(linear_x=1.0, linear_y=0.0, linear_z=0.0)
        )
        motion_deadline = time.time() + 3.0
        while server.x <= 0.05 and time.time() < motion_deadline:
            time.sleep(0.05)
        _require(server.x > 0.05, f"Expected forward motion, got x={server.x}")
        print(f"    Velocity control verified (x position: {server.x:.2f}m).")

        # 7. In-flight emergency stop must land/hold, not disarm.
        print("[7] Testing in-flight emergency stop...")
        transport.emergency_stop()
        mode_deadline = time.time() + 3.0
        while (
            server.flight_mode not in (FlightMode.LAND, FlightMode.HOLD)
            and time.time() < mode_deadline
        ):
            time.sleep(0.05)
        _require(server.armed, "In-flight emergency stop must not disarm")
        _require(
            server.flight_mode in (FlightMode.LAND, FlightMode.HOLD),
            f"Expected LAND or HOLD, got {server.flight_mode}",
        )
        _require(
            not any(
                c[0] == MAV_CMD_COMPONENT_ARM_DISARM and int(c[1]) == 0
                for c in server.received_commands()
            ),
            "In-flight emergency stop sent a disarm",
        )
        _require(
            not any(
                int(c[2]) == MAV_FORCE_DISARM_MAGIC for c in server.received_commands()
            ),
            "Force-disarm magic was sent",
        )
        transport.release_stop()
        print("    In-flight emergency stop verified (land/hold, still armed).")

        # 8. Clean disconnect
        print("[8] Testing disconnect...")
        transport.disconnect()
        _require(
            transport.status().state == ConnectionState.DISCONNECTED,
            "Failed to disconnect",
        )
        print("    Disconnect verified.")

        print("--- ALL SMOKE TESTS PASSED SUCCESSFULLY ---")
        return 0

    except Exception as e:
        print(f"\n[!] Smoke test failed: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1

    finally:
        unsub()
        transport.disconnect()
        server.stop()


if __name__ == "__main__":
    sys.exit(run_smoke_test())
