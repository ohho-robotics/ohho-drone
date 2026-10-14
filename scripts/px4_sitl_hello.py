#!/usr/bin/env python3
"""Minimal PX4 SITL hello-world script using MAVLink adapter prototype.

==============================================================================
WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY
DO NOT USE FOR REAL AIRCRAFT FLIGHT OR PHYSICAL HARDWARE CONTROL.
==============================================================================

Usage:
  # Run in mock SITL mode (zero-dependency, suitable for CI and headless testing):
  python scripts/px4_sitl_hello.py --mock

  # Run against a live local PX4 SITL Docker container or local binary on port 14540:
  python scripts/px4_sitl_hello.py --host 127.0.0.1 --port 14540
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Add repo root to sys.path so script can be run directly
root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from ohho_drone.adapter import MavlinkDroneTransport  # noqa: E402
from ohho_drone.schema import ConnectionState, DroneVelocity  # noqa: E402
from ohho_drone.sitl.mock_server import MockPx4SitlServer  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OhhO Drone — PX4 SITL MAVLink Hello-World (Simulation Prototype)"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="SITL MAVLink UDP target host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=14540,
        help="SITL MAVLink UDP target port (default: 14540)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Launch an in-process Mock PX4 SITL server for headless verification",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=4.0,
        help="Connection timeout in seconds (default: 4.0)",
    )
    parser.add_argument(
        "--takeoff",
        action="store_true",
        default=True,
        help="Run takeoff, velocity setpoint, and land flight sequence (default: True)",
    )
    return parser.parse_args()


def print_banner() -> None:
    print("=" * 72)
    print("  OhhO Drone -- PX4 SITL MAVLink Hello-World Prototype")
    print("  Status: EXPERIMENTAL STRETCH PROTOTYPE -- NOT FLIGHT-READY")
    print("  Software simulation only -- No real aircraft or hardware")
    print("=" * 72)


def main() -> int:
    args = parse_args()
    print_banner()

    mock_server = None
    target_host = args.host
    target_port = args.port

    if args.mock:
        print(f"\n[1/5] Starting in-process Mock PX4 SITL server on {args.host}...")
        mock_server = MockPx4SitlServer(host=args.host, port=0)
        mock_server.start()
        target_port = mock_server.port
        print(f"      Mock SITL server listening on UDP {args.host}:{target_port}")
    else:
        print(
            f"\n[1/5] Targeting PX4 SITL endpoint at UDP {target_host}:{target_port}..."
        )

    transport = MavlinkDroneTransport(
        host=target_host,
        port=target_port,
        sysid=255,
        compid=191,
    )

    telemetry_samples = []

    def on_telemetry(tele):
        telemetry_samples.append(tele)

    unsub = transport.on_telemetry(on_telemetry)

    try:
        print(
            f"[2/5] Connecting to MAVLink endpoint (timeout: {args.timeout:.1f}s)..."
        )
        status = transport.connect(timeout_s=args.timeout)

        if status.state != ConnectionState.CONNECTED:
            print(f"[-] Connection failed: {status.error}", file=sys.stderr)
            if not args.mock:
                print(
                    "\nTip: If PX4 SITL is not running locally, "
                    "pass `--mock` to use the built-in mock server.",
                    file=sys.stderr,
                )
            return 1

        print(
            f"[+] Connected! Protocol: {status.protocol}, State: {status.state.value}"
        )
        print(f"    Label: {status.label}")

        # Wait briefly for initial telemetry stream
        time.sleep(0.5)
        drone_tele = transport.read_drone_telemetry()
        print("\n[3/5] Initial Telemetry Snapshot:")
        print(
            f"      Armed: {drone_tele.armed} | Mode: {drone_tele.flight_mode.value}"
        )
        print(
            f"      Battery: {drone_tele.battery * 100:.1f}% ({drone_tele.voltage_v:.2f}V)"
            if drone_tele.battery
            else "      Battery: N/A"
        )
        if drone_tele.global_pos:
            gp = drone_tele.global_pos
            print(
                f"      GNSS: {gp.lat:.6f}, {gp.lon:.6f} | "
                f"Alt: {gp.relative_alt:.2f}m"
            )
        if drone_tele.pose:
            p = drone_tele.pose
            print(
                f"      Pose (x, y, z): {p.x:.2f}m, {p.y:.2f}m, {p.z:.2f}m"
            )

        if args.takeoff:
            print("\n[4/5] Executing simulated flight sequence:")
            print("      -> Sending ARM command...")
            if transport.arm():
                print("      [+] Autopilot armed.")
            else:
                print("      [!] Arming command sent (proceeding).")

            print("      -> Sending TAKEOFF command (target: 2.5m)...")
            transport.takeoff(altitude_m=2.5)
            time.sleep(1.0)

            print(
                "      -> Sending forward velocity setpoint (linear_x=0.5 m/s)..."
            )
            transport.send_drone_velocity(
                DroneVelocity(linear_x=0.5, linear_y=0.0, linear_z=0.0)
            )
            time.sleep(1.0)

            print("      -> Sending LAND command...")
            transport.land()
            time.sleep(0.5)

        print("\n[5/5] Disconnecting...")
        transport.disconnect()
        print(
            f"[+] Clean shutdown complete. Received {len(telemetry_samples)} telemetry frames."
        )
        return 0

    finally:
        unsub()
        if mock_server is not None:
            mock_server.stop()


if __name__ == "__main__":
    sys.exit(main())
