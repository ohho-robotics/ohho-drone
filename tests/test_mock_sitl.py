"""Integration tests for MockPx4SitlServer and MavlinkDroneTransport over UDP."""

from __future__ import annotations

import time
import unittest

from ohho_drone.adapter import MavlinkDroneTransport
from ohho_drone.schema import ConnectionState, DroneVelocity
from ohho_drone.sitl.mock_server import MockPx4SitlServer


class TestMockSitlIntegration(unittest.TestCase):
    def setUp(self) -> None:
        # Bind to port 0 for an OS-assigned free ephemeral port
        self.server = MockPx4SitlServer(host="127.0.0.1", port=0)
        self.server.start()
        self.port = self.server.port
        self.transport = MavlinkDroneTransport(
            host="127.0.0.1",
            port=self.port,
            sysid=255,
            compid=191,
        )

    def tearDown(self) -> None:
        self.transport.disconnect()
        self.server.stop()

    def test_connect_and_heartbeat(self) -> None:
        """Transport connects to MockPx4SitlServer and establishes heartbeat."""
        status = self.transport.connect(timeout_s=3.0)
        self.assertEqual(status.state, ConnectionState.CONNECTED)
        self.assertIsNone(status.error)
        self.assertIsNotNone(status.connected_since)

    def test_telemetry_streaming(self) -> None:
        """Transport receives local position, attitude, battery, and gnss."""
        self.transport.connect(timeout_s=3.0)

        # Wait for a few telemetry packets
        received = []
        self.transport.on_telemetry(lambda t: received.append(t))
        time.sleep(0.3)

        self.assertGreater(len(received), 0)
        drone_tele = self.transport.read_drone_telemetry()
        self.assertIsNotNone(drone_tele.pose)
        self.assertIsNotNone(drone_tele.global_pos)
        self.assertIsNotNone(drone_tele.battery)
        assert drone_tele.battery is not None
        self.assertAlmostEqual(drone_tele.battery, 0.98, places=2)

    def test_arm_and_disarm(self) -> None:
        """Arming changes vehicle armed state on both server and client."""
        self.transport.connect(timeout_s=3.0)
        self.assertFalse(self.server.armed)

        armed = self.transport.arm(timeout_s=2.0)
        self.assertTrue(armed)
        time.sleep(0.1)
        self.assertTrue(self.server.armed)

        disarmed = self.transport.disarm(timeout_s=2.0)
        self.assertTrue(disarmed)
        time.sleep(0.1)
        self.assertFalse(self.server.armed)

    def test_takeoff_and_velocity_flight(self) -> None:
        """Takeoff causes vehicle altitude climb; velocity causes displacement."""
        self.transport.connect(timeout_s=3.0)
        self.transport.arm(timeout_s=2.0)

        # Takeoff to 2.0m
        self.transport.takeoff(altitude_m=2.0)
        time.sleep(0.25)
        self.assertGreater(self.server.altitude, 0.05)

        # Velocity setpoint
        self.transport.send_drone_velocity(
            DroneVelocity(linear_x=2.0, linear_y=0.0, linear_z=0.0)
        )
        time.sleep(0.25)
        self.assertGreater(self.server.x, 0.05)

    def test_emergency_stop_disarms_vehicle(self) -> None:
        """Emergency stop disarms vehicle and sets internal lock."""
        self.transport.connect(timeout_s=3.0)
        self.transport.arm(timeout_s=2.0)
        self.assertTrue(self.server.armed)

        self.transport.emergency_stop()
        time.sleep(0.15)
        self.assertFalse(self.server.armed)


if __name__ == "__main__":
    unittest.main()
