"""Integration tests for MockPx4SitlServer and MavlinkDroneTransport over UDP."""

from __future__ import annotations

import time
import unittest

from ohho_drone.adapter import MavlinkDroneTransport
from ohho_drone.mavlink.messages import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_FORCE_DISARM_MAGIC,
    MAV_LANDED_STATE_IN_AIR,
    MAV_LANDED_STATE_LANDING,
    MAV_LANDED_STATE_ON_GROUND,
    MAV_LANDED_STATE_TAKEOFF,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_MODE_AUTO_LOITER,
)
from ohho_drone.schema import ConnectionState, DroneVelocity, FlightMode
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

        received = []
        self.transport.on_telemetry(lambda t: received.append(t))

        def _fields_ready() -> bool:
            # Heartbeat, position, global position, and battery arrive as
            # separate datagrams. The first callback can precede the fields
            # this test reads.
            tele = self.transport.read_drone_telemetry()
            return (
                len(received) > 0
                and tele.pose is not None
                and tele.global_pos is not None
                and tele.battery is not None
            )

        self.assertTrue(self._wait_until(_fields_ready))
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
        self.assertTrue(self._wait_until(lambda: self.server.armed))

        disarmed = self.transport.disarm(timeout_s=2.0)
        self.assertTrue(disarmed)
        self.assertTrue(self._wait_until(lambda: not self.server.armed))

    def test_takeoff_and_velocity_flight(self) -> None:
        """Takeoff causes vehicle altitude climb; velocity causes displacement."""
        self.transport.connect(timeout_s=3.0)
        self.transport.arm(timeout_s=2.0)

        self.transport.takeoff(altitude_m=2.0)
        self.assertTrue(self._wait_until(lambda: self.server.altitude > 0.05))

        self.transport.send_drone_velocity(
            DroneVelocity(linear_x=2.0, linear_y=0.0, linear_z=0.0)
        )
        self.assertTrue(self._wait_until(lambda: self.server.x > 0.05))

    def test_emergency_stop_on_ground_disarms(self) -> None:
        """On-ground e-stop disarms only after landed_state confirms ON_GROUND."""
        self.transport.connect(timeout_s=3.0)
        self.assertTrue(
            self._wait_until(
                lambda: self.transport.landed_state == MAV_LANDED_STATE_ON_GROUND
            )
        )
        self.assertTrue(self.transport.arm(timeout_s=2.0))
        self.assertTrue(self._wait_until(lambda: self.server.armed))
        self.assertLess(self.server.altitude, 0.15)

        before = len(self.server.received_commands())
        self.transport.emergency_stop()
        self.assertTrue(self._wait_until(lambda: not self.server.armed))
        new = self.server.received_commands()[before:]
        disarms = [
            c
            for c in new
            if c[0] == MAV_CMD_COMPONENT_ARM_DISARM and int(c[1]) == 0
        ]
        self.assertEqual(len(disarms), 1)
        self.assertEqual(int(disarms[0][2]), 0)
        self.assertNotEqual(int(disarms[0][2]), MAV_FORCE_DISARM_MAGIC)
        self.assertTrue(self.transport._e_stop)

    def test_emergency_stop_in_air_lands_and_never_disarms(self) -> None:
        """In-air e-stop commands LAND or HOLD and does not disarm."""
        self.transport.connect(timeout_s=3.0)
        self.assertTrue(self.transport.arm(timeout_s=2.0))
        self.transport.takeoff(altitude_m=2.0)
        airborne = {
            MAV_LANDED_STATE_IN_AIR,
            MAV_LANDED_STATE_TAKEOFF,
            MAV_LANDED_STATE_LANDING,
        }
        self.assertTrue(
            self._wait_until(
                lambda: self.server.altitude > 0.8
                and self.transport.landed_state in airborne
            )
        )

        before = len(self.server.received_commands())
        self.transport.emergency_stop()
        # Touchdown clears armed and flight_mode in one step. A wait that
        # still required the in-air LAND/HOLD state would time out if it
        # sampled after that. The command log is what remains.
        self.assertTrue(
            self._wait_until(
                lambda: (not self.server.armed)
                and self.server.flight_mode == FlightMode.DISARMED,
                timeout=5.0,
            )
        )
        self.assertTrue(
            self._wait_until(
                lambda: any(
                    c[0] in (MAV_CMD_NAV_LAND, MAV_CMD_DO_SET_MODE)
                    for c in self.server.received_commands()[before:]
                )
            )
        )
        new = self.server.received_commands()[before:]
        self.assertTrue(
            any(c[0] in (MAV_CMD_NAV_LAND, MAV_CMD_DO_SET_MODE) for c in new)
        )
        self.assertFalse(
            any(
                c[0] == MAV_CMD_COMPONENT_ARM_DISARM and int(c[1]) == 0
                for c in new
            )
        )
        self.assertFalse(
            any(int(c[2]) == MAV_FORCE_DISARM_MAGIC for c in self.server.received_commands())
        )

    def test_emergency_stop_unknown_landed_state_holds_then_lands(self) -> None:
        """Missing landed_state commands AUTO.LOITER and then LAND, never disarm."""
        self.transport.disconnect()
        self.server.stop()
        self.server = MockPx4SitlServer(
            host="127.0.0.1", port=0, report_landed_state=False
        )
        self.server.start()
        self.transport = MavlinkDroneTransport(host="127.0.0.1", port=self.server.port)
        status = self.transport.connect(timeout_s=3.0)
        self.assertEqual(status.state, ConnectionState.CONNECTED)
        self.assertTrue(self.transport.arm(timeout_s=2.0))
        self.assertTrue(
            self._wait_until(lambda: self.transport.read_drone_telemetry().pose is not None)
        )
        self.assertIsNone(self.transport.landed_state)

        before = len(self.server.received_commands())
        self.transport.emergency_stop()
        self.assertTrue(
            self._wait_until(
                lambda: any(
                    c[0] == MAV_CMD_NAV_LAND
                    for c in self.server.received_commands()[before:]
                )
            )
        )
        new = self.server.received_commands()[before:]
        hold_at = next(i for i, c in enumerate(new) if c[0] == MAV_CMD_DO_SET_MODE)
        land_at = next(i for i, c in enumerate(new) if c[0] == MAV_CMD_NAV_LAND)
        self.assertLess(hold_at, land_at)
        self.assertEqual(int(new[hold_at][2]), PX4_MODE_AUTO_LOITER)
        self.assertFalse(
            any(
                c[0] == MAV_CMD_COMPONENT_ARM_DISARM and int(c[1]) == 0
                for c in new
            )
        )

    def test_set_offboard_mode_sends_px4_main_mode(self) -> None:
        """OFFBOARD is PX4 main mode 6 in DO_SET_MODE param2."""
        self.transport.connect(timeout_s=3.0)
        self.assertTrue(self.transport.set_offboard_mode(timeout_s=2.0))
        mode_cmds = [
            c
            for c in self.server.received_commands()
            if c[0] == MAV_CMD_DO_SET_MODE
        ]
        self.assertEqual(len(mode_cmds), 1)
        self.assertEqual(int(mode_cmds[0][1]), MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        self.assertEqual(int(mode_cmds[0][2]), PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        self.assertEqual(self.server.flight_mode, FlightMode.OFFBOARD)

    def test_non_loopback_host_refused(self) -> None:
        """A non-loopback host is refused and never contacts the mock SITL."""
        refused = MavlinkDroneTransport(host="192.0.2.10", port=self.port)
        status = refused.connect(timeout_s=1.0)
        self.assertEqual(status.state, ConnectionState.ERROR)
        self.assertIsNotNone(status.error)
        assert status.error is not None
        self.assertIn("Refusing connection", status.error)
        self.assertIn("loopback", status.error)
        self.assertIsNone(refused._sock)
        self.assertIsNone(self.server.client_address)

        flagged = MavlinkDroneTransport(host="203.0.113.9", port=self.port, sitl=True)
        flagged_status = flagged.connect(timeout_s=1.0)
        self.assertEqual(flagged_status.state, ConnectionState.ERROR)
        self.assertIsNone(flagged._sock)
        self.assertIsNone(self.server.client_address)

    def test_takeoff_denied_while_disarmed(self) -> None:
        self.transport.connect(timeout_s=3.0)
        self.assertFalse(self.transport.takeoff(altitude_m=2.0, timeout_s=2.0))
        self.assertFalse(self.server.armed)
        self.assertLess(self.server.altitude, 0.05)

    def test_disarm_denied_while_airborne(self) -> None:
        self.transport.connect(timeout_s=3.0)
        self.assertTrue(self.transport.arm(timeout_s=2.0))
        self.transport.takeoff(altitude_m=2.0)
        self.assertTrue(self._wait_until(lambda: self.server.altitude > 0.5))
        self.assertFalse(self.transport.disarm(timeout_s=2.0))
        self.assertTrue(self.server.armed)

    def _wait_until(self, predicate, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False


if __name__ == "__main__":
    unittest.main()
