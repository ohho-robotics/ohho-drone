"""Unit tests for MavlinkDroneTransport and OhhO OS schema adapters."""

from __future__ import annotations

import socket
import unittest

from ohho_drone.adapter import MavlinkDroneTransport, simulation_target_allowed
from ohho_drone.mavlink.messages import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_FORCE_DISARM_MAGIC,
    MAV_LANDED_STATE_IN_AIR,
    MAV_LANDED_STATE_ON_GROUND,
    PX4_MODE_AUTO_LOITER,
    CommandLongMessage,
)
from ohho_drone.schema import (
    ConnectionState,
    DronePose,
    DroneTelemetry,
    DroneVelocity,
    FlightMode,
    GlobalCoordinates,
    Odometry,
    clamp,
)


class TestAdapter(unittest.TestCase):
    def test_drone_velocity_conversion(self) -> None:
        """Verify planar and 3D velocity conversions."""
        dv = DroneVelocity(linear_x=1.2, linear_y=-0.4, linear_z=0.8, yaw_rate=0.5)
        base_v = dv.to_ohho_velocity()
        self.assertEqual(base_v.linear_x, 1.2)
        self.assertEqual(base_v.linear_y, -0.4)
        self.assertEqual(base_v.angular_z, 0.5)

        restored_dv = DroneVelocity.from_ohho_velocity(base_v, linear_z=0.8)
        self.assertEqual(restored_dv.linear_x, dv.linear_x)
        self.assertEqual(restored_dv.linear_y, dv.linear_y)
        self.assertEqual(restored_dv.linear_z, 0.8)
        self.assertEqual(restored_dv.yaw_rate, dv.yaw_rate)

    def test_drone_telemetry_to_ohho_telemetry(self) -> None:
        """Verify rich DroneTelemetry exports cleanly into OhhO core Telemetry."""
        dt = DroneTelemetry(
            armed=True,
            flight_mode=FlightMode.HOLD,
            odom=Odometry(x=10.0, y=5.0, theta=1.57, vx=0.0, vy=0.0, omega=0.0),
            pose=DronePose(x=10.0, y=5.0, z=-2.5, roll=0.0, pitch=0.0, yaw=1.57),
            global_pos=GlobalCoordinates(
                lat=47.3977, lon=8.5455, alt_amsl=490.5, relative_alt=2.5
            ),
            battery=0.85,
            voltage_v=12.1,
            current_a=4.2,
            satellites_visible=12,
            gps_fix_type=3,
        )

        ohho_tele = dt.to_ohho_telemetry()
        self.assertIsNotNone(ohho_tele.odom)
        assert ohho_tele.odom is not None
        self.assertEqual(ohho_tele.odom.x, 10.0)
        self.assertEqual(ohho_tele.odom.theta, 1.57)
        self.assertEqual(ohho_tele.battery, 0.85)

        drone_custom = ohho_tele.custom.get("drone")
        self.assertIsNotNone(drone_custom)
        assert isinstance(drone_custom, dict)
        self.assertTrue(drone_custom["armed"])
        self.assertEqual(drone_custom["flight_mode"], "hold")
        self.assertEqual(drone_custom["satellites"], 12)
        self.assertEqual(drone_custom["experimental"], True)
        self.assertEqual(drone_custom["flight_ready"], False)

    def test_transport_initial_state(self) -> None:
        """Adapter starts in IDLE state with protocol mavlink."""
        t = MavlinkDroneTransport(host="127.0.0.1", port=14540)
        st = t.status()
        self.assertEqual(st.protocol, "mavlink")
        self.assertEqual(st.state, ConnectionState.IDLE)
        self.assertIn("experimental", st.label)
        self.assertIn("not-flight-ready", st.label)

    def test_telemetry_subscription(self) -> None:
        """Verify on_telemetry callback registration and unsubscription."""
        t = MavlinkDroneTransport()
        samples = []

        unsub = t.on_telemetry(lambda x: samples.append(x))
        # Initial read returns valid Telemetry
        read_tele = t.read()
        self.assertIsNotNone(read_tele)

        # Trigger internal emission
        t._emit_telemetry(read_tele)
        self.assertEqual(len(samples), 1)

        # Unsubscribe
        unsub()
        t._emit_telemetry(read_tele)
        self.assertEqual(len(samples), 1)

    def test_status_subscription(self) -> None:
        """Verify on_status callback registration and unsubscription."""
        t = MavlinkDroneTransport()
        statuses = []

        unsub = t.on_status(lambda s: statuses.append(s))
        curr = t.status()
        t._emit_status(curr)
        self.assertEqual(len(statuses), 1)

        unsub()
        t._emit_status(curr)
        self.assertEqual(len(statuses), 1)

    def test_emergency_stop_flags(self) -> None:
        """Verify emergency_stop sets e_stop and release_stop clears it."""
        t = MavlinkDroneTransport()
        self.assertFalse(t._e_stop)
        t.emergency_stop()
        self.assertTrue(t._e_stop)
        t.release_stop()
        self.assertFalse(t._e_stop)

    def test_emergency_stop_unknown_holds_then_lands(self) -> None:
        """Unknown landed state commands HOLD then LAND, and never disarms."""
        t = MavlinkDroneTransport()
        sent = self._capture(t)
        t.emergency_stop()
        commands = [m for m in sent if isinstance(m, CommandLongMessage)]
        self.assertGreaterEqual(len(commands), 2)
        self.assertEqual(commands[0].command, MAV_CMD_DO_SET_MODE)
        self.assertEqual(int(commands[0].param2), PX4_MODE_AUTO_LOITER)
        self.assertEqual(commands[1].command, MAV_CMD_NAV_LAND)
        self.assertFalse(any(m.command == MAV_CMD_COMPONENT_ARM_DISARM for m in commands))
        self.assertFalse(any(int(m.param2) == MAV_FORCE_DISARM_MAGIC for m in commands))

    def test_emergency_stop_in_air_lands_without_disarm(self) -> None:
        """Airborne landed_state commands LAND and does not disarm."""
        t = MavlinkDroneTransport()
        t._landed_state = MAV_LANDED_STATE_IN_AIR
        t._telemetry.landed_state = MAV_LANDED_STATE_IN_AIR
        sent = self._capture(t)
        t.emergency_stop()
        commands = [m for m in sent if isinstance(m, CommandLongMessage)]
        self.assertEqual([m.command for m in commands], [MAV_CMD_NAV_LAND])
        self.assertFalse(any(int(m.param2) == MAV_FORCE_DISARM_MAGIC for m in commands))

    def test_emergency_stop_altitude_without_landed_state_lands(self) -> None:
        """Armed altitude data counts as airborne when landed_state is missing."""
        t = MavlinkDroneTransport()
        t._telemetry.armed = True
        t._telemetry.pose = DronePose(z=-3.0)
        sent = self._capture(t)
        t.emergency_stop()
        commands = [m for m in sent if isinstance(m, CommandLongMessage)]
        self.assertEqual([m.command for m in commands], [MAV_CMD_NAV_LAND])

    def test_emergency_stop_on_ground_disarms_without_force_magic(self) -> None:
        """Confirmed ON_GROUND disarms with param1=0 and param2=0."""
        t = MavlinkDroneTransport()
        t._landed_state = MAV_LANDED_STATE_ON_GROUND
        t._telemetry.landed_state = MAV_LANDED_STATE_ON_GROUND
        t._telemetry.pose = DronePose(z=0.0)
        sent = self._capture(t)
        t.emergency_stop()
        commands = [m for m in sent if isinstance(m, CommandLongMessage)]
        self.assertEqual(len(commands), 1)
        disarm = commands[0]
        self.assertEqual(disarm.command, MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(int(disarm.param1), 0)
        self.assertEqual(int(disarm.param2), 0)
        self.assertNotEqual(int(disarm.param2), MAV_FORCE_DISARM_MAGIC)

    def test_emergency_stop_on_ground_flag_with_altitude_lands(self) -> None:
        """ON_GROUND is ignored when altitude shows the vehicle is airborne."""
        t = MavlinkDroneTransport()
        t._landed_state = MAV_LANDED_STATE_ON_GROUND
        t._telemetry.pose = DronePose(z=-2.0)
        sent = self._capture(t)
        t.emergency_stop()
        commands = [m for m in sent if isinstance(m, CommandLongMessage)]
        self.assertEqual([m.command for m in commands], [MAV_CMD_NAV_LAND])

    def test_force_disarm_magic_is_never_sent(self) -> None:
        """A command carrying the force-disarm magic is dropped before encode."""
        t = MavlinkDroneTransport(host="127.0.0.1", port=9)
        encoded: list[object] = []
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t._sock = sock
        original = t._codec.encode

        def capture(msg: object) -> bytes:
            encoded.append(msg)
            return original(msg)

        t._codec.encode = capture  # type: ignore[method-assign]
        try:
            t._send_message(
                CommandLongMessage(
                    command=MAV_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0,
                    param2=float(MAV_FORCE_DISARM_MAGIC),
                )
            )
            self.assertEqual(encoded, [])
        finally:
            sock.close()
            t._sock = None

    def test_non_loopback_host_is_refused(self) -> None:
        """Routable hosts are refused before a socket is opened."""
        for host in ("192.0.2.10", "10.1.2.3", "8.8.8.8", "example.com", ""):
            t = MavlinkDroneTransport(host=host, port=9)
            status = t.connect(timeout_s=2.0)
            self.assertEqual(status.state, ConnectionState.ERROR, host)
            self.assertIsNotNone(status.error)
            assert status.error is not None
            self.assertIn("Refusing connection", status.error)
            self.assertIsNone(t._sock)

    def test_sitl_flag_still_refuses_routable_hosts(self) -> None:
        """The SITL flag does not open non-local hosts."""
        t = MavlinkDroneTransport(host="198.51.100.20", port=9, sitl=True)
        status = t.connect(timeout_s=2.0)
        self.assertEqual(status.state, ConnectionState.ERROR)
        self.assertIsNotNone(status.error)
        assert status.error is not None
        self.assertIn("Refusing connection", status.error)
        self.assertIn("docker-local", status.error)
        self.assertIsNone(t._sock)

    def test_docker_local_requires_explicit_sitl_flag(self) -> None:
        """Docker-local names are refused unless sitl=True."""
        blocked = MavlinkDroneTransport(host="host.docker.internal", port=9, sitl=False)
        status = blocked.connect(timeout_s=2.0)
        self.assertEqual(status.state, ConnectionState.ERROR)
        self.assertIsNone(blocked._sock)
        self.assertTrue(simulation_target_allowed("host.docker.internal", sitl=True))
        self.assertTrue(simulation_target_allowed("127.0.0.1", sitl=False))
        self.assertTrue(simulation_target_allowed("127.8.8.8", sitl=False))
        self.assertTrue(simulation_target_allowed("::1", sitl=False))
        self.assertTrue(simulation_target_allowed("localhost", sitl=False))
        self.assertFalse(simulation_target_allowed("192.168.1.20", sitl=True))
        self.assertFalse(simulation_target_allowed("host.docker.internal", sitl=False))

    def _capture(self, transport: MavlinkDroneTransport) -> list[object]:
        sent: list[object] = []

        def collect(msg: object) -> None:
            sent.append(msg)

        transport._send_message = collect  # type: ignore[method-assign]
        return sent

    def test_clamp_helper(self) -> None:
        self.assertEqual(clamp(1.5, 0.0, 1.0), 1.0)
        self.assertEqual(clamp(-0.5, 0.0, 1.0), 0.0)
        self.assertEqual(clamp(0.5, 0.0, 1.0), 0.5)


if __name__ == "__main__":
    unittest.main()
