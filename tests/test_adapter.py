"""Unit tests for MavlinkDroneTransport and OhhO OS schema adapters."""

from __future__ import annotations

import unittest

from ohho_drone.adapter import MavlinkDroneTransport
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

    def test_clamp_helper(self) -> None:
        self.assertEqual(clamp(1.5, 0.0, 1.0), 1.0)
        self.assertEqual(clamp(-0.5, 0.0, 1.0), 0.0)
        self.assertEqual(clamp(0.5, 0.0, 1.0), 0.5)


if __name__ == "__main__":
    unittest.main()
