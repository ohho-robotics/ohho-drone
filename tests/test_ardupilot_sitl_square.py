"""Square geometry, pass criteria, and the CLI refusal. No SITL, no socket."""

from __future__ import annotations

import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ardupilot_sitl_square.py"
sys.path.insert(0, str(ROOT / "scripts"))

import ardupilot_sitl_square as square  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


class FakeVehicle:
    def __init__(self, *, track: bool = True, failsafe_on_goto: bool = False) -> None:
        self.track = track
        self.failsafe_on_goto = failsafe_on_goto
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.armed = False
        self.landed_state = "on_ground"
        self.flight_mode = "stabilize"
        self.failsafe: tuple[str, ...] = ()
        self.link_ok = True
        self.calls: list[str] = []

    def read(self) -> square.VehicleSample:
        return square.VehicleSample(
            x=self.x,
            y=self.y,
            z=self.z,
            armed=self.armed,
            landed_state=self.landed_state,
            flight_mode=self.flight_mode,
            failsafe=self.failsafe,
            link_ok=self.link_ok,
        )

    def arm(self) -> None:
        self.calls.append("arm")
        self.armed = True

    def takeoff(self, alt_m: float) -> None:
        self.calls.append("takeoff")
        self.z = -float(alt_m)
        self.landed_state = "in_air"
        self.flight_mode = "guided"

    def goto(self, x: float, y: float, z: float, yaw: float) -> None:
        self.calls.append("goto")
        if self.failsafe_on_goto:
            self.failsafe = ("geofence",)
        if self.track:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    def land(self) -> None:
        self.calls.append("land")
        self.z = 0.0
        self.landed_state = "on_ground"
        self.flight_mode = "land"

    def disarm(self) -> None:
        self.calls.append("disarm")
        self.armed = False

    def close(self) -> None:
        self.calls.append("close")


class Recorder:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def telemetry(self):
        return type("Tele", (), {})()

    def arm_flight(self) -> None:
        self.calls.append(("arm_flight",))

    def takeoff(self, alt_m: float) -> None:
        self.calls.append(("takeoff", alt_m))

    def offboard_position(self, x, y, z, yaw, frame) -> None:
        self.calls.append(("offboard_position", x, y, z, yaw, frame))

    def land(self) -> None:
        self.calls.append(("land",))

    def disarm_flight(self) -> None:
        self.calls.append(("disarm_flight",))

    def disconnect(self) -> None:
        self.calls.append(("disconnect",))


class GeometryTests(unittest.TestCase):
    def test_takeoff_band_is_strict(self) -> None:
        self.assertEqual(square.ALT_TOL_M, 0.4)
        self.assertTrue(square.in_takeoff_band(-2.5))
        self.assertTrue(square.in_takeoff_band(-2.11))
        self.assertFalse(square.in_takeoff_band(-2.09))
        self.assertFalse(square.in_takeoff_band(-2.91))
        # The D-07 predicate is strict. An error of exactly 0.4 m is outside.
        self.assertFalse(0.4 < square.ALT_TOL_M)

    def test_corner_tolerance_is_inclusive(self) -> None:
        self.assertTrue(square.within_corner(0.8))
        self.assertFalse(square.within_corner(0.81))

    def test_square_is_four_metre_sides_and_closes(self) -> None:
        corners = square.square_corners(1.0, 2.0, -2.5)
        self.assertEqual(len(corners), 4)
        self.assertEqual(corners[-1], (1.0, 2.0, -2.5))
        previous = (1.0, 2.0, -2.5)
        for corner in corners:
            self.assertAlmostEqual(math.dist(previous[:2], corner[:2]), 4.0)
            self.assertEqual(corner[2], -2.5)
            previous = corner

    def test_carrot_moves_at_half_a_metre_per_second(self) -> None:
        start = (0.0, 0.0, -2.5)
        end = (4.0, 0.0, -2.5)
        points = square.carrot_points(start, end)
        self.assertEqual(len(points), 80)
        self.assertEqual(points[-1], end)
        step = math.dist(start[:2], points[0][:2])
        self.assertAlmostEqual(step * square.STREAM_HZ, square.SPEED_MPS)
        for earlier, later in zip(points, points[1:]):
            self.assertAlmostEqual(math.dist(earlier[:2], later[:2]), step)
            self.assertEqual(later[2], -2.5)


class FlightTests(unittest.TestCase):
    def test_square_passes_and_disarms_only_after_land(self) -> None:
        vehicle = FakeVehicle()
        clock = FakeClock()
        report = square.fly_square(vehicle, sleep=clock.sleep, monotonic=clock)
        self.assertTrue(report.passed)
        self.assertEqual(len(report.corner_errors_m), 4)
        self.assertTrue(all(error <= 0.8 for error in report.corner_errors_m))
        self.assertEqual(report.landed_state, "on_ground")
        self.assertFalse(report.armed)
        self.assertEqual(report.failsafe, [])
        self.assertLess(vehicle.calls.index("arm"), vehicle.calls.index("takeoff"))
        self.assertLess(vehicle.calls.index("takeoff"), vehicle.calls.index("goto"))
        self.assertLess(vehicle.calls.index("goto"), vehicle.calls.index("land"))
        self.assertLess(vehicle.calls.index("land"), vehicle.calls.index("disarm"))
        self.assertEqual(vehicle.calls.count("disarm"), 1)
        self.assertGreater(clock.t, square.ALT_HOLD_S + 4 * square.CORNER_HOLD_S)

    def test_missed_corner_lands_without_disarm(self) -> None:
        vehicle = FakeVehicle(track=False)
        clock = FakeClock()
        with self.assertRaises(square.SquareFailed):
            square.fly_square(
                vehicle,
                sleep=clock.sleep,
                monotonic=clock,
                corner_timeout_s=0.5,
            )
        self.assertIn("land", vehicle.calls)
        self.assertNotIn("disarm", vehicle.calls)

    def test_failsafe_flag_lands_without_disarm(self) -> None:
        vehicle = FakeVehicle(failsafe_on_goto=True)
        clock = FakeClock()
        with self.assertRaises(square.SquareFailed) as caught:
            square.fly_square(vehicle, sleep=clock.sleep, monotonic=clock)
        self.assertIn("failsafe", str(caught.exception))
        self.assertIn("land", vehicle.calls)
        self.assertNotIn("disarm", vehicle.calls)

    def test_ohho_vehicle_uses_local_frame_and_autopilot_arm(self) -> None:
        robot = Recorder()
        frame = object()
        vehicle = square.OhhoVehicle(robot, frame)
        vehicle.arm()
        vehicle.takeoff(2.5)
        vehicle.goto(4.0, 0.0, -2.5, 0.0)
        vehicle.land()
        vehicle.disarm()
        vehicle.close()
        self.assertEqual(robot.calls[0], ("arm_flight",))
        self.assertEqual(robot.calls[1], ("takeoff", 2.5))
        self.assertEqual(robot.calls[2][0], "offboard_position")
        self.assertIs(robot.calls[2][5], frame)
        self.assertEqual(robot.calls[3], ("land",))
        self.assertEqual(robot.calls[4], ("disarm_flight",))
        self.assertEqual(robot.calls[5], ("disconnect",))


class CliTests(unittest.TestCase):
    def _run(self, args: list[str], env: dict[str, str] | None = None):
        merged = os_environ_without_arm()
        if env:
            merged.update(env)
        with tempfile.TemporaryDirectory() as tmp:
            return subprocess.run(
                [sys.executable, str(SCRIPT), *args],
                cwd=tmp,
                env=merged,
                capture_output=True,
                text=True,
                check=False,
            )

    def test_default_prints_recipe(self) -> None:
        result = self._run([])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(square.SITL_URI, result.stdout)
        self.assertIn(square.MAVSDK_URL, result.stdout)
        self.assertIn(square.COPTER_TAG, result.stdout)
        self.assertIn(square.OHHO_PIP_URL, result.stdout)
        self.assertIn("ohho-os[mavlink] @ ", result.stdout)
        self.assertNotIn("ohho[mavlink]", result.stdout)
        self.assertIn("2.5", result.stdout)
        self.assertIn("4", result.stdout)
        self.assertIn("0.8", result.stdout)
        self.assertIn("sim_vehicle.py", result.stdout)
        self.assertNotIn("OHHO_ARM_HARDWARE=", result.stdout)

    def test_arm_flag_is_refused(self) -> None:
        result = self._run(["--arm"])
        self.assertEqual(result.returncode, 2)
        self.assertIn("refusing --arm", result.stderr)

    def test_fly_with_hardware_env_is_refused(self) -> None:
        result = self._run(["--fly"], env={"OHHO_ARM_HARDWARE": "1"})
        self.assertEqual(result.returncode, 2)
        self.assertIn("OHHO_ARM_HARDWARE", result.stderr)
        self.assertNotIn("square-result.json", result.stderr)


def os_environ_without_arm() -> dict[str, str]:
    import os

    env = dict(os.environ)
    env.pop("OHHO_ARM_HARDWARE", None)
    return env


if __name__ == "__main__":
    unittest.main()
