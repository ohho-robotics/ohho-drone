"""Square geometry, pass criteria, and the CLI refusal. No SITL, no socket."""

from __future__ import annotations

import io
import math
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
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
        self.arm_at: list[float] = []
        self.arm_rejects = 0
        self.readiness = square.arm_readiness_ok()
        self.clock: FakeClock | None = None

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
            readiness=self.readiness,
        )

    def arm(self) -> None:
        self.calls.append("arm")
        if self.clock is not None:
            self.arm_at.append(self.clock.t)
        if self.arm_rejects > 0:
            self.arm_rejects -= 1
            raise RuntimeError("FAILED; origin: arm(); params: ()")
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


def _not_armable() -> square.ArmReadiness:
    return square.ArmReadiness(
        reported=True,
        is_armable=False,
        is_global_position_ok=False,
        is_home_position_ok=False,
        is_local_position_ok=True,
        is_accelerometer_calibration_ok=True,
        is_gyrometer_calibration_ok=False,
        is_magnetometer_calibration_ok=True,
    )


class ArmReadinessTests(unittest.TestCase):
    def test_detail_names_failed_checks_and_ready_ignores_calibration(self) -> None:
        blocked = _not_armable()
        self.assertFalse(blocked.ready())
        self.assertEqual(
            blocked.blockers(),
            ("is_armable", "global position", "home position"),
        )
        self.assertIn("gyrometer calibration", blocked.detail())
        self.assertNotIn("accelerometer calibration", blocked.detail())
        ready = square.ArmReadiness(
            reported=True,
            is_armable=True,
            is_global_position_ok=True,
            is_home_position_ok=True,
            is_magnetometer_calibration_ok=False,
        )
        self.assertTrue(ready.ready())
        self.assertEqual(ready.blockers(), ())
        self.assertEqual(ready.detail(), ("magnetometer calibration",))
        self.assertEqual(
            square.ArmReadiness().blockers(),
            ("mavsdk health not received",),
        )

    def _fly(
        self,
        vehicle: FakeVehicle,
        clock: FakeClock,
        *,
        arm_timeout_s: float = square.ARM_TIMEOUT_S,
        arm_attempts: int = square.ARM_ATTEMPTS,
    ) -> tuple[square.SquareReport | None, str, BaseException | None]:
        vehicle.clock = clock
        buf = io.StringIO()
        caught: BaseException | None = None
        report: square.SquareReport | None = None
        with redirect_stdout(buf):
            try:
                report = square.fly_square(
                    vehicle,
                    sleep=clock.sleep,
                    monotonic=clock,
                    arm_timeout_s=arm_timeout_s,
                    arm_attempts=arm_attempts,
                    arm_retry_pause_s=0.0,
                )
            except square.SquareFailed as exc:
                caught = exc
        return report, buf.getvalue(), caught

    def test_arm_waits_until_health_is_ready(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = _not_armable()
        clock = FakeClock()
        real_sleep = clock.sleep

        def sleep(dt: float) -> None:
            real_sleep(dt)
            if clock.t >= 1.0:
                vehicle.readiness = square.arm_readiness_ok()

        clock.sleep = sleep  # type: ignore[method-assign]
        report, text, caught = self._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertEqual(vehicle.calls.count("arm"), 1)
        self.assertGreaterEqual(vehicle.arm_at[0], 1.0)
        self.assertLess(vehicle.calls.index("arm"), vehicle.calls.index("takeoff"))
        self.assertIn("not armable yet", text)
        self.assertIn("is_armable", text)
        self.assertIn("global position", text)
        self.assertIn("home position", text)
        self.assertIn("gyrometer calibration", text)
        self.assertIn("autopilot is armable", text)

    def test_timeout_names_the_checks_and_does_not_arm(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.ArmReadiness(
            reported=True,
            is_armable=False,
            is_global_position_ok=True,
            is_home_position_ok=False,
            is_gyrometer_calibration_ok=False,
        )
        clock = FakeClock()
        report, text, caught = self._fly(vehicle, clock, arm_timeout_s=0.4)
        self.assertIsNone(report)
        assert caught is not None
        message = str(caught)
        self.assertIn("not armable within 0.4s", message)
        self.assertIn("is_armable", message)
        self.assertIn("home position", message)
        self.assertIn("gyrometer calibration", message)
        self.assertNotIn("global position", message)
        self.assertNotIn("arm", vehicle.calls)
        self.assertIn("land", vehicle.calls)
        self.assertIn("not armable yet", text)

    def test_missing_health_is_named(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.ArmReadiness()
        clock = FakeClock()
        _report, _text, caught = self._fly(vehicle, clock, arm_timeout_s=0.3)
        assert caught is not None
        self.assertIn("mavsdk health not received", str(caught))
        self.assertNotIn("arm", vehicle.calls)

    def test_false_calibration_does_not_block_an_armable_vehicle(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.ArmReadiness(
            reported=True,
            is_armable=True,
            is_global_position_ok=True,
            is_home_position_ok=True,
            is_local_position_ok=False,
            is_accelerometer_calibration_ok=False,
            is_gyrometer_calibration_ok=False,
            is_magnetometer_calibration_ok=False,
        )
        clock = FakeClock()
        report, text, caught = self._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertEqual(vehicle.calls.count("arm"), 1)
        self.assertNotIn("not armable yet", text)

    def test_rejected_arm_is_retried_then_succeeds(self) -> None:
        vehicle = FakeVehicle()
        vehicle.arm_rejects = 2
        clock = FakeClock()
        report, text, caught = self._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertEqual(vehicle.calls.count("arm"), 3)
        self.assertIn("arming autopilot (attempt 1/3)", text)
        self.assertIn("arming autopilot (attempt 3/3)", text)
        self.assertIn("arm rejected:", text)
        self.assertIn("FAILED; origin: arm()", text)

    def test_arm_retries_are_bounded(self) -> None:
        vehicle = FakeVehicle()
        vehicle.arm_rejects = 10
        clock = FakeClock()
        _report, text, caught = self._fly(vehicle, clock, arm_attempts=3)
        assert caught is not None
        self.assertEqual(vehicle.calls.count("arm"), 3)
        self.assertNotIn("takeoff", vehicle.calls)
        self.assertIn("land", vehicle.calls)
        self.assertIn("after 3 attempts", str(caught))
        self.assertIn("arm rejected:", text)

    def test_link_and_airborne_state_block_arm(self) -> None:
        sample = square.VehicleSample(
            x=0.0,
            y=0.0,
            z=0.0,
            armed=False,
            landed_state="in_air",
            flight_mode="stabilize",
            failsafe=(),
            link_ok=False,
            readiness=square.arm_readiness_ok(),
        )
        blockers = square._arm_blockers(sample)
        self.assertIn("link down", blockers)
        self.assertIn("landed_state is in_air", blockers)


class _Health:
    def __init__(self, *, armable: bool) -> None:
        self.is_armable = armable
        self.is_global_position_ok = armable
        self.is_home_position_ok = armable
        self.is_local_position_ok = True
        self.is_accelerometer_calibration_ok = True
        self.is_gyrometer_calibration_ok = armable
        self.is_magnetometer_calibration_ok = True


class _Plugin:
    def __init__(self, system: object) -> None:
        self.system = system
        self.health_value = _Health(armable=False)
        self.subscribed = False
        self.destroyed = False
        self.fail_subscribe = False

    def health(self) -> _Health:
        return self.health_value

    def subscribe_health(self, callback: object, user_data: object = None) -> None:
        if self.fail_subscribe:
            raise RuntimeError("subscribe failed")
        self.subscribed = True
        self.callback = callback
        self.user_data = user_data
        if callable(callback):
            callback(self.health_value, user_data)

    def destroy(self) -> None:
        self.destroyed = True


class _Owner:
    _handle = object()


class _Action:
    def __init__(self) -> None:
        self._owner = _Owner()


class _Link:
    def __init__(self) -> None:
        self._action = _Action()
        self._sdk = None


class _Transport:
    def __init__(self) -> None:
        self._link = _Link()


class _HealthRobot:
    """Loopback stand-in with the flight methods ``OhhoVehicle`` calls."""

    def __init__(self, transport: _Transport) -> None:
        self.transport = transport
        self.custom: dict[str, object] = {
            "drone": {
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0},
                "armed": False,
                "landed_state": "on_ground",
                "flight_mode": "stabilize",
                "failsafe": (),
                "link_ok": True,
            }
        }
        self.calls: list[tuple[object, ...]] = []

    def telemetry(self) -> object:
        return type("Tele", (), {"custom": self.custom})()

    def arm_flight(self) -> None:
        self.calls.append(("arm_flight",))

    def takeoff(self, alt_m: float) -> None:
        self.calls.append(("takeoff", alt_m))

    def offboard_position(
        self, x: float, y: float, z: float, yaw: float, frame: object
    ) -> None:
        self.calls.append(("offboard_position", x, y, z, yaw, frame))

    def land(self) -> None:
        self.calls.append(("land",))

    def disarm_flight(self) -> None:
        self.calls.append(("disarm_flight",))

    def disconnect(self) -> None:
        self.calls.append(("disconnect",))


class HealthWatchTests(unittest.TestCase):
    def _robot(self) -> tuple[_HealthRobot, _Action]:
        action = _Action()
        link = _Link()
        link._action = action
        transport = _Transport()
        transport._link = link
        return _HealthRobot(transport), action

    def test_watch_uses_the_open_session_and_does_not_need_a_second_socket(self) -> None:
        robot, action = self._robot()
        plugins: list[_Plugin] = []

        def factory(system: object) -> _Plugin:
            plugin = _Plugin(system)
            plugins.append(plugin)
            return plugin

        watch = square.ArmHealthWatch()
        watch.attach(robot, telemetry_factory=factory)
        self.assertEqual(len(plugins), 1)
        self.assertIs(plugins[0].system, action._owner)
        self.assertTrue(plugins[0].subscribed)
        self.assertIsNone(plugins[0].user_data)
        self.assertIn("reading MAVSDK health", watch.note)
        self.assertFalse(watch.poll().ready())
        self.assertIn("is_armable", watch.poll().detail())
        self.assertIn("gyrometer calibration", watch.poll().detail())
        watch.close()
        self.assertTrue(plugins[0].destroyed)

    def test_subscribe_failure_still_polls_health(self) -> None:
        robot, action = self._robot()
        plugin = _Plugin(action._owner)
        plugin.fail_subscribe = True
        plugin.health_value = _Health(armable=True)

        def factory(_system: object) -> _Plugin:
            return plugin

        watch = square.ArmHealthWatch()
        watch.attach(robot, telemetry_factory=factory)
        self.assertIn("subscribe_health failed", watch.note)
        self.assertTrue(watch.poll().ready())
        watch.close()
        self.assertTrue(plugin.destroyed)

    def test_missing_session_is_named(self) -> None:
        watch = square.ArmHealthWatch()
        watch.attach(object())
        self.assertIn("no autopilot system", watch.note)
        self.assertEqual(watch.poll().blockers(), ("mavsdk health not received",))

    def test_ohho_arm_health_field_is_preferred(self) -> None:
        robot, action = self._robot()
        plugin = _Plugin(action._owner)
        plugin.health_value = _Health(armable=True)

        def factory(_system: object) -> _Plugin:
            return plugin

        watch = square.ArmHealthWatch()
        watch.attach(robot, telemetry_factory=factory)
        robot.custom["arm_health"] = {
            "is_armable": False,
            "is_global_position_ok": True,
            "is_home_position_ok": True,
        }
        vehicle = square.OhhoVehicle(robot, object(), health=watch)
        sample = vehicle.read()
        self.assertFalse(sample.readiness.ready())
        self.assertIn("is_armable", sample.readiness.blockers())
        self.assertNotIn("global position", sample.readiness.blockers())
        vehicle.close()
        self.assertTrue(plugin.destroyed)

    def test_plugin_health_is_used_when_ohho_does_not_publish_it(self) -> None:
        robot, action = self._robot()
        plugin = _Plugin(action._owner)
        plugin.health_value = _Health(armable=True)

        def factory(_system: object) -> _Plugin:
            return plugin

        watch = square.ArmHealthWatch()
        watch.attach(robot, telemetry_factory=factory)
        vehicle = square.OhhoVehicle(robot, object(), health=watch)
        sample = vehicle.read()
        self.assertTrue(sample.readiness.ready())
        self.assertIn("reading MAVSDK health", vehicle.arm_health_note)
        vehicle.close()


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
        self.assertIn("is_armable", result.stdout)
        self.assertIn("global position", result.stdout)
        self.assertIn("home position", result.stdout)
        self.assertNotIn("OHHO_ARM_HARDWARE=", result.stdout)

    def test_script_does_not_skip_arming_checks(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("arm_force", text)
        self.assertNotIn("ARMING_CHECK", text)
        self.assertNotIn("ARMING_SKIPCHK", text)
        self.assertEqual(square.ARM_TIMEOUT_S, 90.0)
        self.assertEqual(square.ARM_ATTEMPTS, 3)
        self.assertLessEqual(square.ARM_ATTEMPTS, 5)

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
