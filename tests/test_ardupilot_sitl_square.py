"""Square geometry, pass criteria, and the CLI refusal. No SITL, no socket."""

from __future__ import annotations

import io
import json
import math
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any, Callable

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
        self.readiness = square.copter_readiness_ok()
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


def _sys_status(*, present: bool, enabled: bool, healthy: bool) -> dict[str, int]:
    bit = square.MAV_SYS_STATUS_PREARM_CHECK
    return {
        "onboard_control_sensors_present": bit if present else 0,
        "onboard_control_sensors_enabled": bit if enabled else 0,
        "onboard_control_sensors_health": bit if healthy else 0,
    }


def _not_armable() -> square.CopterReadiness:
    """Messages have arrived, and every arm gate is still failing."""
    state = square.apply_mavlink_message(
        square.CopterReadiness(),
        "SYS_STATUS",
        _sys_status(present=True, enabled=True, healthy=False),
    )
    state = square.apply_mavlink_message(
        state,
        "EKF_STATUS_REPORT",
        {"flags": square.EKF_CONST_POS_MODE | square.EKF_UNINITIALIZED},
    )
    return square.apply_mavlink_message(state, "GPS_RAW_INT", {"fix_type": 1})


class ArmReadinessTests(unittest.TestCase):
    def test_blockers_name_prearm_ekf_and_gps(self) -> None:
        blocked = _not_armable()
        self.assertFalse(blocked.ready())
        self.assertEqual(
            blocked.blockers(),
            (
                "prearm check",
                "EKF attitude",
                "EKF horizontal velocity",
                "EKF vertical velocity",
                "EKF relative horizontal position",
                "EKF absolute horizontal position",
                "EKF absolute vertical position",
                "EKF predicted relative position",
                "EKF predicted absolute position",
                "EKF const position mode",
                "EKF uninitialized",
                "GPS fix 1",
            ),
        )
        ready = square.copter_readiness_ok()
        self.assertTrue(ready.ready())
        self.assertEqual(ready.blockers(), ())
        self.assertIsNotNone(ready.gps_fix_type)
        assert ready.gps_fix_type is not None
        self.assertGreaterEqual(ready.gps_fix_type, 3)
        self.assertEqual(
            square.CopterReadiness().blockers(),
            (
                "SYS_STATUS not received",
                "EKF_STATUS_REPORT not received",
                "GPS_RAW_INT not received",
            ),
        )

    def test_rtk_fix_is_ready_and_a_two_d_fix_is_not(self) -> None:
        ready = square.apply_mavlink_message(
            square.copter_readiness_ok(), "GPS_RAW_INT", {"fix_type": 6}
        )
        self.assertTrue(ready.ready())
        held = square.apply_mavlink_message(
            square.copter_readiness_ok(), "GPS_RAW_INT", {"fix_type": 2}
        )
        self.assertEqual(held.blockers(), ("GPS fix 2",))
        self.assertFalse(held.ready())

    def test_prearm_must_be_present_enabled_and_healthy(self) -> None:
        base = square.copter_readiness_ok()
        disabled = square.apply_mavlink_message(
            base,
            "SYS_STATUS",
            _sys_status(present=True, enabled=False, healthy=False),
        )
        self.assertEqual(disabled.blockers(), ("prearm check not enabled",))
        missing = square.apply_mavlink_message(
            base,
            "SYS_STATUS",
            _sys_status(present=False, enabled=False, healthy=False),
        )
        self.assertEqual(missing.blockers(), ("prearm check not present",))

    def test_ekf_error_bits_block_and_incomplete_messages_are_ignored(self) -> None:
        glitch = square.apply_mavlink_message(
            square.copter_readiness_ok(),
            "EKF_STATUS_REPORT",
            {"flags": square._ekf_ready_flags() | square.EKF_GPS_GLITCHING},
        )
        self.assertEqual(glitch.blockers(), ("EKF GPS glitch",))
        ignored = square.apply_mavlink_message(
            square.CopterReadiness(), "SYS_STATUS", {"load": 1}
        )
        self.assertFalse(ignored.sys_status_seen)
        ignored = square.apply_mavlink_message(
            ignored, "NOT_A_MESSAGE", {"flags": 1}
        )
        self.assertEqual(ignored, square.CopterReadiness())

    def test_interval_request_uses_a_positive_period_and_zero_param3(self) -> None:
        self.assertEqual(square.interval_us(2.0), 500_000)
        self.assertEqual(square.interval_us(10.0), 100_000)
        with self.assertRaises(ValueError):
            square.interval_us(0.0)
        fields = square.command_long_fields(
            target_system=1,
            message_id=193,
            interval_us_value=square.interval_us(2.0),
        )
        self.assertEqual(fields["command"], square.MAV_CMD_SET_MESSAGE_INTERVAL)
        self.assertEqual(fields["param1"], 193.0)
        self.assertEqual(fields["param2"], 500_000.0)
        self.assertEqual(fields["param3"], 0.0)
        period = fields["param2"]
        assert isinstance(period, float)
        self.assertGreater(period, 0.0)

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

    def test_arm_waits_until_ardupilot_is_ready(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = _not_armable()
        clock = FakeClock()
        real_sleep = clock.sleep

        def sleep(dt: float) -> None:
            real_sleep(dt)
            if clock.t >= 1.0:
                vehicle.readiness = square.copter_readiness_ok()

        clock.sleep = sleep  # type: ignore[method-assign]
        report, text, caught = self._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertEqual(vehicle.calls.count("arm"), 1)
        self.assertGreaterEqual(vehicle.arm_at[0], 1.0)
        self.assertLess(vehicle.calls.index("arm"), vehicle.calls.index("takeoff"))
        self.assertIn("not armable yet", text)
        self.assertIn("prearm check", text)
        self.assertIn("EKF attitude", text)
        self.assertIn("GPS fix 1", text)
        self.assertIn("autopilot is armable", text)

    def test_timeout_names_the_checks_and_does_not_arm(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.apply_mavlink_message(
            square.copter_readiness_ok(),
            "SYS_STATUS",
            _sys_status(present=True, enabled=True, healthy=False),
        )
        clock = FakeClock()
        report, text, caught = self._fly(vehicle, clock, arm_timeout_s=0.4)
        self.assertIsNone(report)
        assert caught is not None
        message = str(caught)
        self.assertIn("not armable within 0.4s", message)
        self.assertIn("prearm check", message)
        self.assertNotIn("EKF attitude", message)
        self.assertNotIn("GPS fix", message)
        self.assertNotIn("arm", vehicle.calls)
        self.assertIn("land", vehicle.calls)
        self.assertIn("not armable yet", text)

    def test_missing_messages_are_named(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.CopterReadiness()
        clock = FakeClock()
        _report, _text, caught = self._fly(vehicle, clock, arm_timeout_s=0.3)
        assert caught is not None
        message = str(caught)
        self.assertIn("SYS_STATUS not received", message)
        self.assertIn("EKF_STATUS_REPORT not received", message)
        self.assertIn("GPS_RAW_INT not received", message)
        self.assertNotIn("arm", vehicle.calls)

    def test_ready_vehicle_arms_without_a_wait(self) -> None:
        vehicle = FakeVehicle()
        vehicle.readiness = square.copter_readiness_ok()
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
            readiness=square.copter_readiness_ok(),
        )
        blockers = square._arm_blockers(sample)
        self.assertIn("link down", blockers)
        self.assertIn("landed_state is in_air", blockers)


class _DirectMessage:
    def __init__(self, message_name: str, fields: dict[str, object]) -> None:
        self.message_name = message_name
        self.fields_json = json.dumps(fields)

    @classmethod
    def from_mapping(cls, message_name: str, fields: dict[str, int]) -> "_DirectMessage":
        payload: dict[str, object] = {key: value for key, value in fields.items()}
        return cls(message_name, payload)


class _Direct:
    def __init__(self, system: object) -> None:
        self.system = system
        self.subscriptions: list[tuple[str, Callable[..., None]]] = []
        self.sent: list[Any] = []
        self.destroyed = False
        self.fail_names: set[str] = set()
        self.fail_send = False

    def subscribe_message(
        self,
        message_name: str,
        callback: Callable[..., None],
        user_data: object = None,
    ) -> None:
        if message_name in self.fail_names:
            raise RuntimeError(f"subscribe {message_name} failed")
        self.subscriptions.append((message_name, callback))
        self.user_data = user_data

    def send_message(self, message: object) -> None:
        if self.fail_send:
            raise RuntimeError("send failed")
        self.sent.append(message)

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

    def _attach(self, robot: _HealthRobot, direct: _Direct) -> square.CopterReadinessWatch:
        def factory(system: object) -> _Direct:
            direct.system = system
            return direct

        watch = square.CopterReadinessWatch()
        watch.attach(robot, direct_factory=factory)
        return watch

    def test_watch_uses_the_open_session_and_does_not_need_a_second_socket(self) -> None:
        robot, action = self._robot()
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        names = [name for name, _callback in direct.subscriptions]
        self.assertEqual(names, ["SYS_STATUS", "GPS_RAW_INT", "EKF_STATUS_REPORT"])
        self.assertIs(direct.system, action._owner)
        self.assertIsNone(direct.user_data)
        self.assertIn("reading ArduPilot SYS_STATUS", watch.note)
        self.assertFalse(watch.poll().ready())
        self.assertIn("SYS_STATUS not received", watch.poll().blockers())
        callback = direct.subscriptions[0][1]
        assert callable(callback)
        callback(
            _DirectMessage.from_mapping(
                "SYS_STATUS",
                _sys_status(present=True, enabled=True, healthy=True),
            ),
            None,
        )
        self.assertNotIn("SYS_STATUS not received", watch.poll().blockers())
        self.assertNotIn("prearm check", watch.poll().blockers())
        watch.close()
        self.assertTrue(direct.destroyed)

    def test_one_failed_subscription_still_reads_the_others(self) -> None:
        robot, _action = self._robot()
        direct = _Direct(object())
        direct.fail_names.add("EKF_STATUS_REPORT")
        watch = self._attach(robot, direct)
        self.assertIn("EKF_STATUS_REPORT", watch.note)
        names = [name for name, _callback in direct.subscriptions]
        self.assertIn("SYS_STATUS", names)
        self.assertIn("GPS_RAW_INT", names)
        watch.close()
        self.assertTrue(direct.destroyed)

    def test_messages_become_ready_and_intervals_keep_param3_zero(self) -> None:
        robot, _action = self._robot()
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        self.assertGreaterEqual(len(direct.sent), len(square.STREAM_REQUESTS))
        requested: set[float] = set()
        for message in direct.sent:
            fields = json.loads(message.fields_json)
            self.assertEqual(message.message_name, "COMMAND_LONG")
            self.assertEqual(message.system_id, square.MAVSDK_GCS_SYSTEM_ID)
            self.assertEqual(message.component_id, square.MAVSDK_GCS_COMPONENT_ID)
            self.assertEqual(fields["param3"], 0.0)
            self.assertGreater(fields["param2"], 0.0)
            self.assertEqual(fields["command"], square.MAV_CMD_SET_MESSAGE_INTERVAL)
            requested.add(fields["param1"])
        for _name, message_id, _rate in square.STREAM_REQUESTS:
            self.assertIn(float(message_id), requested)
        self.assertEqual(watch.request_streams(now=1.0), 0)
        sent_before = len(direct.sent)
        self.assertEqual(watch.request_streams(now=2.0), len(square.STREAM_REQUESTS))
        self.assertEqual(len(direct.sent), sent_before + len(square.STREAM_REQUESTS))
        by_name = {name: callback for name, callback in direct.subscriptions}
        by_name["SYS_STATUS"](
            _DirectMessage.from_mapping(
                "SYS_STATUS",
                _sys_status(present=True, enabled=True, healthy=True),
            ),
            None,
        )
        by_name["EKF_STATUS_REPORT"](
            _DirectMessage.from_mapping(
                "EKF_STATUS_REPORT", {"flags": square._ekf_ready_flags()}
            ),
            None,
        )
        by_name["GPS_RAW_INT"](
            _DirectMessage.from_mapping("GPS_RAW_INT", {"fix_type": 3}),
            None,
        )
        self.assertTrue(watch.poll().ready())
        watch.close()

    def test_missing_session_is_named(self) -> None:
        watch = square.CopterReadinessWatch()
        watch.attach(object())
        self.assertIn("no autopilot system", watch.note)
        self.assertIn("SYS_STATUS not received", watch.poll().blockers())
        self.assertEqual(watch.request_streams(now=0.0), 0)

    def test_published_mavsdk_health_does_not_override_the_prearm_bit(self) -> None:
        robot, _action = self._robot()
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        robot.custom["arm_health"] = {
            "is_armable": True,
            "is_global_position_ok": True,
            "is_home_position_ok": True,
        }
        vehicle = square.OhhoVehicle(robot, object(), health=watch)
        sample = vehicle.read()
        self.assertFalse(sample.readiness.ready())
        self.assertIn("SYS_STATUS not received", sample.readiness.blockers())
        self.assertIn("reading ArduPilot SYS_STATUS", vehicle.arm_health_note)
        vehicle.close()
        self.assertTrue(direct.destroyed)

    def test_send_failure_is_logged_and_does_not_arm(self) -> None:
        robot, _action = self._robot()
        direct = _Direct(object())
        direct.fail_send = True
        buf = io.StringIO()
        with redirect_stdout(buf):
            watch = self._attach(robot, direct)
        self.assertIn("message interval request failed", buf.getvalue())
        self.assertFalse(watch.poll().ready())
        watch.close()


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
        self.assertIn("pre-arm", result.stdout)
        self.assertIn("EKF", result.stdout)
        self.assertIn("GPS fix_type", result.stdout)
        self.assertNotIn("is_armable", result.stdout)
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
