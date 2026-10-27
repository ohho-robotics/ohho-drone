"""Square geometry, pass criteria, and the CLI refusal. No SITL, no socket."""

from __future__ import annotations

import io
import json
import math
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from collections.abc import Callable
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any

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

    def arm_gate(self) -> None:
        self.calls.append("arm_gate")

    def stream_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        self.calls.append("stream_setpoint")

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


class _HeartbeatLagVehicle(FakeVehicle):
    """Arm command returns before the HEARTBEAT armed bit is visible."""

    def __init__(self, lag_s: float) -> None:
        super().__init__()
        self.lag_s = lag_s
        self._armed_at: float | None = None
        self._latched = False
        self.takeoff_at = 0.0

    def arm(self) -> None:
        self.calls.append("arm")
        if self.clock is not None:
            self.arm_at.append(self.clock.t)
            self._armed_at = self.clock.t + self.lag_s
        if self.arm_rejects > 0:
            self.arm_rejects -= 1
            raise RuntimeError("FAILED; origin: arm(); params: ()")

    def read(self) -> square.VehicleSample:
        if (
            not self._latched
            and self._armed_at is not None
            and self.clock is not None
            and self.clock.t + 1e-9 >= self._armed_at
        ):
            self.armed = True
            self._latched = True
        return super().read()

    def takeoff(self, alt_m: float) -> None:
        if self.clock is not None:
            self.takeoff_at = self.clock.t
        super().takeoff(alt_m)


class Recorder:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def telemetry(self):
        return type("Tele", (), {})()

    def arm(self) -> bool:
        self.calls.append(("arm",))
        return True

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

    def test_square_fits_the_aerial_gate_limits(self) -> None:
        # ohh-305 aerial profile: takeoff cap 3 m, fence altitude 3 m, fence
        # radius 10 m, horizontal speed cap 1.5 m/s, setpoint window 300 ms.
        self.assertLessEqual(square.TAKEOFF_ALT_M, 3.0)
        self.assertLessEqual(square.SPEED_MPS, 1.5)
        self.assertLess(math.hypot(square.SIDE_M, square.SIDE_M), 10.0)
        self.assertLess(1.0 / square.STREAM_HZ, 0.3)


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
        arm_at = vehicle.calls.index("arm")
        takeoff_at = vehicle.calls.index("takeoff")
        self.assertLess(vehicle.calls.index("arm_gate"), vehicle.calls.index("stream_setpoint"))
        self.assertLess(vehicle.calls.index("stream_setpoint"), arm_at)
        self.assertGreater(
            vehicle.calls[arm_at + 1 : takeoff_at].count("stream_setpoint"), 1
        )
        self.assertLess(arm_at, takeoff_at)
        self.assertLess(takeoff_at, vehicle.calls.index("goto"))
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

    def test_stale_heartbeat_armed_bit_is_not_an_arm_rejection(self) -> None:
        """Copter HEARTBEAT is 1 Hz. A sample at 0.5 s can still read disarmed."""
        vehicle = _HeartbeatLagVehicle(lag_s=1.0)
        clock = FakeClock()
        report, text, caught = self._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertEqual(vehicle.calls.count("arm"), 1)
        self.assertNotIn("arm rejected:", text)
        waited = vehicle.takeoff_at - vehicle.arm_at[0]
        self.assertGreater(waited, square.ARM_SETTLE_S)
        self.assertAlmostEqual(waited, 1.0, delta=1e-6)
        self.assertLess(waited, square.ARM_CONFIRM_S)

    def test_armed_bit_that_never_arrives_is_still_rejected(self) -> None:
        vehicle = _HeartbeatLagVehicle(lag_s=square.ARM_CONFIRM_S + 5.0)
        clock = FakeClock()
        _report, text, caught = self._fly(vehicle, clock, arm_attempts=1)
        assert caught is not None
        self.assertIn("stayed disarmed", str(caught))
        self.assertIn("arm rejected:", text)
        self.assertNotIn("takeoff", vehicle.calls)
        self.assertEqual(vehicle.calls.count("arm"), 1)

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

    def test_disarmed_prearm_telemetry_failure_does_not_land(self) -> None:
        vehicle = FakeVehicle()
        vehicle.landed_state = "unknown"
        clock = FakeClock()
        err = io.StringIO()
        with redirect_stderr(err):
            with self.assertRaises(square.SquareFailed) as caught:
                square.fly_square(
                    vehicle,
                    sleep=clock.sleep,
                    monotonic=clock,
                    ground_timeout_s=0.2,
                )
        self.assertIn("no on_ground telemetry before arm", str(caught.exception))
        self.assertNotIn("land", vehicle.calls)
        self.assertNotIn("arm", vehicle.calls)
        self.assertIn("not landing", err.getvalue())
        payload = square._failure_payload(caught.exception, 1.25)
        self.assertFalse(payload["passed"])
        self.assertTrue(payload["attempted"])
        self.assertIn(
            "no on_ground telemetry before arm", str(payload["message"])
        )

    def test_armed_vehicle_still_lands_when_the_ground_wait_fails(self) -> None:
        vehicle = FakeVehicle()
        vehicle.landed_state = "unknown"
        vehicle.armed = True
        clock = FakeClock()
        with self.assertRaises(square.SquareFailed):
            square.fly_square(
                vehicle,
                sleep=clock.sleep,
                monotonic=clock,
                ground_timeout_s=0.2,
            )
        self.assertIn("land", vehicle.calls)
        self.assertNotIn("disarm", vehicle.calls)


class _PositionGateVehicle(FakeVehicle):
    """Reports ``position_fed`` the way ``OhhoVehicle`` does."""

    def __init__(self, *, ready_on_read: int) -> None:
        super().__init__()
        self.ready_on_read = ready_on_read
        self.reads = 0
        self._position_fed = False

    @property
    def position_fed(self) -> bool:
        return self._position_fed

    def read(self) -> square.VehicleSample:
        self.reads += 1
        self._position_fed = self.reads >= self.ready_on_read
        return super().read()


class _RefuseGateArm(FakeVehicle):
    def __init__(self, *, autopilot_armed: bool) -> None:
        super().__init__()
        self.armed = autopilot_armed

    def arm_gate(self) -> None:
        self.calls.append("arm_gate")
        raise square.SquareFailed(
            "SafetyGate.arm() refused the gate: no_fresh_state"
        )


class ArmFailurePathTests(unittest.TestCase):
    def test_arm_waits_until_a_new_position_sample_is_fed(self) -> None:
        vehicle = _PositionGateVehicle(ready_on_read=3)
        clock = FakeClock()
        report, _text, caught = ArmReadinessTests()._fly(vehicle, clock)
        self.assertIsNone(caught)
        assert report is not None
        self.assertTrue(report.passed)
        self.assertGreaterEqual(vehicle.reads, 3)
        self.assertIn("arm_gate", vehicle.calls)
        self.assertGreaterEqual(clock.t, 1.0 / square.STREAM_HZ)

    def test_missing_position_sample_fails_inside_the_arm_timeout(self) -> None:
        vehicle = _PositionGateVehicle(ready_on_read=10**9)
        clock = FakeClock()
        _report, text, caught = ArmReadinessTests()._fly(
            vehicle, clock, arm_timeout_s=0.25
        )
        assert caught is not None
        self.assertIn(square._FRESH_POSITION_BLOCKER, str(caught))
        self.assertIn("not armable", text)
        self.assertNotIn("arm_gate", vehicle.calls)
        payload = square._failure_payload(caught, clock.t)
        self.assertIn(square._FRESH_POSITION_BLOCKER, str(payload["message"]))

    def test_disarmed_gate_arm_refusal_does_not_land(self) -> None:
        vehicle = _RefuseGateArm(autopilot_armed=False)
        clock = FakeClock()
        err = io.StringIO()
        with redirect_stderr(err):
            with self.assertRaises(square.SquareFailed) as caught:
                square.fly_square(vehicle, sleep=clock.sleep, monotonic=clock)
        self.assertIn("SafetyGate.arm() refused the gate", str(caught.exception))
        self.assertIn("no_fresh_state", str(caught.exception))
        self.assertNotIn("land", vehicle.calls)
        self.assertIn("arm_gate", vehicle.calls)
        self.assertIn("not landing", err.getvalue())
        self.assertIn("safety gate arm was refused", err.getvalue())
        payload = square._failure_payload(caught.exception, 40.212)
        self.assertFalse(payload["passed"])
        self.assertTrue(payload["attempted"])
        self.assertIn("no_fresh_state", str(payload["message"]))

    def test_gate_arm_refusal_still_lands_when_the_autopilot_is_armed(self) -> None:
        vehicle = _RefuseGateArm(autopilot_armed=True)
        clock = FakeClock()
        with self.assertRaises(square.SquareFailed):
            square.fly_square(vehicle, sleep=clock.sleep, monotonic=clock)
        self.assertIn("land", vehicle.calls)
        self.assertNotIn("disarm", vehicle.calls)


class _DirectMessage:
    def __init__(self, message_name: str, fields: dict[str, object]) -> None:
        self.message_name = message_name
        self.fields_json = json.dumps(fields)

    @classmethod
    def from_mapping(cls, message_name: str, fields: dict[str, int]) -> _DirectMessage:
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

    def arm(self) -> bool:
        self.calls.append(("arm",))
        return True

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
        self.assertEqual(
            names,
            [
                "SYS_STATUS",
                "GPS_RAW_INT",
                "EKF_STATUS_REPORT",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
                "HEARTBEAT",
                "EXTENDED_SYS_STATE",
                "COMMAND_ACK",
            ],
        )
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

    def test_watch_reads_the_system_through_the_wrapped_inner_transport(self) -> None:
        robot, action = self._robot()
        inner = robot.transport
        robot.transport = type("Gate", (), {"_inner": inner})()
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        self.assertIs(direct.system, action._owner)
        self.assertIn("reading ArduPilot SYS_STATUS", watch.note)
        watch.close()
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

    def test_unknown_is_not_ground_until_copter_says_landed(self) -> None:
        self.assertEqual(
            square.landed_state_from_copter_heartbeat({"system_status": 3}),
            "on_ground",
        )
        self.assertEqual(
            square.landed_state_from_copter_heartbeat({"system_status": 4}),
            "in_air",
        )
        self.assertEqual(
            square.landed_state_from_copter_heartbeat({"system_status": 1}),
            "unknown",
        )
        self.assertEqual(
            square.landed_state_from_copter_heartbeat({"system_status": 5}),
            "unknown",
        )
        self.assertIsNone(square.landed_state_from_copter_heartbeat({}))
        self.assertEqual(
            square.landed_state_from_extended_sys_state({"landed_state": 1}),
            "on_ground",
        )
        self.assertEqual(
            square.landed_state_from_extended_sys_state({"landed_state": 0}),
            "unknown",
        )
        self.assertIsNone(
            square.landed_state_from_extended_sys_state({"landed_state": 9})
        )
        self.assertEqual(
            square.prefer_landed_state("unknown", None, "on_ground"), "on_ground"
        )
        self.assertEqual(
            square.prefer_landed_state("unknown", None, "unknown"), "unknown"
        )
        self.assertEqual(square.prefer_landed_state("unknown", None, None), "unknown")
        self.assertEqual(
            square.prefer_landed_state("in_air", None, "on_ground"), "in_air"
        )
        self.assertEqual(
            square.prefer_landed_state("in_air", "on_ground", "in_air"), "on_ground"
        )
        self.assertEqual(
            square.prefer_landed_state("unknown", "unknown", "on_ground"), "on_ground"
        )

        robot, _action = self._robot()
        drone = robot.custom["drone"]
        assert isinstance(drone, dict)
        drone["landed_state"] = "unknown"
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        by_name = {name: callback for name, callback in direct.subscriptions}
        vehicle = square.OhhoVehicle(robot, object(), health=watch)
        self.assertEqual(vehicle.read().landed_state, "unknown")

        out = io.StringIO()
        with redirect_stdout(out):
            by_name["COMMAND_ACK"](
                _DirectMessage.from_mapping(
                    "COMMAND_ACK", {"command": 511, "result": 2}
                ),
                None,
            )
            by_name["HEARTBEAT"](
                _DirectMessage.from_mapping(
                    "HEARTBEAT", {"system_status": 1, "base_mode": 0}
                ),
                None,
            )
        self.assertEqual(vehicle.read().landed_state, "unknown")
        self.assertIn("DENIED", out.getvalue())
        self.assertIn("rejected DENIED", watch.interval_ack_note)
        detail = square._timeout_detail(
            vehicle, "no on_ground telemetry before arm", vehicle.read()
        )
        self.assertIn("last=unknown", detail)
        self.assertIn("SET_MESSAGE_INTERVAL rejected DENIED", detail)

        with redirect_stdout(io.StringIO()):
            by_name["HEARTBEAT"](
                _DirectMessage.from_mapping(
                    "HEARTBEAT", {"system_status": square.MAV_STATE_STANDBY}
                ),
                None,
            )
        self.assertEqual(vehicle.read().landed_state, "on_ground")

        by_name["HEARTBEAT"](
            _DirectMessage.from_mapping(
                "HEARTBEAT", {"system_status": square.MAV_STATE_ACTIVE}
            ),
            None,
        )
        self.assertEqual(vehicle.read().landed_state, "in_air")

        with redirect_stdout(io.StringIO()):
            by_name["EXTENDED_SYS_STATE"](
                _DirectMessage.from_mapping(
                    "EXTENDED_SYS_STATE",
                    {"landed_state": square.MAV_LANDED_STATE_ON_GROUND},
                ),
                None,
            )
        self.assertEqual(vehicle.read().landed_state, "on_ground")

        by_name["EXTENDED_SYS_STATE"](
            _DirectMessage.from_mapping(
                "EXTENDED_SYS_STATE",
                {"landed_state": square.MAV_LANDED_STATE_UNDEFINED},
            ),
            None,
        )
        self.assertEqual(vehicle.read().landed_state, "in_air")

        drone["landed_state"] = "takeoff"
        with redirect_stdout(io.StringIO()):
            by_name["HEARTBEAT"](
                _DirectMessage.from_mapping(
                    "HEARTBEAT", {"system_status": square.MAV_STATE_STANDBY}
                ),
                None,
            )
        self.assertEqual(vehicle.read().landed_state, "takeoff")

        with redirect_stdout(io.StringIO()):
            by_name["EXTENDED_SYS_STATE"](
                _DirectMessage.from_mapping(
                    "EXTENDED_SYS_STATE",
                    {"landed_state": square.MAV_LANDED_STATE_LANDING},
                ),
                None,
            )
        self.assertEqual(vehicle.read().landed_state, "landing")

        with redirect_stdout(io.StringIO()):
            by_name["COMMAND_ACK"](
                _DirectMessage.from_mapping(
                    "COMMAND_ACK",
                    {"command": square.MAV_CMD_SET_MESSAGE_INTERVAL, "result": 0},
                ),
                None,
            )
        accepted_note = watch.interval_ack_note
        self.assertIn("accepted x1", accepted_note)
        self.assertIn("rejected DENIED x1", accepted_note)
        by_name["COMMAND_ACK"](
            _DirectMessage.from_mapping("COMMAND_ACK", {"command": 400, "result": 0}),
            None,
        )
        self.assertEqual(watch.interval_ack_note, accepted_note)
        vehicle.close()
        self.assertTrue(direct.destroyed)

    def test_local_position_is_emitted_once_and_a_heartbeat_is_not(self) -> None:
        robot, _action = self._robot()
        direct = _Direct(object())
        watch = self._attach(robot, direct)
        inner = _FeedInner()
        robot.transport = type("Gate", (), {"_inner": inner})()
        vehicle = square.OhhoVehicle(robot, object(), health=watch)
        by_name = {name: callback for name, callback in direct.subscriptions}
        by_name["HEARTBEAT"](
            _DirectMessage(
                "HEARTBEAT",
                {"base_mode": 128, "system_status": square.MAV_STATE_STANDBY},
            ),
            None,
        )
        self.assertFalse(vehicle.feed_gate())
        self.assertFalse(vehicle.position_fed)
        self.assertEqual(inner.emits, 0)
        self.assertEqual(inner.ingested[0][0], "HEARTBEAT")
        by_name["LOCAL_POSITION_NED"](
            _DirectMessage(
                "LOCAL_POSITION_NED",
                {"x": 1.25, "y": -2.5, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
            ),
            None,
        )
        with redirect_stdout(io.StringIO()) as out:
            fed = vehicle.feed_gate()
        self.assertTrue(fed)
        self.assertTrue(vehicle.position_fed)
        self.assertEqual(inner.emits, 1)
        self.assertEqual(inner.ingested[-1][0], "LOCAL_POSITION_NED")
        self.assertEqual(inner.ingested[-1][1]["x"], 1.25)
        self.assertIn("LOCAL_POSITION_NED", out.getvalue())
        self.assertFalse(vehicle.feed_gate())
        self.assertFalse(vehicle.position_fed)
        self.assertEqual(inner.emits, 1)
        vehicle.close()
        self.assertTrue(direct.destroyed)


class _FeedInner:
    def __init__(self) -> None:
        self.ingested: list[tuple[str, dict[str, object]]] = []
        self.emits = 0
        self._pending = False

    def ingest_mavlink(self, name: object, fields: object) -> bool:
        if not isinstance(fields, dict):
            return False
        self.ingested.append((str(name), dict(fields)))
        self._pending = True
        return True

    def emit_ingested_telemetry(self) -> bool:
        if not self._pending:
            return False
        self._pending = False
        self.emits += 1
        return True


class _Gate:
    def __init__(self, error: str | None = None) -> None:
        self.last_error = error
        self.latched = False
        self.latch_reason = None
        self.profile = type("Profile", (), {"aerial": True})()


class _GateRobot:
    def __init__(self, error: str | None = None, *, arm_ok: bool = True) -> None:
        self.transport = _Gate(error)
        self.arm_ok = arm_ok
        self.refusal = error
        self.calls: list[tuple[object, ...]] = []

    def telemetry(self) -> object:
        return type("Tele", (), {})()

    def arm(self) -> bool:
        self.calls.append(("arm",))
        if not self.arm_ok:
            self.transport.last_error = self.refusal
            return False
        return True

    def arm_flight(self) -> None:
        self.calls.append(("arm_flight",))

    def offboard_position(
        self, x: float, y: float, z: float, yaw: float, frame: object
    ) -> None:
        self.calls.append(("offboard_position", x, y, z, yaw, frame))

    def takeoff(self, alt_m: float) -> None:
        self.calls.append(("takeoff", alt_m))

    def land(self) -> None:
        self.calls.append(("land",))

    def disarm_flight(self) -> None:
        self.calls.append(("disarm_flight",))

    def disconnect(self) -> None:
        self.calls.append(("disconnect",))


class GateFlightTests(unittest.TestCase):
    def test_vehicle_calls_gate_methods_with_a_setpoint_before_arm_flight(self) -> None:
        robot = _GateRobot()
        frame = object()
        vehicle = square.OhhoVehicle(
            robot, frame, health=square.CopterReadinessWatch()
        )
        vehicle.arm_gate()
        vehicle.stream_setpoint(1.0, 2.0, 0.0, 0.0)
        vehicle.arm()
        vehicle.takeoff(2.5)
        vehicle.goto(4.0, 0.0, -2.5, 0.0)
        vehicle.land()
        vehicle.disarm()
        names = [call[0] for call in robot.calls]
        self.assertEqual(
            names,
            [
                "arm",
                "offboard_position",
                "arm_flight",
                "takeoff",
                "offboard_position",
                "land",
                "disarm_flight",
            ],
        )
        self.assertLess(names.index("offboard_position"), names.index("arm_flight"))
        self.assertIs(robot.calls[1][5], frame)
        self.assertEqual(robot.calls[3], ("takeoff", 2.5))

    def test_gate_refusal_becomes_the_failure_reason(self) -> None:
        robot = _GateRobot()

        def arm_flight() -> None:
            robot.calls.append(("arm_flight",))
            robot.transport.last_error = "no_setpoint_stream"

        robot.arm_flight = arm_flight  # type: ignore[method-assign]
        vehicle = square.OhhoVehicle(
            robot, object(), health=square.CopterReadinessWatch()
        )
        with self.assertRaises(square.SquareFailed) as caught:
            vehicle.arm()
        self.assertIn("no_setpoint_stream", str(caught.exception))
        self.assertIn("refused by SafetyGate", str(caught.exception))
        payload = square._failure_payload(caught.exception, 1.25)
        self.assertFalse(payload["passed"])
        self.assertIn("no_setpoint_stream", str(payload["message"]))

        flyer = FakeVehicle()

        def refuse_arm() -> None:
            flyer.calls.append("arm")
            raise square.SquareFailed(
                "arm_flight refused by SafetyGate: no_setpoint_stream"
            )

        flyer.arm = refuse_arm  # type: ignore[method-assign]
        clock = FakeClock()
        with self.assertRaises(square.SquareFailed) as flown:
            square.fly_square(flyer, sleep=clock.sleep, monotonic=clock)
        self.assertIn("no_setpoint_stream", str(flown.exception))
        flown_payload = square._failure_payload(flown.exception, clock.t)
        self.assertIn("no_setpoint_stream", str(flown_payload["message"]))
        self.assertIn("land", flyer.calls)
        self.assertNotIn("disarm", flyer.calls)

    def test_gate_arm_refusal_names_last_error(self) -> None:
        robot = _GateRobot(error="ci", arm_ok=False)
        vehicle = square.OhhoVehicle(
            robot, object(), health=square.CopterReadinessWatch()
        )
        with self.assertRaises(square.SquareFailed) as caught:
            vehicle.arm_gate()
        self.assertIn("SafetyGate.arm() refused the gate", str(caught.exception))
        self.assertIn("ci", str(caught.exception))


class WallClockTests(unittest.TestCase):
    def test_budget_is_the_phase_sum_plus_a_margin(self) -> None:
        self.assertEqual(square.GROUND_TIMEOUT_S, 90.0)
        self.assertEqual(square.ARM_TIMEOUT_S, 90.0)
        self.assertEqual(square.CLIMB_TIMEOUT_S, 90.0)
        self.assertEqual(square.CORNER_TIMEOUT_S, 45.0)
        self.assertEqual(square.LAND_TIMEOUT_S, 60.0)
        self.assertEqual(square.DISCOVERY_TIMEOUT_S, 120.0)
        self.assertEqual(square.ARM_SETTLE_S, 0.5)
        self.assertGreaterEqual(square.ARM_CONFIRM_S, 2.0)
        self.assertGreater(square.WALL_MARGIN_S, 0.0)
        self.assertLess(square.WALL_MARGIN_S, 60.0)
        self.assertGreater(square.WALL_LAND_ATTEMPT_S, 0.0)
        self.assertLess(square.WALL_LAND_ATTEMPT_S, square.LAND_TIMEOUT_S)
        corners = square.SQUARE_CORNERS
        phases = (
            square.GROUND_TIMEOUT_S
            + square.ARM_TIMEOUT_S
            + square.ARM_ATTEMPTS * square.ARM_CONFIRM_S
            + (square.ARM_ATTEMPTS - 1) * square.ARM_RETRY_PAUSE_S
            + square.CLIMB_TIMEOUT_S
            + corners * square.CORNER_TIMEOUT_S
            + square.LAND_TIMEOUT_S
            + corners * (square.SIDE_M / square.SPEED_MPS)
        )
        self.assertAlmostEqual(square.flight_wall_s(), phases + square.WALL_MARGIN_S)
        self.assertAlmostEqual(
            square.script_wall_s(),
            square.DISCOVERY_TIMEOUT_S + square.flight_wall_s(),
        )

    def test_expiry_attempts_land_and_writes_a_failure(self) -> None:
        vehicle = FakeVehicle()
        box: list[object] = [vehicle]
        exited: dict[str, int] = {}

        def exit_fn(code: int) -> None:
            exited["code"] = code

        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "square-result.json")
            wall = square._FlightWall(
                0.05,
                box,
                path,
                time.perf_counter(),
                exit_fn,
                land_attempt_s=0.2,
            )
            wall.start()
            wall._timer.join(2.0)
            self.assertFalse(wall._timer.is_alive())
            self.assertEqual(exited.get("code"), 1)
            self.assertIn("land", vehicle.calls)
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            self.assertFalse(payload["passed"])
            self.assertIn("wall clock expired", payload["message"])
            self.assertIn("0.05", payload["message"])

    def test_expiry_still_writes_when_land_blocks(self) -> None:
        vehicle = FakeVehicle()
        started = threading.Event()

        def land() -> None:
            vehicle.calls.append("land")
            started.set()
            threading.Event().wait()

        vehicle.land = land  # type: ignore[method-assign]
        box: list[object] = [vehicle]
        exited: dict[str, int] = {}
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "square-result.json")
            wall = square._FlightWall(
                0.05,
                box,
                path,
                time.perf_counter(),
                lambda code: exited.__setitem__("code", code),
                land_attempt_s=0.15,
            )
            wall.start()
            wall._timer.join(2.0)
            self.assertFalse(wall._timer.is_alive())
            self.assertTrue(started.is_set())
            self.assertEqual(exited.get("code"), 1)
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            self.assertIn("wall clock expired", payload["message"])
            self.assertFalse(payload["passed"])


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
        self.assertIn("SafetyGate", result.stdout)
        self.assertIn("OHHO_CI_SIM_ARMING=1", result.stdout)
        self.assertIn("OHHO_CI_SITL=1", result.stdout)
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

    def test_open_sitl_refuses_without_opt_in_under_github_actions(self) -> None:
        import os
        from unittest import mock

        env = os_environ_without_arm()
        env["GITHUB_ACTIONS"] = "true"
        env.pop("OHHO_CI_SIM_ARMING", None)
        env.pop("OHHO_CI_SITL", None)
        with mock.patch.dict(os.environ, env, clear=True):
            with self.assertRaises(square.Refused) as caught:
                square.open_sitl()
        message = str(caught.exception)
        self.assertIn("OHHO_CI_SIM_ARMING", message)
        self.assertIn("OHHO_CI_SITL", message)

    def test_open_sitl_refuses_a_partial_or_non_exact_opt_in(self) -> None:
        import os
        from unittest import mock

        cases = (
            {"OHHO_CI_SIM_ARMING": "1"},
            {"OHHO_CI_SITL": "1"},
            {"OHHO_CI_SIM_ARMING": "true", "OHHO_CI_SITL": "1"},
            {"OHHO_CI_SIM_ARMING": "1", "OHHO_CI_SITL": "true"},
        )
        for extra in cases:
            env = os_environ_without_arm()
            env["GITHUB_ACTIONS"] = "true"
            env.pop("OHHO_CI_SIM_ARMING", None)
            env.pop("OHHO_CI_SITL", None)
            env.update(extra)
            with mock.patch.dict(os.environ, env, clear=True):
                with self.assertRaises(square.Refused):
                    square.open_sitl()

    def test_open_sitl_refuses_when_hardware_arming_is_present(self) -> None:
        import os
        from unittest import mock

        env = os_environ_without_arm()
        env["OHHO_ARM_HARDWARE"] = ""
        env["GITHUB_ACTIONS"] = "true"
        env["OHHO_CI_SIM_ARMING"] = "1"
        env["OHHO_CI_SITL"] = "1"
        with mock.patch.dict(os.environ, env, clear=True):
            with self.assertRaises(square.Refused) as caught:
                square.open_sitl()
        self.assertIn("OHHO_ARM_HARDWARE", str(caught.exception))

    def test_opt_in_is_not_required_outside_github_actions(self) -> None:
        square._require_ci_sim_opt_in({})
        square._require_ci_sim_opt_in({"OHHO_CI_SIM_ARMING": "1"})


def os_environ_without_arm() -> dict[str, str]:
    import os

    env = dict(os.environ)
    env.pop("OHHO_ARM_HARDWARE", None)
    return env


if __name__ == "__main__":
    unittest.main()
