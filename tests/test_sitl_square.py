"""Mission rules for the PX4 SITL square. No ROS, no Docker, no Ohho install."""

from __future__ import annotations

import math
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import sitl_square  # noqa: E402


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeVehicle:
    """Kinematic stand-in. Sleep in the test is what calls ``step``."""

    def __init__(self) -> None:
        self.calls: list[str | tuple[str, float] | tuple[str, float, float, float, float]] = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.armed = False
        self.landed = "on_ground"
        self.battery = 1.0
        self.link_ok = True
        self.prearm: bool | None = True
        self.failsafe: tuple[str, ...] = ()
        self.gate_armed = False
        self.target: tuple[float, float, float] | None = None
        self.takeoff_cmd = False
        self.landing = False
        self.follow = True
        self.disarm_landed: list[str] = []

    def snapshot(self) -> sitl_square.DroneSample:
        return sitl_square.DroneSample(
            x=self.x,
            y=self.y,
            z=self.z,
            pose_frame="LOCAL_NED",
            armed=self.armed,
            landed_state=self.landed,
            failsafe=self.failsafe,
            link_ok=self.link_ok,
            battery=self.battery,
            prearm=self.prearm,
        )

    def step(self) -> None:
        if self.landing:
            self.z = 0.0
            self.landed = "on_ground"
            self.armed = False
            self.landing = False
            return
        if self.takeoff_cmd and self.landed != "in_air":
            self.z = -sitl_square.TAKEOFF_ALT_M
            self.landed = "in_air"
            return
        if (
            self.follow
            and self.landed == "in_air"
            and self.target is not None
            and self.target[2] < -1.0
        ):
            self.x, self.y, self.z = self.target

    def arm_gate(self) -> bool:
        self.calls.append("arm_gate")
        self.gate_armed = True
        return True

    def arm_flight(self) -> None:
        self.calls.append("arm_flight")
        if not self.gate_armed:
            raise sitl_square.MissionError("gate is disarmed")
        self.armed = True

    def takeoff(self, alt_m: float) -> None:
        self.calls.append(("takeoff", alt_m))
        self.takeoff_cmd = True
        self.landed = "takeoff"

    def offboard_position(self, x: float, y: float, z: float, yaw: float) -> None:
        self.calls.append(("offboard", x, y, z, yaw))
        self.target = (x, y, z)

    def land(self) -> None:
        self.calls.append("land")
        self.landing = True

    def disarm_flight(self) -> None:
        self.disarm_landed.append(self.landed)
        self.calls.append("disarm")
        if self.landed != "on_ground":
            raise sitl_square.MissionError("not_on_ground")
        self.armed = False

    def latched(self) -> bool:
        return False

    def latch_reason(self) -> None:
        return None

    def last_error(self) -> None:
        return None


def _names(vehicle: FakeVehicle) -> list[str]:
    names: list[str] = []
    for call in vehicle.calls:
        names.append(call if isinstance(call, str) else call[0])
    return names


def _run(vehicle: FakeVehicle) -> sitl_square.MissionResult:
    clock = Clock()

    def sleep(dt: float) -> None:
        clock.advance(dt)
        vehicle.step()

    return sitl_square.run_mission(
        vehicle, clock=clock, sleep=sleep, log=lambda _line: None
    )


class GeometryTest(unittest.TestCase):
    def test_square_is_four_metre_sides_in_ned(self) -> None:
        corners = sitl_square.square_corners()
        self.assertEqual(len(corners), 5)
        self.assertEqual(corners[0], corners[-1])
        for corner in corners:
            self.assertAlmostEqual(corner[2], -2.5)
        for start, end in zip(corners, corners[1:]):
            self.assertAlmostEqual(math.hypot(end[0] - start[0], end[1] - start[1]), 4.0)

    def test_cruise_speed_is_half_a_metre_per_second(self) -> None:
        points = sitl_square.cruise_points()
        period = 1.0 / sitl_square.STREAM_HZ
        length = 0.0
        for start, end in zip(points, points[1:]):
            length += math.hypot(end[0] - start[0], end[1] - start[1])
        duration = (len(points) - 1) * period
        self.assertAlmostEqual(length, 16.0, places=6)
        self.assertAlmostEqual(length / duration, 0.5, places=6)
        for corner in sitl_square.scored_corners():
            self.assertTrue(
                any(
                    all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(point, corner))
                    for point in points
                )
            )

    def test_corner_box_uses_the_published_tolerances(self) -> None:
        corner = (4.0, 0.0, -2.5)
        inside = sitl_square.DroneSample(
            4.5, 0.0, -2.5, "LOCAL_NED", True, "in_air", (), True, 1.0, True
        )
        outside = sitl_square.DroneSample(
            5.0, 0.0, -2.5, "LOCAL_NED", True, "in_air", (), True, 1.0, True
        )
        high = sitl_square.DroneSample(
            4.0, 0.0, -3.0, "LOCAL_NED", True, "in_air", (), True, 1.0, True
        )
        self.assertTrue(sitl_square.corner_accepted(inside, corner))
        self.assertFalse(sitl_square.corner_accepted(outside, corner))
        self.assertFalse(sitl_square.corner_accepted(high, corner))
        on_edge = sitl_square.horizontal_error_m(4.0 + 0.8, 0.0, corner)
        self.assertIsNotNone(on_edge)
        assert on_edge is not None
        self.assertLessEqual(on_edge, 0.8 + 1e-9)

    def test_altitude_hold_needs_two_continuous_seconds(self) -> None:
        ok, since = sitl_square.altitude_held(-2.5, None, 10.0)
        self.assertFalse(ok)
        self.assertEqual(since, 10.0)
        ok, since = sitl_square.altitude_held(-2.5, since, 11.9)
        self.assertFalse(ok)
        ok, _since = sitl_square.altitude_held(-2.5, since, 12.0)
        self.assertTrue(ok)
        ok, since = sitl_square.altitude_held(-2.0, since, 12.1)
        self.assertFalse(ok)
        self.assertIsNone(since)

    def test_positive_up_is_ned_only(self) -> None:
        sample = sitl_square.DroneSample(
            0.0, 0.0, -2.5, "LOCAL_NED", True, "in_air", (), True, 1.0, True
        )
        self.assertAlmostEqual(sitl_square.positive_up_m(sample) or 0.0, 2.5)
        other = sitl_square.DroneSample(
            0.0, 0.0, 2.5, "LOCAL_ENU", True, "in_air", (), True, 1.0, True
        )
        self.assertIsNone(sitl_square.positive_up_m(other))


class UriAndSafetyTest(unittest.TestCase):
    def test_loopback_uris_are_accepted(self) -> None:
        for uri in (
            "px4+dds://127.0.0.1:8888",
            "px4+dds://127.0.0.2",
            "px4+dds://localhost:8888",
            "px4+dds://[::1]:8888",
        ):
            sitl_square.check_sitl_uri(uri)

    def test_hardware_uris_are_refused(self) -> None:
        for uri in (
            "px4+dds://10.0.0.8:8888",
            "px4+dds://192.168.1.10:8888",
            "serial://COM3",
            "px4+dds://user:secret@127.0.0.1:8888",
        ):
            with self.assertRaises(sitl_square.MissionError):
                sitl_square.check_sitl_uri(uri)

    def test_hardware_arm_env_is_refused(self) -> None:
        with self.assertRaises(sitl_square.MissionError):
            sitl_square.refuse_hardware_arm_env({"OHHO_ARM_HARDWARE": "1"})
        sitl_square.refuse_hardware_arm_env({})

    def test_hardware_classification_blocks_before_any_phrase(self) -> None:
        self.assertIsNone(sitl_square.hardware_arming_block(False))
        message = sitl_square.hardware_arming_block(True)
        self.assertIsNotNone(message)
        assert message is not None
        self.assertIn("OHHO_ARM_HARDWARE", message)

    def test_script_does_not_set_the_hardware_switch_or_force_disarm(self) -> None:
        source = Path(sitl_square.__file__).read_text(encoding="utf-8")
        self.assertNotIn('os.environ["OHHO_ARM_HARDWARE"] =', source)
        self.assertNotIn("21196", source)
        self.assertNotIn("--arm", source)

    def test_main_refuses_a_hardware_uri_without_importing_ohho(self) -> None:
        code = sitl_square.main(["--uri", "px4+dds://10.1.2.3:8888"])
        self.assertEqual(code, 1)

    def test_main_refuses_a_preset_hardware_arm_variable(self) -> None:
        with mock.patch.dict(os.environ, {"OHHO_ARM_HARDWARE": "1"}):
            code = sitl_square.main(["--uri", sitl_square.DEFAULT_URI])
        self.assertEqual(code, 1)


class MissionTest(unittest.TestCase):
    def test_square_passes_when_the_vehicle_tracks(self) -> None:
        vehicle = FakeVehicle()
        result = _run(vehicle)
        self.assertTrue(result.passed, result.reasons)
        self.assertTrue(result.takeoff_held)
        self.assertEqual(result.landed_state, "on_ground")
        self.assertIs(result.armed, False)
        self.assertEqual(len(result.corner_errors_m), 4)
        for error in result.corner_errors_m:
            self.assertIsNotNone(error)
            assert error is not None
            self.assertLessEqual(error, 0.8)
        names = _names(vehicle)
        arm_at = names.index("arm_flight")
        self.assertEqual(names[0], "arm_gate")
        self.assertIn("offboard", names[:arm_at])
        self.assertLess(arm_at, names.index("takeoff"))
        self.assertIn("land", names)
        self.assertNotIn("disarm", names)
        takeoffs = [call for call in vehicle.calls if isinstance(call, tuple) and call[0] == "takeoff"]
        self.assertEqual(takeoffs, [("takeoff", 2.5)])
        self.assertIn("square", result.phases)
        self.assertEqual(result.to_json()["criteria"]["square_side_m"], 4.0)

    def test_gate_arm_refusal_does_not_take_off(self) -> None:
        vehicle = FakeVehicle()
        vehicle.arm_gate = lambda: False  # type: ignore[method-assign]
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertTrue(any("SafetyGate.arm()" in reason for reason in result.reasons))
        self.assertNotIn("takeoff", _names(vehicle))
        self.assertNotIn("arm_flight", _names(vehicle))

    def test_missed_corner_fails_and_still_lands(self) -> None:
        vehicle = FakeVehicle()
        vehicle.follow = False
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertTrue(any("corner 1" in reason for reason in result.reasons))
        self.assertIn("land", _names(vehicle))

    def test_failsafe_after_takeoff_fails_the_run(self) -> None:
        vehicle = FakeVehicle()
        vehicle.failsafe = ("rc_loss",)
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertEqual(result.failsafes, ["rc_loss"])
        self.assertTrue(any("failsafe" in reason for reason in result.reasons))
        self.assertIn("land", _names(vehicle))

    def test_disarm_is_not_sent_while_airborne(self) -> None:
        vehicle = FakeVehicle()

        def land() -> None:
            vehicle.calls.append("land")

        vehicle.land = land  # type: ignore[method-assign]
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertNotIn("disarm", _names(vehicle))
        self.assertEqual(vehicle.disarm_landed, [])
        self.assertNotEqual(result.landed_state, "on_ground")


class WorkflowFileTest(unittest.TestCase):
    def test_workflow_pins_v1_17_and_keeps_the_hardware_switch_off(self) -> None:
        text = (ROOT / ".github" / "workflows" / "sitl-px4.yml").read_text(encoding="utf-8")
        script = (ROOT / "scripts" / "run_sitl_px4.sh").read_text(encoding="utf-8")
        for required in (
            "workflow_dispatch:",
            "pull_request:",
            "paths:",
            "timeout-minutes: 40",
            "px4io/px4-dev",
            "v1.17.0",
            "sha256:288dc1d7f5290855e86dbaf8b8f55cd17f46f27e9df2affb2056df6363bc6fb0",
            "HEADLESS=1",
            "PX4_SIM_MODEL=gz_x500",
            "PX4_SIM_SPEED_FACTOR=1",
            "actions/upload-artifact@v4",
            "if: always()",
            "px4-console-tail.txt",
            "GITHUB_ACTIONS",
        ):
            self.assertIn(required, text, required)
        self.assertNotIn("OHHO_ARM_HARDWARE=", text)
        self.assertNotIn("-e OHHO_ARM_HARDWARE", text)
        self.assertIn("OHHO_ARM_HARDWARE:-", text)
        self.assertNotIn(":latest", text)
        self.assertNotIn("--arm", text)
        self.assertNotIn("--privileged", text)
        self.assertNotIn("--network host", text)
        self.assertNotIn("sitl-ardupilot", text)
        for required in (
            "v1.17.0",
            "release/1.17",
            "v2.4.3",
            "ohh-290-px4-dds",
            "HEADLESS=1",
            "PX4_SIM_MODEL=gz_x500",
            "PX4_SIM_SPEED_FACTOR=1",
            "MicroXRCEAgent udp4 -p 8888",
            "px4+dds://127.0.0.1:8888",
            "echo $$",
            "px4.pid",
        ):
            self.assertIn(required, script, required)
        self.assertNotIn("PX4_PID=$!", script)
        self.assertNotIn("OHHO_ARM_HARDWARE=1", script)
        self.assertNotIn("--privileged", script)
        self.assertNotIn("--network host", script)
        self.assertNotIn("21196", script)


if __name__ == "__main__":
    unittest.main()
