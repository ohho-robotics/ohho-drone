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

import sitl_gcs_heartbeat  # noqa: E402
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
        for value in ("1", "0", "", "true", "yes"):
            with self.assertRaises(sitl_square.MissionError):
                sitl_square.refuse_hardware_arm_env({"OHHO_ARM_HARDWARE": value})
        sitl_square.refuse_hardware_arm_env({})
        sitl_square.refuse_hardware_arm_env(
            {"OHHO_CI_SIM_ARMING": "1", "OHHO_CI_SITL": "1"}
        )

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
        for value in ("1", "", "0"):
            with mock.patch.dict(os.environ, {"OHHO_ARM_HARDWARE": value}):
                with mock.patch.object(sitl_square, "connect_and_run") as connect:
                    code = sitl_square.main(["--uri", sitl_square.DEFAULT_URI])
            self.assertEqual(code, 1)
            connect.assert_not_called()

    def test_github_actions_refuses_to_connect_without_both_opt_ins(self) -> None:
        incomplete = (
            {"GITHUB_ACTIONS": "true"},
            {"GITHUB_ACTIONS": "true", "OHHO_CI_SIM_ARMING": "1"},
            {"GITHUB_ACTIONS": "true", "OHHO_CI_SITL": "1"},
            {"GITHUB_ACTIONS": " True ", "OHHO_CI_SIM_ARMING": "1", "OHHO_CI_SITL": "true"},
            {"GITHUB_ACTIONS": "true", "OHHO_CI_SIM_ARMING": "1", "OHHO_CI_SITL": ""},
        )
        for env in incomplete:
            with self.assertRaises(sitl_square.MissionError) as caught:
                sitl_square.require_ci_sim_opt_in(env)
            message = str(caught.exception)
            self.assertIn("OHHO_CI_SIM_ARMING=1", message)
            self.assertIn("OHHO_CI_SITL=1", message)
        sitl_square.require_ci_sim_opt_in({})
        sitl_square.require_ci_sim_opt_in(
            {"CI": "true", "OHHO_CI_SIM_ARMING": "1", "OHHO_CI_SITL": "1"}
        )
        sitl_square.require_ci_sim_opt_in(
            {
                "GITHUB_ACTIONS": "true",
                "OHHO_CI_SIM_ARMING": "1",
                "OHHO_CI_SITL": "1",
            }
        )

    def test_main_does_not_connect_on_actions_without_the_opt_in(self) -> None:
        with mock.patch.dict(os.environ, {"GITHUB_ACTIONS": "true", "CI": "true"}, clear=False):
            os.environ.pop("OHHO_CI_SIM_ARMING", None)
            os.environ.pop("OHHO_CI_SITL", None)
            os.environ.pop("OHHO_ARM_HARDWARE", None)
            with mock.patch.object(sitl_square, "connect_and_run") as connect:
                code = sitl_square.main(["--uri", sitl_square.DEFAULT_URI])
        self.assertEqual(code, 1)
        connect.assert_not_called()

    def test_main_reaches_connect_when_actions_opted_in(self) -> None:
        env = {
            "GITHUB_ACTIONS": "true",
            "CI": "true",
            "OHHO_CI_SIM_ARMING": "1",
            "OHHO_CI_SITL": "1",
        }
        with mock.patch.dict(os.environ, env, clear=False):
            os.environ.pop("OHHO_ARM_HARDWARE", None)
            with mock.patch.object(
                sitl_square,
                "connect_and_run",
                side_effect=sitl_square.MissionError("stopped in test"),
            ) as connect:
                code = sitl_square.main(["--uri", sitl_square.DEFAULT_URI])
        self.assertEqual(code, 1)
        connect.assert_called_once()


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
        self.assertEqual(result.preflight_failures, [])
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

    def test_wait_status_timeout_names_current_preflight_failures(self) -> None:
        vehicle = FakeVehicle()
        vehicle.prearm = False

        def report() -> tuple[list[str], str]:
            return ["No connection to the GCS"], ""

        clock = Clock()

        def sleep(dt: float) -> None:
            clock.advance(dt)

        result = sitl_square.run_mission(
            vehicle,
            clock=clock,
            sleep=sleep,
            log=lambda _line: None,
            preflight_report=report,
        )
        self.assertFalse(result.passed)
        self.assertEqual(result.phases, ["wait-status"])
        self.assertEqual(result.preflight_failures, ["No connection to the GCS"])
        self.assertTrue(
            any(
                "PX4 pre-arm checks did not pass: No connection to the GCS" in reason
                for reason in result.reasons
            )
        )
        self.assertNotIn("arm_gate", result.phases)

    def test_missing_console_is_named_when_prearm_does_not_pass(self) -> None:
        vehicle = FakeVehicle()
        vehicle.prearm = False
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertEqual(result.preflight_failures, [])
        self.assertTrue(
            any("PX4 console log was not available" in reason for reason in result.reasons)
        )

    def test_arm_flight_ci_gate_is_named_and_does_not_take_off(self) -> None:
        vehicle = FakeVehicle()

        def arm_flight() -> None:
            vehicle.calls.append("arm_flight")
            raise sitl_square.MissionError(sitl_square.flight_refusal("arm_flight", "ci"))

        vehicle.arm_flight = arm_flight  # type: ignore[method-assign]
        result = _run(vehicle)
        self.assertFalse(result.passed)
        self.assertTrue(any("_human_tty_reason" in reason for reason in result.reasons))
        self.assertTrue(any("_aerial_privileged" in reason for reason in result.reasons))
        self.assertIn("arm-flight", result.phases)
        self.assertNotIn("takeoff", result.phases)
        self.assertNotIn("takeoff", _names(vehicle))


class PreflightReportTest(unittest.TestCase):
    def test_tail_keeps_checks_that_are_still_printing(self) -> None:
        early = "WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data\n"
        current = "WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS\n"
        text = early + (current * sitl_square.PREFLIGHT_TAIL_LINES)
        self.assertEqual(
            sitl_square.preflight_failures(text),
            ["No connection to the GCS"],
        )

    def test_several_current_checks_stay_in_order(self) -> None:
        gcs = "WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS\t\n"
        power = "WARN  [health_and_arming_checks] Preflight Fail: system power unavailable\n"
        self.assertEqual(
            sitl_square.preflight_failures((gcs + power) * 40),
            ["No connection to the GCS", "system power unavailable"],
        )

    def test_console_file_uses_the_same_tail(self) -> None:
        import tempfile

        early = "WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data\n"
        current = "WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "px4-console.log"
            path.write_text(early + current * 120, encoding="utf-8")
            failures, note = sitl_square.read_preflight_failures(str(path))
        self.assertEqual(failures, ["No connection to the GCS"])
        self.assertEqual(note, "")
        missing, missing_note = sitl_square.read_preflight_failures(
            str(Path(tmp) / "absent.log")
        )
        self.assertEqual(missing, [])
        self.assertIn("not available", missing_note)

    def test_tty_gate_text_is_only_for_the_human_reasons(self) -> None:
        ci = sitl_square.flight_refusal("arm_flight", "ci")
        self.assertIn("_human_tty_reason", ci)
        self.assertIn("_aerial_privileged", ci)
        self.assertIn("(ci)", ci)
        self.assertIn("OHHO_CI_SIM_ARMING=1", ci)
        self.assertIn("OHHO_CI_SITL=1", ci)
        self.assertNotIn("OHHO_ARM_HARDWARE=1", ci)
        other = sitl_square.flight_refusal("takeoff", "battery")
        self.assertEqual(other, "takeoff refused by SafetyGate: battery")

    def test_ohho_vehicle_maps_the_ci_gate_without_taking_off(self) -> None:
        class Gate:
            def __init__(self) -> None:
                self.latched = False
                self.latch_reason = None
                self.last_error: str | None = None

        class Robot:
            def __init__(self) -> None:
                self.transport = Gate()

            def arm_flight(self) -> None:
                self.transport.last_error = "ci"

        vehicle = sitl_square.OhhoVehicle(Robot())
        with self.assertRaises(sitl_square.MissionError) as caught:
            vehicle.arm_flight()
        self.assertIn("_human_tty_reason", str(caught.exception))
        self.assertIn("(ci)", str(caught.exception))


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
            "scripts/sitl_gcs_heartbeat.py",
            "requirements-sitl.txt",
            "python -m pip install -r requirements-sitl.txt",
            "tail -c 262144 artifacts/px4-console.log | tail -n 200 > artifacts/px4-console-tail.txt",
            "GITHUB_ACTIONS",
            'sudo chown -R "$(id -u):$(id -g)" artifacts',
        ):
            self.assertIn(required, text, required)
        self.assertLess(
            text.index('sudo chown -R "$(id -u):$(id -g)" artifacts'),
            text.index("artifacts/px4-console-tail.txt"),
        )
        self.assertNotIn(
            "tail -n 200 artifacts/px4-console.log > artifacts/px4-console-tail.txt",
            text,
        )
        self.assertNotIn("OHHO_ARM_HARDWARE=", text)
        self.assertNotIn("-e OHHO_ARM_HARDWARE", text)
        self.assertIn("${OHHO_ARM_HARDWARE+x}", text)
        self.assertNotIn("${OHHO_ARM_HARDWARE:-}", text)
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
            "libopencv-dev",
            "libgstreamer-plugins-base1.0-dev",
            "cppzmq-dev",
            "libeigen3-dev",
            "libunwind-dev",
            "protobuf-compiler",
            "install_px4_sim_deps",
            "checkout_gz_models",
            "submodule update --init -- Tools/simulation/gz",
            "worlds/default.sdf",
            "build gz_x500:",
            "bc=1.07.1-3ubuntu4",
            "GZ_IP=127.0.0.1",
            'bin/px4" -d',
            "Startup script returned successfully",
            "tail -c 262144",
            "write_console_tail",
            "scripts/sitl_gcs_heartbeat.py",
            "requirements-sitl.txt",
            "127.0.0.1:14550 -> PX4 GCS port 18570",
            "--px4-console",
            'kill "$GCS_PID"',
        ):
            self.assertIn(required, script, required)
        self.assertNotIn("make px4_sitl gz_x500", script)
        self.assertNotIn("relay_px4_topics", script)
        self.assertNotIn("topic-relay", script)
        self.assertNotIn("NAV_DLL_ACT", script)
        self.assertNotIn("param set", script)
        self.assertNotIn("relay_px4_topics", text)
        self.assertNotIn("NAV_DLL_ACT", text)
        self.assertFalse((ROOT / "scripts" / "relay_px4_topics.py").exists())
        self.assertLess(
            script.index("scripts/sitl_gcs_heartbeat.py"),
            script.index('bin/px4" -d'),
        )
        self.assertLess(
            script.index("scripts/sitl_gcs_heartbeat.py"),
            script.index('scripts/sitl_square.py"'),
        )
        self.assertLess(
            script.index("\ninstall_px4_sim_deps\n"),
            script.index("\nbuild_px4\n"),
        )
        self.assertLess(
            script.index("\n  checkout_gz_models\n"),
            script.index('make -C "$PX4_SRC" px4_sitl_default'),
        )
        self.assertLess(
            script.index("libopencv-dev"),
            script.index('make -C "$PX4_SRC" px4_sitl_default'),
        )
        self.assertNotIn("PX4_PID=$!", script)
        self.assertNotIn("OHHO_ARM_HARDWARE=1", script)
        self.assertIn("${OHHO_ARM_HARDWARE+x}", script)
        self.assertNotIn("${OHHO_ARM_HARDWARE:-}", script)
        self.assertIn('OHHO_OS_REF="${OHHO_OS_REF:-ohh-290-px4-dds}"', script)
        self.assertIn("resolved ohho-os commit", script)
        self.assertIn("export OHHO_CI_SIM_ARMING", script)
        self.assertIn("export OHHO_CI_SITL", script)
        self.assertNotIn("OHHO_CI_SIM_ARMING=1", script)
        self.assertNotIn("OHHO_CI_SITL=1", script)
        self.assertNotIn("OHHO_CI_SIM_ARMING:-", script)
        self.assertNotIn("OHHO_CI_SITL:-", script)
        self.assertNotIn("--privileged", script)
        self.assertNotIn("--network host", script)
        self.assertNotIn("21196", script)

    def test_sim_opt_in_is_only_on_the_fly_step_and_passed_into_docker(self) -> None:
        text = (ROOT / ".github" / "workflows" / "sitl-px4.yml").read_text(encoding="utf-8")
        self.assertIn("Refuse hardware arming", text)
        _top, jobs = text.split("jobs:", 1)
        workflow_env = text.split("env:", 1)[1].split("jobs:", 1)[0]
        self.assertNotIn("OHHO_CI_SIM_ARMING", workflow_env)
        self.assertNotIn("OHHO_CI_SITL", workflow_env)
        criteria, square = jobs.split("  square:", 1)
        self.assertNotIn("OHHO_CI_SIM_ARMING", criteria)
        self.assertNotIn("OHHO_CI_SITL", criteria)
        before_fly, fly = square.split("- name: Fly the square", 1)
        self.assertNotIn("OHHO_CI_SIM_ARMING", before_fly)
        self.assertNotIn("OHHO_CI_SITL", before_fly)
        self.assertIn('${OHHO_ARM_HARDWARE+x}', before_fly)
        for required in (
            'OHHO_CI_SIM_ARMING: "1"',
            'OHHO_CI_SITL: "1"',
            '-e CI="${CI}"',
            '-e GITHUB_ACTIONS="${GITHUB_ACTIONS}"',
            '-e OHHO_CI_SIM_ARMING="${OHHO_CI_SIM_ARMING}"',
            '-e OHHO_CI_SITL="${OHHO_CI_SITL}"',
        ):
            self.assertIn(required, fly, required)
        self.assertLess(fly.index('-e CI="${CI}"'), fly.index('-e OHHO_CI_SIM_ARMING='))
        self.assertLess(
            fly.index('-e GITHUB_ACTIONS="${GITHUB_ACTIONS}"'),
            fly.index('-e OHHO_CI_SIM_ARMING='),
        )
        self.assertLess(
            fly.index('-e OHHO_CI_SIM_ARMING='),
            fly.index('-e OHHO_CI_SITL='),
        )

    def test_gcs_heartbeat_is_one_loopback_heartbeat_and_does_not_command(self) -> None:
        import socket

        from pymavlink.dialects.v20 import common as mavlink2

        pin = (ROOT / "requirements-sitl.txt").read_text(encoding="utf-8")
        import pymavlink

        self.assertIn(f"pymavlink=={pymavlink.__version__}", pin)
        self.assertEqual(sitl_gcs_heartbeat.PX4_GCS_PORT, 18570)
        self.assertEqual(sitl_gcs_heartbeat.GCS_BIND_PORT, 14550)
        self.assertEqual(sitl_gcs_heartbeat.check_pymavlink_pin(), pymavlink.__version__)
        sitl_gcs_heartbeat.require_instance_zero({})
        sitl_gcs_heartbeat.require_instance_zero({"PX4_INSTANCE": "0"})
        with self.assertRaises(sitl_gcs_heartbeat.GcsHeartbeatError):
            sitl_gcs_heartbeat.require_instance_zero({"PX4_INSTANCE": "1"})
        with mock.patch.dict(os.environ, {"PX4_INSTANCE": "1"}):
            self.assertEqual(sitl_gcs_heartbeat.main([]), 1)

        packet = sitl_gcs_heartbeat.gcs_heartbeat_bytes()
        self.assertEqual(packet[0], 0xFD)
        decoded = sitl_gcs_heartbeat.decode_packets(packet)
        self.assertEqual([message.get_type() for message in decoded], ["HEARTBEAT"])
        self.assertEqual(decoded[0].type, mavlink2.MAV_TYPE_GCS)
        self.assertEqual(decoded[0].autopilot, mavlink2.MAV_AUTOPILOT_INVALID)
        self.assertEqual(decoded[0].get_srcSystem(), sitl_gcs_heartbeat.GCS_SYSTEM_ID)

        px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            px4.bind(("127.0.0.1", 0))
            gcs.bind(("127.0.0.1", 0))
            px4_port = px4.getsockname()[1]
            gcs_port = gcs.getsockname()[1]
        finally:
            gcs.close()
        px4.settimeout(2)
        try:
            sent = sitl_gcs_heartbeat.serve(
                px4_port,
                gcs_port,
                max_heartbeats=1,
                log=lambda _line: None,
            )
            self.assertEqual(sent, 1)
            data, addr = px4.recvfrom(2048)
        finally:
            px4.close()
        self.assertEqual(addr[0], "127.0.0.1")
        self.assertEqual(addr[1], gcs_port)
        live = sitl_gcs_heartbeat.decode_packets(data)
        self.assertEqual([message.get_type() for message in live], ["HEARTBEAT"])
        self.assertEqual(live[0].type, mavlink2.MAV_TYPE_GCS)

        source = Path(sitl_gcs_heartbeat.__file__).read_text(encoding="utf-8")
        for banned in (
            "COMMAND_LONG",
            "ARM_DISARM",
            "param set",
            "param_set",
            "OHHO_ARM_HARDWARE",
            "--arm",
            "vehicle_command",
        ):
            self.assertNotIn(banned, source, banned)
        self.assertIn("MAV_TYPE_GCS", source)
        self.assertIn(sitl_gcs_heartbeat.LOOPBACK, source)


if __name__ == "__main__":
    unittest.main()
