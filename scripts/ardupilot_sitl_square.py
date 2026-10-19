"""D-07 square on ArduPilot Copter SITL through Ohho ``mavlink+udp://``.

OHH-306. The workflow starts ``sim_vehicle.py -v ArduCopter`` headless at the
pinned Copter-4.7.2 tag, then this script waits until ArduPilot's own pre-arm
bit, EKF flags, and a 3D GPS fix say the copter is armable. It arms the aerial
``SafetyGate``, streams an offboard setpoint, arms the autopilot, climbs to
2.5 m, flies a 4 m square, and lands. ``--print-recipe`` only prints those steps.

``Robot.connect`` wraps the loopback transport in that gate. Under GitHub
Actions the process connects only when ``OHHO_CI_SIM_ARMING`` and
``OHHO_CI_SITL`` are both exactly ``1``. That opt-in is the sim-only arming
exception. The gate's allowlist, geofence, takeoff cap, speed caps, battery
and link checks, setpoint-before-arm rule, e-stop latch, and watchdogs stay
enforced. The script leaves ``OHHO_ARM_HARDWARE`` unset and refuses ``--arm``.

Nothing in this module opens a socket unless ``--fly`` is passed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Mapping, Protocol, Sequence

COPTER_TAG = "Copter-4.7.2"
COPTER_COMMIT = "fb773567896014da007bca58358704d4a0d8516f"
SITL_URI = "mavlink+udp://127.0.0.1:14540"
MAVSDK_URL = "udpin://127.0.0.1:14540"
# Direct URL for branch ohh-305-mavlink-backend. That branch's project name
# is ohho-os and the extra in its pyproject is mavlink. A requirement named
# ohho does not match the built metadata, so pip discards the candidate.
# pip 25.1 also rejects a #egg= fragment that is not a plain project name.
OHHO_PIP_URL = (
    "ohho-os[mavlink] @ git+https://github.com/ohho-robotics/ohho-os"
    "@ohh-305-mavlink-backend"
)
# Copter-4.7.2 Tools/environment_install/install-prereqs-ubuntu.sh
# (fb773567896014da007bca58358704d4a0d8516f).
# Packages that ./waf copter and headless sim_vehicle.py import, using
# that script's names. empy==3.3.4 is the only pin. pexpect is only the
# apt package python3-pexpect there. The prereqs script creates
# ~/venv-ardupilot from whichever python3 is first on PATH. After
# actions/setup-python that is not Ubuntu's interpreter, so the venv
# cannot import the apt module. Tools/AP_Periph/wscript then aborts
# ./waf copter on `import pexpect`.
# sim_vehicle.py imports it through Tools/autotest/pysim/util.py.
# numpy, pyparsing, and psutil are what the script pip-installs when it
# creates that venv. intelhex is from the default extra list.
# `future` is not in this tag's prereqs and not in the pinned pymavlink
# install_requires. MAVProxy, geocoder, tabulate, flake8, junitparser,
# wsproto, and pygame are in the script but not on this import path.
# sim_vehicle.py is started with --no-mavproxy. Current MAVProxy also
# depends on vtk, matplotlib, and opencv-python, which this job does not
# import. install-prereqs-ubuntu.sh still installs those itself.
ARDUPILOT_VENV_PIP_PACKAGES = (
    "lxml",
    "pymavlink",
    "pyserial",
    "empy==3.3.4",
    "pexpect",
    "ptyprocess",
    "dronecan",
    "intelhex",
    "numpy",
    "pyparsing",
    "psutil",
)
ROBOT_ID = "px4-sitl"

# Aerial SafetyGate on ohh-305: takeoff cap 3 m, fence altitude 3 m, fence
# radius 10 m, horizontal speed cap 1.5 m/s. 2.5 m, 0.5 m/s, and a 4 m square
# from the origin (far corner about 5.7 m) fit those limits. Do not raise them.
TAKEOFF_ALT_M = 2.5
TAKEOFF_Z_NED = -TAKEOFF_ALT_M
ALT_TOL_M = 0.4
ALT_HOLD_S = 2.0
SIDE_M = 4.0
SPEED_MPS = 0.5
CORNER_TOL_M = 0.8
CORNER_HOLD_S = 1.0
STREAM_HZ = 10.0
DISARM_ALT_M = 0.25
DISCOVERY_TIMEOUT_S = 120.0

GROUND_TIMEOUT_S = 90.0
# Copter pre-arm (EKF / AHRS / home) usually passes 20-60 s after boot.
# This bounds the wait. It does not change the climb, corner, or land budgets.
ARM_TIMEOUT_S = 90.0
ARM_ATTEMPTS = 3
ARM_RETRY_PAUSE_S = 2.0
ARM_READY_LOG_S = 5.0
# Minimum post-arm stream. Not a phase timeout. Copter's HEARTBEAT is 1 Hz,
# so this alone can still see a stale disarmed bit.
ARM_SETTLE_S = 0.5
# Keep polling the HEARTBEAT armed bit at least this long before a retry.
ARM_CONFIRM_S = 3.0
# Four corners, each with its own CORNER_TIMEOUT_S. The carrot between
# corners is not inside that timer.
SQUARE_CORNERS = 4
# Added to the sum of the phase budgets. Does not replace any of them.
WALL_MARGIN_S = 30.0
# How long the wall-clock expiry waits for a land call that may itself be stuck.
WALL_LAND_ATTEMPT_S = 5.0
# Copter-4.7.2 leaves every SR0_* rate at 0 Hz. ``--no-mavproxy`` means nothing
# else asks for streams. MAVSDK heartbeats then republish a default health
# object, so ``is_armable`` stays false even after "EKF3 IMU0 is using GPS".
# These intervals are requested on the open MAVSDK link. They do not change
# arming checks. param2 is microseconds; 0 would restore the 0 Hz default.
STREAM_REQUEST_PERIOD_S = 2.0
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_SYS_STATUS_PREARM_CHECK = 268435456
AUTOPILOT_COMPONENT_ID = 1
# mavsdk 4.0.6 Configuration.create_with_component_type(GROUND_STATION).
MAVSDK_GCS_SYSTEM_ID = 245
MAVSDK_GCS_COMPONENT_ID = 190
GPS_FIX_3D = 3
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_VELOCITY_VERT = 4
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_CONST_POS_MODE = 128
EKF_PRED_POS_HORIZ_REL = 256
EKF_PRED_POS_HORIZ_ABS = 512
EKF_UNINITIALIZED = 1024
EKF_ACCEL_ERROR = 2048
EKF_GPS_GLITCHING = 32768
CLIMB_TIMEOUT_S = 90.0
CORNER_TIMEOUT_S = 45.0
LAND_TIMEOUT_S = 60.0

# Modes that mean the autopilot left the square on its own.
FAILSAFE_MODES = frozenset(
    {
        "rtl",
        "smart_rtl",
        "smartrtl",
        "brake",
        "land",
        "terminate",
        "kill",
        "return_to_launch",
        "auto.rtl",
        "auto.land",
    }
)

SIM_VEHICLE_ARGS = (
    "-v ArduCopter",
    "-w",
    "--speedup=1",
    "--no-mavproxy",
    "--no-rebuild",
    '-A "--serial0=udpclient:127.0.0.1:14540"',
)


class SquareFailed(Exception):
    """The square did not meet the D-07 criteria."""


class Refused(Exception):
    """The process was asked to arm hardware, or the URI is not loopback SITL."""


# Same bits ArduPilot's autotest requires before arming with an absolute
# position (Tools/autotest/vehicle_test_suite.py wait_ekf_happy). On
# EKF_STATUS_REPORT, bit 1024 is EKF_UNINITIALIZED and bit 15 is GPS glitching.
_EKF_REQUIRED = (
    (EKF_ATTITUDE, "EKF attitude"),
    (EKF_VELOCITY_HORIZ, "EKF horizontal velocity"),
    (EKF_VELOCITY_VERT, "EKF vertical velocity"),
    (EKF_POS_HORIZ_REL, "EKF relative horizontal position"),
    (EKF_POS_HORIZ_ABS, "EKF absolute horizontal position"),
    (EKF_POS_VERT_ABS, "EKF absolute vertical position"),
    (EKF_PRED_POS_HORIZ_REL, "EKF predicted relative position"),
    (EKF_PRED_POS_HORIZ_ABS, "EKF predicted absolute position"),
)
_EKF_ERRORS = (
    (EKF_CONST_POS_MODE, "EKF const position mode"),
    (EKF_UNINITIALIZED, "EKF uninitialized"),
    (EKF_ACCEL_ERROR, "EKF accel error"),
    (EKF_GPS_GLITCHING, "EKF GPS glitch"),
)
# (message name, MAVLink id, hertz). The first three are the arm gate.
# The rest are what Ohho already subscribes to; Copter does not send them
# at the firmware default rate.
STREAM_REQUESTS = (
    ("SYS_STATUS", 1, 2.0),
    ("GPS_RAW_INT", 24, 2.0),
    ("EKF_STATUS_REPORT", 193, 2.0),
    ("LOCAL_POSITION_NED", 32, 10.0),
    ("ATTITUDE", 30, 5.0),
    ("EXTENDED_SYS_STATE", 245, 2.0),
)
_READINESS_MESSAGES = frozenset(name for name, _msg_id, _hz in STREAM_REQUESTS[:3])


@dataclass(frozen=True)
class CopterReadiness:
    """ArduPilot pre-arm bit, EKF flags, and GPS fix. Not MAVSDK health.

    ``ready`` matches Copter autotest ``wait_ready_to_arm`` for an absolute
    position: SYS_STATUS pre-arm present, enabled, and healthy; the EKF
    status bits that must be set and the bits that must be clear; and
    ``GPS_RAW_INT.fix_type >= 3``. A missing message is its own failure.
    """

    sys_status_seen: bool = False
    prearm_present: bool = False
    prearm_enabled: bool = False
    prearm_healthy: bool = False
    ekf_seen: bool = False
    ekf_flags: int | None = None
    gps_seen: bool = False
    gps_fix_type: int | None = None

    def ready(self) -> bool:
        return not self.blockers()

    def blockers(self) -> tuple[str, ...]:
        """Checks that still refuse the arm."""
        failed: list[str] = []
        if not self.sys_status_seen:
            failed.append("SYS_STATUS not received")
        elif not self.prearm_present:
            failed.append("prearm check not present")
        elif not self.prearm_enabled:
            failed.append("prearm check not enabled")
        elif not self.prearm_healthy:
            failed.append("prearm check")
        if not self.ekf_seen or self.ekf_flags is None:
            failed.append("EKF_STATUS_REPORT not received")
        else:
            flags = self.ekf_flags
            for bit, label in _EKF_REQUIRED:
                if not _bit(flags, bit):
                    failed.append(label)
            for bit, label in _EKF_ERRORS:
                if _bit(flags, bit):
                    failed.append(label)
        if not self.gps_seen or self.gps_fix_type is None:
            failed.append("GPS_RAW_INT not received")
        elif self.gps_fix_type < GPS_FIX_3D:
            failed.append(f"GPS fix {self.gps_fix_type}")
        return tuple(failed)


def _bit(mask: int, flag: int) -> bool:
    return (mask & flag) == flag


def _mav_int(value: object) -> int | None:
    """Integer field from a MAVSDK fields_json object. Bools are not numbers."""
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            return None
        return int(value)
    if isinstance(value, str):
        text = value.strip()
        sign = text[1:] if text.startswith("-") else text
        if sign.isdigit():
            return int(text)
    return None


def apply_mavlink_message(
    state: CopterReadiness, message_name: str, fields: Mapping[str, object]
) -> CopterReadiness:
    """Fold one decoded MAVLink message into ``state``. Unknown names are ignored."""
    name = message_name.strip().upper()
    if name == "SYS_STATUS":
        present = _mav_int(fields.get("onboard_control_sensors_present"))
        enabled = _mav_int(fields.get("onboard_control_sensors_enabled"))
        health = _mav_int(fields.get("onboard_control_sensors_health"))
        if present is None or enabled is None or health is None:
            return state
        return CopterReadiness(
            sys_status_seen=True,
            prearm_present=_bit(present, MAV_SYS_STATUS_PREARM_CHECK),
            prearm_enabled=_bit(enabled, MAV_SYS_STATUS_PREARM_CHECK),
            prearm_healthy=_bit(health, MAV_SYS_STATUS_PREARM_CHECK),
            ekf_seen=state.ekf_seen,
            ekf_flags=state.ekf_flags,
            gps_seen=state.gps_seen,
            gps_fix_type=state.gps_fix_type,
        )
    if name == "EKF_STATUS_REPORT":
        flags = _mav_int(fields.get("flags"))
        if flags is None:
            return state
        return CopterReadiness(
            sys_status_seen=state.sys_status_seen,
            prearm_present=state.prearm_present,
            prearm_enabled=state.prearm_enabled,
            prearm_healthy=state.prearm_healthy,
            ekf_seen=True,
            ekf_flags=flags,
            gps_seen=state.gps_seen,
            gps_fix_type=state.gps_fix_type,
        )
    if name == "GPS_RAW_INT":
        fix_type = _mav_int(fields.get("fix_type"))
        if fix_type is None:
            return state
        return CopterReadiness(
            sys_status_seen=state.sys_status_seen,
            prearm_present=state.prearm_present,
            prearm_enabled=state.prearm_enabled,
            prearm_healthy=state.prearm_healthy,
            ekf_seen=state.ekf_seen,
            ekf_flags=state.ekf_flags,
            gps_seen=True,
            gps_fix_type=fix_type,
        )
    return state


def _ekf_ready_flags() -> int:
    flags = 0
    for bit, _label in _EKF_REQUIRED:
        flags |= bit
    return flags


def copter_readiness_ok() -> CopterReadiness:
    """Pre-arm, EKF, and a 3D fix. Tests use this as the default sample."""
    state = apply_mavlink_message(
        CopterReadiness(),
        "SYS_STATUS",
        {
            "onboard_control_sensors_present": MAV_SYS_STATUS_PREARM_CHECK,
            "onboard_control_sensors_enabled": MAV_SYS_STATUS_PREARM_CHECK,
            "onboard_control_sensors_health": MAV_SYS_STATUS_PREARM_CHECK,
        },
    )
    state = apply_mavlink_message(
        state, "EKF_STATUS_REPORT", {"flags": _ekf_ready_flags()}
    )
    return apply_mavlink_message(state, "GPS_RAW_INT", {"fix_type": GPS_FIX_3D})


def parse_mavlink_fields(fields_json: object) -> Mapping[str, object] | None:
    """Decode ``fields_json``. A non-object payload is ignored."""
    if not isinstance(fields_json, str) or not fields_json:
        return None
    try:
        payload = json.loads(fields_json)
    except json.JSONDecodeError:
        return None
    parsed = _string_mapping(payload)
    return parsed


def _string_mapping(value: object) -> Mapping[str, object] | None:
    if not isinstance(value, Mapping):
        return None
    return {str(key): item for key, item in value.items()}


def interval_us(rate_hz: float) -> int:
    """``MAV_CMD_SET_MESSAGE_INTERVAL`` param2. Never 0 or negative."""
    if rate_hz <= 0.0:
        raise ValueError("stream rate must be positive")
    return int(round(1_000_000 / rate_hz))


def command_long_fields(
    *, target_system: int, message_id: int, interval_us_value: int
) -> dict[str, object]:
    """COMMAND_LONG body. param3 stays 0; Copter denies any other value."""
    return {
        "target_system": target_system,
        "target_component": AUTOPILOT_COMPONENT_ID,
        "command": MAV_CMD_SET_MESSAGE_INTERVAL,
        "confirmation": 0,
        "param1": float(message_id),
        "param2": float(interval_us_value),
        "param3": 0.0,
        "param4": 0.0,
        "param5": 0.0,
        "param6": 0.0,
        "param7": 0.0,
    }


@dataclass(frozen=True)
class VehicleSample:
    x: float
    y: float
    z: float
    armed: bool
    landed_state: str
    flight_mode: str
    failsafe: tuple[str, ...]
    link_ok: bool
    readiness: CopterReadiness = field(default_factory=copter_readiness_ok)


@dataclass
class SquareReport:
    passed: bool
    takeoff_z_ned: float | None
    corner_errors_m: list[float] = field(default_factory=list)
    landed_state: str | None = None
    armed: bool | None = None
    failsafe: list[str] = field(default_factory=list)
    message: str = ""

    def as_dict(self, *, elapsed_s: float) -> dict[str, object]:
        return {
            "attempted": True,
            "passed": self.passed,
            "elapsed_s": elapsed_s,
            "uri": SITL_URI,
            "mavsdk_url": MAVSDK_URL,
            "copter_tag": COPTER_TAG,
            "copter_commit": COPTER_COMMIT,
            "takeoff_alt_m": TAKEOFF_ALT_M,
            "takeoff_z_ned": self.takeoff_z_ned,
            "side_m": SIDE_M,
            "speed_mps": SPEED_MPS,
            "corner_tolerance_m": CORNER_TOL_M,
            "corner_errors_m": list(self.corner_errors_m),
            "landed_state": self.landed_state,
            "armed": self.armed,
            "failsafe": list(self.failsafe),
            "message": self.message,
        }


def in_takeoff_band(z_ned: float, target_z: float = TAKEOFF_Z_NED) -> bool:
    """True when NED down is strictly inside 0.4 m of the 2.5 m climb."""
    return abs(z_ned - target_z) < ALT_TOL_M


def horizontal_error(x: float, y: float, corner: Sequence[float]) -> float:
    return math.hypot(x - corner[0], y - corner[1])


def within_corner(error_m: float) -> bool:
    """Corners must finish within 0.8 m horizontal."""
    return error_m <= CORNER_TOL_M


def square_corners(
    x: float, y: float, z: float
) -> tuple[tuple[float, float, float], ...]:
    """Four metre sides in local NED, altitude held, closing on the start."""
    side = SIDE_M
    return (
        (x + side, y, z),
        (x + side, y + side, z),
        (x, y + side, z),
        (x, y, z),
    )


def carrot_points(
    start: Sequence[float],
    end: Sequence[float],
    speed_mps: float = SPEED_MPS,
    hz: float = STREAM_HZ,
) -> tuple[tuple[float, float, float], ...]:
    """Position targets that walk the side at ``speed_mps``.

    The vehicle is already at ``start``. Points begin one step along the
    side and end on ``end``. Step length is ``speed / hz`` (0.05 m at the
    D-07 0.5 m/s and 10 Hz).
    """
    if speed_mps <= 0.0 or hz <= 0.0:
        raise ValueError("speed and stream rate must be positive")
    x0, y0, z0 = float(start[0]), float(start[1]), float(start[2])
    x1, y1, z1 = float(end[0]), float(end[1]), float(end[2])
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist == 0.0:
        return ((x1, y1, z1),)
    steps = max(1, int(round(dist / speed_mps * hz)))
    points: list[tuple[float, float, float]] = []
    for index in range(1, steps + 1):
        frac = index / steps
        points.append(
            (
                x0 + (x1 - x0) * frac,
                y0 + (y1 - y0) * frac,
                z0 + (z1 - z0) * frac,
            )
        )
    return tuple(points)


def positive_up_ned(z_ned: float) -> float:
    return -z_ned


def hardware_arming_refusal(argv: Sequence[str], env: Mapping[str, str]) -> str | None:
    """Reason to stop before any socket, or None when the invocation may fly."""
    if "--arm" in argv:
        return "refusing --arm; this script arms only the loopback SITL autopilot"
    if "OHHO_ARM_HARDWARE" in env:
        return (
            "refusing because OHHO_ARM_HARDWARE is set; "
            "CI leaves that variable unset"
        )
    return None


def _github_actions(env: Mapping[str, str]) -> bool:
    value = env.get("GITHUB_ACTIONS", "")
    return isinstance(value, str) and value.strip().lower() == "true"


def _require_ci_sim_opt_in(env: Mapping[str, str]) -> None:
    """Refuse to connect on GitHub Actions without the sim-only opt-in.

    ``OHHO_CI_SIM_ARMING`` and ``OHHO_CI_SITL`` must both be exactly ``1``.
    This does not arm, does not set ``OHHO_ARM_HARDWARE``, and does not
    bypass ``SafetyGate``. Callers outside GitHub Actions are unchanged.
    """
    if not _github_actions(env):
        return
    if env.get("OHHO_CI_SIM_ARMING") == "1" and env.get("OHHO_CI_SITL") == "1":
        return
    raise Refused(
        "refusing to connect under GitHub Actions without "
        "OHHO_CI_SIM_ARMING=1 and OHHO_CI_SITL=1. "
        "The aerial SafetyGate stays enforced. This script does not "
        "allocate a fake TTY or set OHHO_ARM_HARDWARE."
    )


def recipe() -> str:
    launch = " ".join(
        ("Tools/autotest/sim_vehicle.py", *SIM_VEHICLE_ARGS)
    )
    return "\n".join(
        [
            f"ArduPilot Copter SITL square ({COPTER_TAG}, {COPTER_COMMIT}).",
            f"Ohho URI: {SITL_URI}",
            f"MAVSDK 4 connection: {MAVSDK_URL}",
            f"Install: pip install \"{OHHO_PIP_URL}\"",
            "The square goes through Robot.connect and the aerial SafetyGate.",
            "Under GitHub Actions, OHHO_CI_SIM_ARMING=1 and OHHO_CI_SITL=1 "
            "are required before connect.",
            "CI leaves OHHO_ARM_HARDWARE unset.",
            "Headless launch (no DISPLAY, so sim_vehicle logs to /tmp/ArduCopter.log):",
            f"  {launch}",
            "",
            "1. Wait until the autopilot reports on_ground on the loopback link.",
            "2. Request SYS_STATUS, EKF_STATUS_REPORT, and GPS_RAW_INT on the "
            "open link (Copter's SR0_* rates are 0). Wait until the pre-arm "
            "check bit is healthy, the EKF flags match an absolute-position "
            "ready-to-arm, and GPS fix_type is at least 3. The wait is bounded "
            "and prints each check that is still failing. Arming checks stay "
            "at their defaults.",
            "3. Arm the aerial SafetyGate, stream a ground-hold offboard setpoint "
            f"at {STREAM_HZ:g} Hz, and arm the autopilot "
            "(MAV_CMD_COMPONENT_ARM_DISARM param1 1, param2 0). "
            "The gate refuses that arm unless a setpoint was forwarded in the "
            "last 300 ms. That stream starts MAVSDK offboard, which is Copter "
            "GUIDED. The arm counts only after the HEARTBEAT armed bit is "
            f"visible (polled for {ARM_CONFIRM_S:g} s; the heartbeat is 1 Hz). "
            f"A rejected arm is retried {ARM_ATTEMPTS} times.",
            f"4. Take off to {TAKEOFF_ALT_M} m above home. The ground-hold "
            "position stream stops first, then MAVSDK takeoff enters GUIDED "
            "and sends MAV_CMD_NAV_TAKEOFF. Pass the climb when "
            f"|z_ned + {TAKEOFF_ALT_M}| < {ALT_TOL_M} m for {ALT_HOLD_S:g} s.",
            f"5. Square, {SIDE_M:g} m sides, LOCAL_NED, carrot at {SPEED_MPS} m/s, "
            "altitude held, setpoints at "
            f"{STREAM_HZ:g} Hz. Each corner within {CORNER_TOL_M} m horizontal.",
            "6. Land. Pass when landed_state is on_ground, motors are disarmed "
            "with a normal disarm, and no failsafe flag was set after takeoff.",
            "",
            "This process connects only when --fly is passed.",
        ]
    )


def _mode(sample: VehicleSample) -> str:
    return sample.flight_mode.strip().lower()


def _raise_if_flags(sample: VehicleSample) -> None:
    if sample.failsafe:
        raise SquareFailed(f"failsafe flag set: {', '.join(sample.failsafe)}")


def _raise_if_failsafe(sample: VehicleSample) -> None:
    _raise_if_flags(sample)
    mode = _mode(sample)
    if mode in FAILSAFE_MODES:
        raise SquareFailed(f"autopilot left the square in mode {sample.flight_mode}")


def _wait(
    vehicle: object,
    timeout_s: float,
    predicate: Callable[[VehicleSample], bool],
    *,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period_s: float,
    on_fail: str,
) -> VehicleSample:
    deadline = monotonic() + timeout_s
    last: VehicleSample | None = None
    while monotonic() < deadline:
        last = _read(vehicle)
        _raise_if_flags(last)
        if predicate(last):
            return last
        sleep(period_s)
    detail = on_fail if last is None else f"{on_fail}; last={last.landed_state} z={last.z}"
    raise SquareFailed(detail)


def _read(vehicle: object) -> VehicleSample:
    read = getattr(vehicle, "read")
    sample = read()
    if not isinstance(sample, VehicleSample):
        raise SquareFailed("vehicle read() did not return a VehicleSample")
    return sample


def _hold_until(
    vehicle: object,
    ready: Callable[[VehicleSample, float | None], float | None],
    *,
    hold_s: float,
    timeout_s: float,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period_s: float,
    on_fail: str,
) -> VehicleSample:
    """Call ``ready`` each sample. It returns a start time once the condition holds."""
    deadline = monotonic() + timeout_s
    held_since: float | None = None
    last: VehicleSample | None = None
    while monotonic() < deadline:
        last = _read(vehicle)
        held_since = ready(last, held_since)
        if held_since is not None and monotonic() - held_since >= hold_s:
            return last
        sleep(period_s)
    raise SquareFailed(on_fail if last is None else f"{on_fail}; last z={last.z}")


def flight_wall_s(
    *,
    ground_timeout_s: float = GROUND_TIMEOUT_S,
    arm_timeout_s: float = ARM_TIMEOUT_S,
    arm_attempts: int = ARM_ATTEMPTS,
    arm_retry_pause_s: float = ARM_RETRY_PAUSE_S,
    arm_confirm_s: float = ARM_CONFIRM_S,
    climb_timeout_s: float = CLIMB_TIMEOUT_S,
    corner_timeout_s: float = CORNER_TIMEOUT_S,
    land_timeout_s: float = LAND_TIMEOUT_S,
    corners: int = SQUARE_CORNERS,
) -> float:
    """Sum of the time each phase is allowed to run, plus a small margin.

    The corner timer applies once per corner. The carrot between corners is
    outside that timer. Nothing here is shorter than the phase it covers.
    """
    carrot_s = corners * (SIDE_M / SPEED_MPS)
    return (
        ground_timeout_s
        + arm_timeout_s
        + arm_attempts * arm_confirm_s
        + max(0, arm_attempts - 1) * arm_retry_pause_s
        + climb_timeout_s
        + corners * corner_timeout_s
        + land_timeout_s
        + carrot_s
        + WALL_MARGIN_S
    )


def script_wall_s() -> float:
    """``--fly`` budget, including autopilot discovery before the phases."""
    return DISCOVERY_TIMEOUT_S + flight_wall_s()


def fly_square(
    vehicle: object,
    *,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    ground_timeout_s: float = GROUND_TIMEOUT_S,
    arm_timeout_s: float = ARM_TIMEOUT_S,
    arm_attempts: int = ARM_ATTEMPTS,
    arm_retry_pause_s: float = ARM_RETRY_PAUSE_S,
    arm_settle_s: float = ARM_SETTLE_S,
    arm_confirm_s: float = ARM_CONFIRM_S,
    climb_timeout_s: float = CLIMB_TIMEOUT_S,
    corner_timeout_s: float = CORNER_TIMEOUT_S,
    land_timeout_s: float = LAND_TIMEOUT_S,
) -> SquareReport:
    """Arm the gate, stream a setpoint, arm, climb, fly, and land.

    On a miss after the vehicle may be airborne, sends land and does not disarm.
    A gate refusal stays the exception ``main`` writes into square-result.json.
    """
    period = 1.0 / STREAM_HZ
    try:
        return _fly(
            vehicle,
            sleep=sleep,
            monotonic=monotonic,
            period=period,
            ground_timeout_s=ground_timeout_s,
            arm_timeout_s=arm_timeout_s,
            arm_attempts=arm_attempts,
            arm_retry_pause_s=arm_retry_pause_s,
            arm_settle_s=arm_settle_s,
            arm_confirm_s=arm_confirm_s,
            climb_timeout_s=climb_timeout_s,
            corner_timeout_s=corner_timeout_s,
            land_timeout_s=land_timeout_s,
        )
    except Exception:
        try:
            _call(vehicle, "land")
        except Exception as land_exc:
            print(f"land after failure failed: {land_exc}", file=sys.stderr)
        raise


def _call(vehicle: object, name: str, *args: object) -> None:
    fn = getattr(vehicle, name, None)
    if not callable(fn):
        raise SquareFailed(f"vehicle has no {name}()")
    try:
        fn(*args)
    except SquareFailed:
        raise
    except Exception as exc:
        raise SquareFailed(f"{name} failed: {exc}") from exc


def _fly(
    vehicle: object,
    *,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period: float,
    ground_timeout_s: float,
    arm_timeout_s: float,
    arm_attempts: int,
    arm_retry_pause_s: float,
    arm_settle_s: float,
    arm_confirm_s: float,
    climb_timeout_s: float,
    corner_timeout_s: float,
    land_timeout_s: float,
) -> SquareReport:
    print("waiting for on_ground telemetry", flush=True)
    _wait(
        vehicle,
        ground_timeout_s,
        lambda sample: sample.link_ok and sample.landed_state == "on_ground",
        sleep=sleep,
        monotonic=monotonic,
        period_s=period,
        on_fail="no on_ground telemetry before arm",
    )

    _wait_until_armable(
        vehicle,
        arm_timeout_s,
        sleep=sleep,
        monotonic=monotonic,
        period_s=period,
    )
    print("arming the safety gate", flush=True)
    _call(vehicle, "arm_gate")
    _arm_with_retries(
        vehicle,
        attempts=arm_attempts,
        pause_s=arm_retry_pause_s,
        settle_s=arm_settle_s,
        confirm_s=arm_confirm_s,
        sleep=sleep,
    )

    print(f"takeoff to {TAKEOFF_ALT_M} m", flush=True)
    _call(vehicle, "takeoff", TAKEOFF_ALT_M)

    def _climb_ready(sample: VehicleSample, held_since: float | None) -> float | None:
        _raise_if_failsafe(sample)
        if not sample.armed:
            raise SquareFailed("autopilot disarmed during climb")
        if in_takeoff_band(sample.z):
            return monotonic() if held_since is None else held_since
        return None

    climbed = _hold_until(
        vehicle,
        _climb_ready,
        hold_s=ALT_HOLD_S,
        timeout_s=climb_timeout_s,
        sleep=sleep,
        monotonic=monotonic,
        period_s=period,
        on_fail=(
            f"climb did not hold |z_ned + {TAKEOFF_ALT_M}| < {ALT_TOL_M} "
            f"for {ALT_HOLD_S:g} s"
        ),
    )
    origin = (climbed.x, climbed.y, climbed.z)
    print(
        f"climb held at z_ned={climbed.z:.3f} (target {TAKEOFF_Z_NED})",
        flush=True,
    )
    corners = square_corners(*origin)
    errors: list[float] = []
    cursor = origin
    for index, corner in enumerate(corners, start=1):
        print(f"leg {index} toward {corner[0]:.2f},{corner[1]:.2f}", flush=True)
        for point in carrot_points(cursor, corner):
            sample = _read(vehicle)
            _check_cruise(sample)
            _call(vehicle, "goto", point[0], point[1], point[2], 0.0)
            sleep(period)
        error = _hold_corner(
            vehicle,
            corner,
            timeout_s=corner_timeout_s,
            sleep=sleep,
            monotonic=monotonic,
            period=period,
        )
        errors.append(error)
        print(f"corner {index} error {error:.3f} m", flush=True)
        cursor = corner

    print("land", flush=True)
    _call(vehicle, "land")
    final = _wait_disarmed(
        vehicle,
        timeout_s=land_timeout_s,
        sleep=sleep,
        monotonic=monotonic,
        period=period,
    )
    report = SquareReport(
        passed=True,
        takeoff_z_ned=origin[2],
        corner_errors_m=errors,
        landed_state=final.landed_state,
        armed=final.armed,
        failsafe=list(final.failsafe),
        message="square passed",
    )
    print(
        f"passed corners={errors} landed={final.landed_state} armed={final.armed}",
        flush=True,
    )
    return report


def _arm_blockers(sample: VehicleSample) -> tuple[str, ...]:
    reasons: list[str] = []
    if not sample.link_ok:
        reasons.append("link down")
    if sample.landed_state != "on_ground":
        reasons.append(f"landed_state is {sample.landed_state}")
    if not sample.readiness.ready():
        reasons.extend(sample.readiness.blockers())
    return tuple(reasons)


def _wait_until_armable(
    vehicle: object,
    timeout_s: float,
    *,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period_s: float,
) -> VehicleSample:
    """Block until ArduPilot's pre-arm, EKF, and GPS say it can arm.

    Does not send an arm command. Stream requests are not arming checks.
    """
    note = getattr(vehicle, "arm_health_note", None)
    if isinstance(note, str) and note:
        print(note, flush=True)
    print("waiting until the autopilot is armable", flush=True)
    deadline = monotonic() + timeout_s
    started = monotonic()
    last_text: str | None = None
    last_log = started
    reasons: tuple[str, ...] = CopterReadiness().blockers()
    while monotonic() < deadline:
        _request_vehicle_streams(vehicle, monotonic())
        sample = _read(vehicle)
        _raise_if_flags(sample)
        reasons = _arm_blockers(sample)
        if not reasons:
            print("autopilot is armable", flush=True)
            return sample
        text = ", ".join(reasons)
        now = monotonic()
        if text != last_text or now - last_log >= ARM_READY_LOG_S:
            print(
                f"not armable yet ({now - started:.0f}s): {text}",
                flush=True,
            )
            last_text = text
            last_log = now
        sleep(period_s)
    raise SquareFailed(
        f"not armable within {timeout_s:g}s: {', '.join(reasons)}"
    )


def _request_vehicle_streams(vehicle: object, now: float) -> None:
    request = getattr(vehicle, "request_streams", None)
    if not callable(request):
        return
    try:
        request(now=now)
    except TypeError:
        request()


def _arm_hold_ned(sample: VehicleSample) -> tuple[float, float, float]:
    """Ground hold the aerial geofence accepts.

    The cylinder floor is the local origin. A positive NED z is below that
    floor, so the pre-arm stream uses the current horizontal position and
    z=0. The square still climbs with takeoff and holds the climbed altitude.
    """
    return (sample.x, sample.y, 0.0)


def _stream_setpoints(
    vehicle: object,
    duration_s: float,
    *,
    sleep: Callable[[float], None],
) -> VehicleSample:
    """Publish a ground hold at ``STREAM_HZ`` for ``duration_s`` seconds.

    A duration of 0 publishes once and does not sleep. The last publish is
    the newest setpoint, so the following ``arm_flight`` sees it inside the
    gate's 300 ms command window when nothing sleeps between the two calls.
    """
    period = 1.0 / STREAM_HZ
    remaining = duration_s
    while True:
        sample = _read(vehicle)
        _raise_if_flags(sample)
        x, y, z = _arm_hold_ned(sample)
        _call(vehicle, "stream_setpoint", x, y, z, 0.0)
        if remaining <= 1e-9:
            return sample
        step = period if remaining > period else remaining
        sleep(step)
        remaining -= step


def _confirm_armed(
    vehicle: object,
    *,
    settle_s: float,
    confirm_s: float,
    sleep: Callable[[float], None],
) -> VehicleSample:
    """Stream a ground hold until the HEARTBEAT armed bit is set.

    Copter sends HEARTBEAT at 1 Hz. ``subscribe_armed`` follows that bit, so
    a single sample ``settle_s`` after the arm command can still read
    disarmed. Keep the post-arm stream for at least ``settle_s`` and keep
    polling until ``confirm_s``.
    """
    if confirm_s + 1e-9 < settle_s:
        raise SquareFailed("arm confirm window is shorter than the settle stream")
    sample = _stream_setpoints(vehicle, settle_s, sleep=sleep)
    if sample.armed or confirm_s <= settle_s + 1e-9:
        return sample
    period = 1.0 / STREAM_HZ
    remaining = confirm_s - settle_s
    while remaining > 1e-9:
        _raise_if_flags(sample)
        if sample.armed:
            return sample
        step = period if remaining > period else remaining
        sleep(step)
        remaining -= step
        sample = _read(vehicle)
        x, y, z = _arm_hold_ned(sample)
        _call(vehicle, "stream_setpoint", x, y, z, 0.0)
    return sample


def _arm_with_retries(
    vehicle: object,
    *,
    attempts: int,
    pause_s: float,
    settle_s: float,
    confirm_s: float,
    sleep: Callable[[float], None],
) -> VehicleSample:
    """Arm a vehicle that is already armable. A few rejections are retried.

    The aerial gate refuses ``arm_flight`` unless an offboard setpoint was
    forwarded in the last 300 ms. This publishes that stream at ``STREAM_HZ``
    immediately before every arm, and keeps publishing through the settle
    window and the retry pause. Takeoff waits until the HEARTBEAT armed bit
    is set, not until a fixed 0.5 s sample happens to show it.
    """
    if attempts < 1:
        raise SquareFailed("arm attempts must be at least 1")
    last_error = "arm was not called"
    for attempt in range(1, attempts + 1):
        _stream_setpoints(vehicle, 0.0, sleep=sleep)
        print(f"arming autopilot (attempt {attempt}/{attempts})", flush=True)
        try:
            _call(vehicle, "arm")
        except SquareFailed as exc:
            last_error = str(exc)
            print(f"arm rejected: {last_error}", flush=True)
        else:
            sample = _confirm_armed(
                vehicle,
                settle_s=settle_s,
                confirm_s=confirm_s,
                sleep=sleep,
            )
            _raise_if_flags(sample)
            if sample.armed:
                return sample
            last_error = "arm call returned but the autopilot stayed disarmed"
            print(f"arm rejected: {last_error}", flush=True)
        if attempt < attempts:
            _stream_setpoints(vehicle, pause_s, sleep=sleep)
    raise SquareFailed(
        f"autopilot did not arm after {attempts} attempts: {last_error}"
    )


def _check_cruise(sample: VehicleSample) -> None:
    _raise_if_failsafe(sample)
    if not sample.armed:
        raise SquareFailed("autopilot disarmed during the square")
    if sample.landed_state == "on_ground":
        raise SquareFailed("vehicle was on the ground during the square")


def _hold_corner(
    vehicle: object,
    corner: Sequence[float],
    *,
    timeout_s: float,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period: float,
) -> float:
    deadline = monotonic() + timeout_s
    held_since: float | None = None
    last_error = math.inf
    while monotonic() < deadline:
        sample = _read(vehicle)
        _check_cruise(sample)
        if not in_takeoff_band(sample.z, corner[2]):
            held_since = None
        else:
            last_error = horizontal_error(sample.x, sample.y, corner)
            if within_corner(last_error):
                if held_since is None:
                    held_since = monotonic()
                elif monotonic() - held_since >= CORNER_HOLD_S:
                    return last_error
            else:
                held_since = None
        _call(vehicle, "goto", corner[0], corner[1], corner[2], 0.0)
        sleep(period)
    raise SquareFailed(
        f"corner {corner[0]:.2f},{corner[1]:.2f} stayed outside "
        f"{CORNER_TOL_M} m (last error {last_error:.2f} m)"
    )


def _wait_disarmed(
    vehicle: object,
    *,
    timeout_s: float,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period: float,
) -> VehicleSample:
    deadline = monotonic() + timeout_s
    requested = False
    last: VehicleSample | None = None
    while monotonic() < deadline:
        last = _read(vehicle)
        if last.failsafe:
            raise SquareFailed(
                f"failsafe flag set after takeoff: {', '.join(last.failsafe)}"
            )
        up = positive_up_ned(last.z)
        on_ground = last.landed_state == "on_ground" and up <= DISARM_ALT_M
        if on_ground and last.armed and not requested:
            _call(vehicle, "disarm")
            requested = True
            last = _read(vehicle)
        if (
            last.landed_state == "on_ground"
            and not last.armed
            and not last.failsafe
            and positive_up_ned(last.z) <= DISARM_ALT_M
        ):
            return last
        sleep(period)
    raise SquareFailed(
        "land did not finish on_ground, disarmed, with no failsafe flag"
    )


class _FlightRobot(Protocol):
    """The Ohho ``Robot`` methods this square calls. Imported only for ``--fly``."""

    def telemetry(self) -> object: ...

    def arm(self) -> object: ...

    def arm_flight(self) -> object: ...

    def takeoff(self, alt_m: float) -> object: ...

    def offboard_position(
        self, x: float, y: float, z: float, yaw: float, frame: object
    ) -> object: ...

    def land(self) -> object: ...

    def disarm_flight(self) -> object: ...

    def disconnect(self) -> None: ...


def _mavlink_direct(system: object) -> object:
    """MavlinkDirect on a system Ohho already discovered.

    mavsdk 4.0.6 has no ``MavlinkPassthrough`` plugin. ``MavlinkDirect`` is
    the same-link replacement: it does not open a socket. A second ``udpin``
    listener would take the only SITL stream away from Ohho.
    """
    from mavsdk.plugins.mavlink_direct import MavlinkDirect

    return MavlinkDirect(system)


def _direct_message(
    *,
    message_name: str,
    target_system: int,
    fields: Mapping[str, object],
) -> object:
    fields_json = json.dumps(fields)
    try:
        from mavsdk.plugins.mavlink_direct import MavlinkMessage
    except ImportError:
        return _FallbackDirectMessage(
            message_name=message_name,
            system_id=MAVSDK_GCS_SYSTEM_ID,
            component_id=MAVSDK_GCS_COMPONENT_ID,
            target_system_id=target_system,
            target_component_id=AUTOPILOT_COMPONENT_ID,
            fields_json=fields_json,
        )
    return MavlinkMessage(
        message_name=message_name,
        system_id=MAVSDK_GCS_SYSTEM_ID,
        component_id=MAVSDK_GCS_COMPONENT_ID,
        target_system_id=target_system,
        target_component_id=AUTOPILOT_COMPONENT_ID,
        fields_json=fields_json,
    )


class _FallbackDirectMessage:
    """Stand-in used when the unit tests do not have mavsdk installed."""

    def __init__(
        self,
        *,
        message_name: str,
        system_id: int,
        component_id: int,
        target_system_id: int,
        target_component_id: int,
        fields_json: str,
    ) -> None:
        self.message_name = message_name
        self.system_id = system_id
        self.component_id = component_id
        self.target_system_id = target_system_id
        self.target_component_id = target_component_id
        self.fields_json = fields_json


def _target_system_id(system: object) -> int:
    getter = getattr(system, "get_system_id", None)
    if callable(getter):
        try:
            value = getter()
        except Exception:
            value = None
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            return value
    return 1


def _mavlink_transport(robot: object) -> object | None:
    """Inner MAVLink transport when a SafetyGate wraps it.

    Readiness uses this only to find the open MAVSDK session. It does not
    send arm or takeoff on that object.
    """
    transport = getattr(robot, "transport", None)
    if transport is None:
        return None
    inner = getattr(transport, "_inner", None)
    if inner is not None:
        return inner
    return transport


def _autopilot_system(robot: object) -> object | None:
    """MAVSDK system kept alive by the transport's action plugin.

    ohho-os does not publish health. MAVSDK 4 stores the system on the
    plugin as ``_owner``. A new ``udpin`` connection would take the only
    SITL stream away from that session. A SafetyGate hides the link on
    ``_inner``; this reads that inner transport and does not command it.
    """
    transport = _mavlink_transport(robot)
    link = getattr(transport, "_link", None)
    if link is None:
        return None
    for attr in ("_action", "_offboard"):
        plugin = getattr(link, attr, None)
        owner = getattr(plugin, "_owner", None)
        if owner is not None and getattr(owner, "_handle", None):
            return owner
    sdk = getattr(link, "_sdk", None)
    getter = getattr(sdk, "get_systems", None)
    if not callable(getter):
        return None
    systems = getter()
    chosen: object | None = None
    for system in systems:
        has_autopilot = getattr(system, "has_autopilot", None)
        if callable(has_autopilot) and has_autopilot():
            return system
        if chosen is None:
            chosen = system
    return chosen


class CopterReadinessWatch:
    """SYS_STATUS, EKF_STATUS_REPORT, and GPS_RAW_INT on Ohho's open session."""

    def __init__(self) -> None:
        self._direct: object | None = None
        self._note = "ardupilot readiness not attached"
        self._lock = threading.Lock()
        self._latest = CopterReadiness()
        self._target_system = 1
        self._last_request: float | None = None
        self._announced = False

    @property
    def note(self) -> str:
        return self._note

    def attach(
        self,
        robot: object,
        *,
        direct_factory: Callable[[object], object] | None = None,
    ) -> None:
        system = _autopilot_system(robot)
        if system is None:
            self._note = (
                "ardupilot readiness not attached: "
                "the open session has no autopilot system"
            )
            return
        self._target_system = _target_system_id(system)
        factory = _mavlink_direct if direct_factory is None else direct_factory
        try:
            direct = factory(system)
        except Exception as exc:
            self._note = f"ardupilot readiness not attached: {exc}"
            return
        subscribe = getattr(direct, "subscribe_message", None)
        if not callable(subscribe):
            self._note = (
                "ardupilot readiness not attached: "
                "mavlink direct has no subscribe_message()"
            )
            return
        errors: list[str] = []
        subscribed = 0
        for name, _message_id, _rate_hz in STREAM_REQUESTS:
            if name not in _READINESS_MESSAGES:
                continue
            try:
                subscribe(name, self._on_message)
                subscribed += 1
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if subscribed == 0:
            detail = "; ".join(errors) or "no message subscription"
            self._note = f"ardupilot readiness not attached: {detail}"
            return
        self._direct = direct
        self._note = (
            "reading ArduPilot SYS_STATUS, EKF_STATUS_REPORT, and "
            "GPS_RAW_INT on the open session"
        )
        if errors:
            self._note += f" ({'; '.join(errors)})"
        self.request_streams(now=0.0, force=True)

    def request_streams(self, *, now: float | None = None, force: bool = False) -> int:
        """Ask Copter to emit the arm-gate and flight messages. Returns how many were sent."""
        direct = self._direct
        if direct is None:
            return 0
        stamp = time.monotonic() if now is None else now
        if (
            not force
            and self._last_request is not None
            and stamp - self._last_request < STREAM_REQUEST_PERIOD_S
        ):
            return 0
        send = getattr(direct, "send_message", None)
        if not callable(send):
            self._note = (
                "reading ArduPilot readiness, but mavlink direct has no send_message()"
            )
            return 0
        sent: list[str] = []
        errors: list[str] = []
        for name, message_id, rate_hz in STREAM_REQUESTS:
            fields = command_long_fields(
                target_system=self._target_system,
                message_id=message_id,
                interval_us_value=interval_us(rate_hz),
            )
            message = _direct_message(
                message_name="COMMAND_LONG",
                target_system=self._target_system,
                fields=fields,
            )
            try:
                send(message)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                continue
            sent.append(f"{name} {rate_hz:g} Hz")
        self._last_request = stamp
        if sent and not self._announced:
            print(
                "requested ArduPilot message intervals: " + ", ".join(sent),
                flush=True,
            )
            self._announced = True
        if errors:
            print(
                "message interval request failed: " + "; ".join(errors),
                flush=True,
            )
        return len(sent)

    def _on_message(self, message: object, _user_data: object = None) -> None:
        name = getattr(message, "message_name", None)
        if not isinstance(name, str):
            return
        fields = parse_mavlink_fields(getattr(message, "fields_json", None))
        if fields is None:
            return
        with self._lock:
            self._latest = apply_mavlink_message(self._latest, name, fields)

    def poll(self) -> CopterReadiness:
        with self._lock:
            return self._latest

    def close(self) -> None:
        direct = self._direct
        self._direct = None
        destroy = getattr(direct, "destroy", None)
        if callable(destroy):
            try:
                destroy()
            except Exception:
                return


class OhhoVehicle:
    """Aerial ``SafetyGate`` flight methods on one loopback MAVLink transport.

    ``frame`` is the Ohho ``Frame`` passed to every position setpoint. The
    opener passes ``Frame.LOCAL_NED``. ``arm_gate`` is ``Robot.arm``. The
    autopilot arm is ``arm_flight``. A gate ``last_error`` becomes
    ``SquareFailed`` so square-result.json reports that refusal.
    """

    def __init__(
        self,
        robot: _FlightRobot,
        frame: object,
        *,
        health: CopterReadinessWatch | None = None,
    ) -> None:
        self._robot = robot
        self._frame = frame
        self._health = CopterReadinessWatch() if health is None else health
        if health is None:
            self._health.attach(robot)

    @property
    def arm_health_note(self) -> str:
        return self._health.note

    def request_streams(self, *, now: float | None = None) -> int:
        return self._health.request_streams(now=now)

    def read(self) -> VehicleSample:
        tele = self._robot.telemetry()
        custom = getattr(tele, "custom", None)
        if not isinstance(custom, Mapping):
            custom = {}
        drone = custom.get("drone")
        if not isinstance(drone, Mapping):
            raise SquareFailed("mavlink telemetry has no drone sample")
        pose = drone.get("pose")
        if not isinstance(pose, Mapping):
            raise SquareFailed("mavlink telemetry has no pose")
        flags = drone.get("failsafe") or ()
        if isinstance(flags, str) or not isinstance(flags, Sequence):
            flags = ()
        return VehicleSample(
            x=float(pose["x"]),
            y=float(pose["y"]),
            z=float(pose["z"]),
            armed=bool(drone.get("armed")),
            landed_state=str(drone.get("landed_state", "unknown")),
            flight_mode=str(drone.get("flight_mode", "unknown")),
            failsafe=tuple(str(flag) for flag in flags),
            link_ok=bool(drone.get("link_ok")),
            readiness=self._health.poll(),
        )

    def _gate(self) -> object | None:
        return getattr(self._robot, "transport", None)

    def _flight(self, label: str, fn: Callable[[], object]) -> None:
        gate = self._gate()
        if gate is not None and hasattr(gate, "last_error"):
            gate.last_error = None
        fn()
        if gate is None:
            return
        if bool(getattr(gate, "latched", False)):
            reason = getattr(gate, "latch_reason", None)
            raise SquareFailed(f"{label} latched the safety gate ({reason})")
        error = getattr(gate, "last_error", None)
        if error:
            raise SquareFailed(f"{label} refused by SafetyGate: {error}")

    def arm_gate(self) -> None:
        """Arm the safety gate. This does not arm the autopilot."""
        gate = self._gate()
        if bool(getattr(gate, "latched", False)):
            reason = getattr(gate, "latch_reason", None)
            raise SquareFailed(f"arm latched the safety gate ({reason})")
        if gate is not None and hasattr(gate, "last_error"):
            gate.last_error = None
        ok = self._robot.arm()
        if ok is False:
            error = getattr(gate, "last_error", None)
            detail = f": {error}" if error else ""
            raise SquareFailed(f"SafetyGate.arm() refused the gate{detail}")
        if bool(getattr(gate, "latched", False)):
            reason = getattr(gate, "latch_reason", None)
            raise SquareFailed(f"arm latched the safety gate ({reason})")

    def arm(self) -> None:
        self._flight("arm_flight", self._robot.arm_flight)

    def takeoff(self, alt_m: float) -> None:
        self._flight("takeoff", lambda: self._robot.takeoff(alt_m))

    def stream_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        self._flight(
            "offboard_position",
            lambda: self._robot.offboard_position(x, y, z, yaw, self._frame),
        )

    def goto(self, x: float, y: float, z: float, yaw: float) -> None:
        self.stream_setpoint(x, y, z, yaw)

    def land(self) -> None:
        self._flight("land", self._robot.land)

    def disarm(self) -> None:
        self._flight("disarm_flight", self._robot.disarm_flight)

    def close(self) -> None:
        self._health.close()
        self._robot.disconnect()


def _refuse_unless_loopback_sitl(uri: str) -> None:
    """Check the MAVSDK URL and the loopback flag before any socket."""
    from ohho.adapters.mavlink_drone import parse_mavlink_uri

    target = parse_mavlink_uri(uri)
    if target.mavsdk_url != MAVSDK_URL:
        raise Refused(f"refusing unexpected MAVSDK URL {target.mavsdk_url}")
    if not target.simulation_target or target.requires_hardware_arming:
        raise Refused("refusing a mavlink target that is not loopback SITL")


def _assert_aerial_gate(robot: object, safety_gate_type: type[object]) -> None:
    """Fail loudly unless ``Robot.connect`` wrapped an aerial SafetyGate."""
    gate = getattr(robot, "transport", None)
    if not isinstance(gate, safety_gate_type):
        raise Refused(
            "SITL connected without a SafetyGate. "
            "Robot.connect must wrap the mavlink transport."
        )
    profile = getattr(gate, "profile", None)
    if getattr(profile, "aerial", False) is not True:
        raise Refused("SafetyGate profile is not the aerial profile")
    inner = getattr(gate, "_inner", None)
    if inner is None:
        raise Refused("SafetyGate has no wrapped mavlink transport")
    mavsdk_url = getattr(inner, "mavsdk_url", None)
    if mavsdk_url != MAVSDK_URL:
        raise Refused(f"refusing unexpected MAVSDK URL {mavsdk_url}")
    simulation = getattr(inner, "simulation_target", False) is True
    hardware = getattr(inner, "requires_hardware_arming", True) is not False
    if not simulation or hardware:
        raise Refused("refusing a mavlink target that is not loopback SITL")
    if getattr(gate, "requires_hardware_arming", False):
        raise Refused(
            "SafetyGate.requires_hardware_arming is true for this loopback "
            "mavlink SITL target. Refusing to set OHHO_ARM_HARDWARE. "
            "GitHub Actions must set OHHO_CI_SIM_ARMING=1 and OHHO_CI_SITL=1 "
            "with OHHO_ARM_HARDWARE absent."
        )


def open_sitl() -> OhhoVehicle:
    """Connect via ``Robot.connect`` so the aerial ``SafetyGate`` wraps SITL.

    Under GitHub Actions this returns before connecting unless
    ``OHHO_CI_SIM_ARMING`` and ``OHHO_CI_SITL`` are both exactly ``1``.
    The loopback and MAVSDK URL checks run before the socket, and again on
    the wrapped transport. ``OHHO_ARM_HARDWARE`` is refused whenever it is
    present. The returned vehicle fails if the transport is not an aerial gate.
    """
    if "OHHO_ARM_HARDWARE" in os.environ:
        raise Refused(
            "refusing because OHHO_ARM_HARDWARE is set; CI leaves that variable unset"
        )
    _require_ci_sim_opt_in(os.environ)
    _refuse_unless_loopback_sitl(SITL_URI)
    from ohho.adapters.mavlink_drone import MavlinkDroneTransport
    from ohho.aerial import Frame
    from ohho.robot import Robot
    from ohho.safety import SafetyGate

    # Robot.connect does not take a discovery timeout. The square's wait for
    # the first autopilot stays DISCOVERY_TIMEOUT_S (120 s), not the adapter
    # default of 10 s.
    init = MavlinkDroneTransport.__init__
    defaults = getattr(init, "__kwdefaults__", None)
    if not isinstance(defaults, dict) or "timeout_s" not in defaults:
        raise Refused(
            "mavlink transport has no timeout_s default; "
            "refusing to connect with a shorter discovery wait"
        )
    saved_timeout = defaults["timeout_s"]
    defaults["timeout_s"] = DISCOVERY_TIMEOUT_S
    try:
        robot = Robot.connect(ROBOT_ID, SITL_URI, runtime="native")
    finally:
        defaults["timeout_s"] = saved_timeout
    try:
        _assert_aerial_gate(robot, SafetyGate)
    except Exception:
        try:
            robot.disconnect()
        except Exception:
            pass
        raise
    return OhhoVehicle(robot, Frame.LOCAL_NED)


def _write_result(path: str, payload: Mapping[str, object]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def _failure_payload(exc: BaseException, elapsed_s: float) -> dict[str, object]:
    return {
        "attempted": True,
        "passed": False,
        "elapsed_s": elapsed_s,
        "uri": SITL_URI,
        "mavsdk_url": MAVSDK_URL,
        "copter_tag": COPTER_TAG,
        "copter_commit": COPTER_COMMIT,
        "takeoff_alt_m": TAKEOFF_ALT_M,
        "side_m": SIDE_M,
        "speed_mps": SPEED_MPS,
        "corner_tolerance_m": CORNER_TOL_M,
        "message": str(exc),
    }


class _FlightWall:
    """End a hung ``--fly`` as a failed result instead of the job timeout.

    The timer is real time. A blocked MAVSDK call does not advance the
    flight's own phase clocks, so this does not live inside those loops.
    Expiry attempts land, writes square-result.json, and exits even if that
    land call is itself stuck.
    """

    def __init__(
        self,
        budget_s: float,
        vehicle_box: list[object],
        result_path: str,
        started: float,
        exit_fn: Callable[[int], None],
        *,
        land_attempt_s: float = WALL_LAND_ATTEMPT_S,
    ) -> None:
        self.budget_s = budget_s
        self._vehicle_box = vehicle_box
        self._result_path = result_path
        self._started = started
        self._exit_fn = exit_fn
        self._land_attempt_s = land_attempt_s
        self._lock = threading.Lock()
        self._done = False
        self._timer = threading.Timer(budget_s, self._fire)
        self._timer.daemon = True

    def start(self) -> None:
        self._timer.start()

    def cancel(self) -> None:
        with self._lock:
            self._done = True
        self._timer.cancel()

    def _fire(self) -> None:
        with self._lock:
            if self._done:
                return
            self._done = True
        message = (
            f"square wall clock expired after {self.budget_s:g}s "
            "before the flight returned"
        )
        print(message, file=sys.stderr, flush=True)
        vehicle = self._vehicle_box[0]
        if vehicle is not None:
            _attempt_land(vehicle, self._land_attempt_s)
        elapsed = time.perf_counter() - self._started
        try:
            _write_result(
                self._result_path,
                _failure_payload(SquareFailed(message), elapsed),
            )
        except Exception as write_exc:
            print(
                f"could not write {self._result_path}: {write_exc}",
                file=sys.stderr,
                flush=True,
            )
        self._exit_fn(1)


def _attempt_land(vehicle: object, attempt_s: float) -> None:
    """Call ``land`` without waiting forever if that call is deadlocked."""
    done = threading.Event()

    def run() -> None:
        try:
            _call(vehicle, "land")
        except Exception as exc:
            print(f"land after wall clock failed: {exc}", file=sys.stderr, flush=True)
        finally:
            done.set()

    threading.Thread(target=run, name="ohho-square-wall-land", daemon=True).start()
    done.wait(attempt_s)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--print-recipe",
        action="store_true",
        help="print the square steps and exit 0 (the default)",
    )
    parser.add_argument(
        "--fly",
        action="store_true",
        help="connect to loopback SITL and fly the square",
    )
    parser.add_argument(
        "--arm",
        action="store_true",
        help="refused; hardware arming is not a flag on this script",
    )
    parser.add_argument(
        "--result",
        default="square-result.json",
        help="where --fly writes the pass/fail JSON",
    )
    args = parser.parse_args(None if argv is None else list(argv))
    refusal = hardware_arming_refusal(sys.argv[1:] if argv is None else argv, os.environ)
    if refusal is not None:
        print(refusal, file=sys.stderr)
        return 2
    if not args.fly:
        print(recipe())
        return 0
    started = time.perf_counter()
    vehicle_box: list[object] = [None]
    wall = _FlightWall(script_wall_s(), vehicle_box, args.result, started, os._exit)
    wall.start()
    vehicle: OhhoVehicle | None = None
    try:
        vehicle = open_sitl()
        vehicle_box[0] = vehicle
        report = fly_square(vehicle)
        wall.cancel()
        elapsed = time.perf_counter() - started
        _write_result(args.result, report.as_dict(elapsed_s=elapsed))
        print(f"elapsed_s={elapsed:.3f}", flush=True)
        return 0
    except Refused as exc:
        wall.cancel()
        print(str(exc), file=sys.stderr)
        return 2
    except Exception as exc:
        wall.cancel()
        elapsed = time.perf_counter() - started
        try:
            _write_result(args.result, _failure_payload(exc, elapsed))
        except Exception as write_exc:
            print(f"could not write {args.result}: {write_exc}", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        print(f"elapsed_s={elapsed:.3f}", flush=True)
        return 1
    finally:
        wall.cancel()
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception as exc:
                print(f"disconnect failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
