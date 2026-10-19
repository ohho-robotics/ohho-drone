"""D-07 square on ArduPilot Copter SITL through Ohho ``mavlink+udp://``.

OHH-306. The workflow starts ``sim_vehicle.py -v ArduCopter`` headless at the
pinned Copter-4.7.2 tag, then this script waits until MAVSDK health says the
copter is armable, arms, climbs to 2.5 m, flies a 4 m square, and lands.
``--print-recipe`` only prints those steps.

The script leaves ``OHHO_ARM_HARDWARE`` unset and refuses ``--arm``. The
loopback transport is a simulation target, so this path does not wrap
``SafetyGate`` (that gate still treats every ``mavlink`` protocol as
hardware and refuses arm and takeoff when ``CI`` is set).

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


# MAVSDK Telemetry.Health fields. The first three gate arming. The rest are
# logged while those three are still false so the timeout names the cause.
_REQUIRED_HEALTH = (
    ("is_armable", "is_armable"),
    ("is_global_position_ok", "global position"),
    ("is_home_position_ok", "home position"),
)
_OPTIONAL_HEALTH = (
    ("is_local_position_ok", "local position"),
    ("is_accelerometer_calibration_ok", "accelerometer calibration"),
    ("is_gyrometer_calibration_ok", "gyrometer calibration"),
    ("is_magnetometer_calibration_ok", "magnetometer calibration"),
)
_HEALTH_FIELDS = tuple(name for name, _label in _REQUIRED_HEALTH + _OPTIONAL_HEALTH)


@dataclass(frozen=True)
class ArmReadiness:
    """MAVSDK health, or the same flags when Ohho publishes them.

    ``ready`` is is_armable plus a global position and a home position.
    Calibration and local-position flags are diagnostic. A false compass bit
    on SITL does not block once the pre-arm bit is set.
    """

    reported: bool = False
    is_armable: bool | None = None
    is_global_position_ok: bool | None = None
    is_home_position_ok: bool | None = None
    is_local_position_ok: bool | None = None
    is_accelerometer_calibration_ok: bool | None = None
    is_gyrometer_calibration_ok: bool | None = None
    is_magnetometer_calibration_ok: bool | None = None

    def ready(self) -> bool:
        return (
            self.reported
            and self.is_armable is True
            and self.is_global_position_ok is True
            and self.is_home_position_ok is True
        )

    def blockers(self) -> tuple[str, ...]:
        """Checks that still refuse the arm."""
        if not self.reported:
            return ("mavsdk health not received",)
        failed: list[str] = []
        for attr, label in _REQUIRED_HEALTH:
            if getattr(self, attr) is not True:
                failed.append(label)
        return tuple(failed)

    def detail(self) -> tuple[str, ...]:
        """Blockers plus calibration and local-position flags that are false."""
        extra: list[str] = []
        if self.reported:
            for attr, label in _OPTIONAL_HEALTH:
                if getattr(self, attr) is False:
                    extra.append(label)
        return self.blockers() + tuple(extra)


def arm_readiness_ok() -> ArmReadiness:
    """Health that may arm. Tests use this as the default sample."""
    return ArmReadiness(
        reported=True,
        is_armable=True,
        is_global_position_ok=True,
        is_home_position_ok=True,
        is_local_position_ok=True,
        is_accelerometer_calibration_ok=True,
        is_gyrometer_calibration_ok=True,
        is_magnetometer_calibration_ok=True,
    )


def _health_flag(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    return None


def arm_readiness_from_mapping(payload: Mapping[str, object]) -> ArmReadiness | None:
    """Build readiness from a dict that contains ``is_armable``."""
    if "is_armable" not in payload:
        return None
    flags = {name: _health_flag(payload.get(name)) for name in _HEALTH_FIELDS}
    if not any(flag is not None for flag in flags.values()):
        return None
    return ArmReadiness(reported=True, **flags)


def arm_readiness_from_health(health: object) -> ArmReadiness | None:
    """Build readiness from a MAVSDK ``Health`` object."""
    if health is None:
        return None
    payload = {name: getattr(health, name, None) for name in _HEALTH_FIELDS}
    return arm_readiness_from_mapping(payload)


def _string_mapping(value: object) -> Mapping[str, object] | None:
    if not isinstance(value, Mapping):
        return None
    return {str(key): item for key, item in value.items()}


def arm_readiness_from_telemetry(
    custom: Mapping[str, object], drone: Mapping[str, object]
) -> ArmReadiness | None:
    """Ohho-published health, when the backend has started sending it.

    Checked before the direct MAVSDK plugin. ``custom["arm_health"]`` is the
    place that does not change ``DRONE_TELEMETRY_KEYS``.
    """
    for source in (
        custom.get("arm_health"),
        drone.get("health"),
        drone if "is_armable" in drone else None,
    ):
        payload = _string_mapping(source)
        if payload is None:
            continue
        parsed = arm_readiness_from_mapping(payload)
        if parsed is not None:
            return parsed
    return None


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
    readiness: ArmReadiness = field(default_factory=arm_readiness_ok)


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
            "CI leaves OHHO_ARM_HARDWARE unset.",
            "Headless launch (no DISPLAY, so sim_vehicle logs to /tmp/ArduCopter.log):",
            f"  {launch}",
            "",
            "1. Wait until the autopilot reports on_ground on the loopback link.",
            "2. Wait until MAVSDK health reports is_armable, a global position, "
            "and a home position. The wait is bounded and prints each check "
            "that is still failing. Arming checks stay at their defaults.",
            "3. Arm the autopilot (MAV_CMD_COMPONENT_ARM_DISARM param1 1, param2 0). "
            f"A rejected arm is retried {ARM_ATTEMPTS} times.",
            f"4. Take off to {TAKEOFF_ALT_M} m. Pass the climb when "
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


def fly_square(
    vehicle: object,
    *,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    ground_timeout_s: float = GROUND_TIMEOUT_S,
    arm_timeout_s: float = ARM_TIMEOUT_S,
    arm_attempts: int = ARM_ATTEMPTS,
    arm_retry_pause_s: float = ARM_RETRY_PAUSE_S,
    climb_timeout_s: float = CLIMB_TIMEOUT_S,
    corner_timeout_s: float = CORNER_TIMEOUT_S,
    land_timeout_s: float = LAND_TIMEOUT_S,
) -> SquareReport:
    """Arm, climb, fly the square, land. Raises ``SquareFailed`` on a miss.

    On a miss after the vehicle may be airborne, sends land and does not disarm.
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
            climb_timeout_s=climb_timeout_s,
            corner_timeout_s=corner_timeout_s,
            land_timeout_s=land_timeout_s,
        )
    except Exception:
        _call(vehicle, "land")
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
    _arm_with_retries(
        vehicle,
        attempts=arm_attempts,
        pause_s=arm_retry_pause_s,
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
        reasons.extend(sample.readiness.detail())
    return tuple(reasons)


def _wait_until_armable(
    vehicle: object,
    timeout_s: float,
    *,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    period_s: float,
) -> VehicleSample:
    """Block until health says the copter can arm. Do not send an arm command."""
    note = getattr(vehicle, "arm_health_note", None)
    if isinstance(note, str) and note:
        print(note, flush=True)
    print("waiting until the autopilot is armable", flush=True)
    deadline = monotonic() + timeout_s
    started = monotonic()
    last_text: str | None = None
    last_log = started
    reasons: tuple[str, ...] = ("mavsdk health not received",)
    while monotonic() < deadline:
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


def _arm_with_retries(
    vehicle: object,
    *,
    attempts: int,
    pause_s: float,
    sleep: Callable[[float], None],
) -> VehicleSample:
    """Arm a vehicle that is already armable. A few rejections are retried."""
    if attempts < 1:
        raise SquareFailed("arm attempts must be at least 1")
    last_error = "arm was not called"
    for attempt in range(1, attempts + 1):
        print(f"arming autopilot (attempt {attempt}/{attempts})", flush=True)
        try:
            _call(vehicle, "arm")
        except SquareFailed as exc:
            last_error = str(exc)
            print(f"arm rejected: {last_error}", flush=True)
        else:
            sleep(0.5)
            sample = _read(vehicle)
            _raise_if_flags(sample)
            if sample.armed:
                return sample
            last_error = "arm call returned but the autopilot stayed disarmed"
            print(f"arm rejected: {last_error}", flush=True)
        if attempt < attempts:
            sleep(pause_s)
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

    def arm_flight(self) -> object: ...

    def takeoff(self, alt_m: float) -> object: ...

    def offboard_position(
        self, x: float, y: float, z: float, yaw: float, frame: object
    ) -> object: ...

    def land(self) -> object: ...

    def disarm_flight(self) -> object: ...

    def disconnect(self) -> None: ...


def _mavsdk_telemetry(system: object) -> object:
    """Second Telemetry plugin on a system Ohho already discovered.

    This does not open a socket. The SITL stream is already owned by the
    session that discovered the autopilot.
    """
    from mavsdk.plugins.telemetry import Telemetry

    return Telemetry(system)


def _autopilot_system(robot: object) -> object | None:
    """MAVSDK system kept alive by the transport's action plugin.

    ohho-os does not publish health. MAVSDK 4 stores the system on the
    plugin as ``_owner``. A new ``udpin`` connection would take the only
    SITL stream away from that session.
    """
    transport = getattr(robot, "transport", None)
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


class ArmHealthWatch:
    """Poll MAVSDK health on the session Ohho already opened."""

    def __init__(self) -> None:
        self._plugin: object | None = None
        self._note = "mavsdk health not attached"
        self._lock = threading.Lock()
        self._latest = ArmReadiness()

    @property
    def note(self) -> str:
        return self._note

    def attach(
        self,
        robot: object,
        *,
        telemetry_factory: Callable[[object], object] | None = None,
    ) -> None:
        system = _autopilot_system(robot)
        if system is None:
            self._note = (
                "mavsdk health not attached: the open session has no autopilot system"
            )
            return
        factory = _mavsdk_telemetry if telemetry_factory is None else telemetry_factory
        try:
            plugin = factory(system)
        except Exception as exc:
            self._note = f"mavsdk health not attached: {exc}"
            return
        self._plugin = plugin
        subscribe_error = ""
        subscribed = False
        subscribe = getattr(plugin, "subscribe_health", None)
        if callable(subscribe):
            try:
                subscribe(self._on_health)
                subscribed = True
            except Exception as exc:
                subscribe_error = str(exc)
        can_poll = callable(getattr(plugin, "health", None))
        if not can_poll and not subscribed:
            reason = subscribe_error or "telemetry plugin has no health()"
            self._note = f"mavsdk health not attached: {reason}"
            self._plugin = None
            return
        self._note = "reading MAVSDK health on the open session"
        if subscribe_error:
            self._note += f" (subscribe_health failed: {subscribe_error})"
        self.poll()

    def _on_health(self, health: object, _user_data: object = None) -> None:
        parsed = arm_readiness_from_health(health)
        if parsed is None:
            return
        with self._lock:
            self._latest = parsed

    def poll(self) -> ArmReadiness:
        plugin = self._plugin
        if plugin is not None:
            getter = getattr(plugin, "health", None)
            if callable(getter):
                try:
                    parsed = arm_readiness_from_health(getter())
                except Exception:
                    parsed = None
                if parsed is not None:
                    with self._lock:
                        self._latest = parsed
        with self._lock:
            return self._latest

    def close(self) -> None:
        plugin = self._plugin
        self._plugin = None
        destroy = getattr(plugin, "destroy", None)
        if callable(destroy):
            try:
                destroy()
            except Exception:
                return


class OhhoVehicle:
    """``Robot`` flight methods on one loopback MAVLink transport.

    ``frame`` is the Ohho ``Frame`` passed to every position setpoint. The
    opener passes ``Frame.LOCAL_NED``.
    """

    def __init__(
        self,
        robot: _FlightRobot,
        frame: object,
        *,
        health: ArmHealthWatch | None = None,
    ) -> None:
        self._robot = robot
        self._frame = frame
        self._health = ArmHealthWatch() if health is None else health
        if health is None:
            self._health.attach(robot)

    @property
    def arm_health_note(self) -> str:
        return self._health.note

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
        custom_map = _string_mapping(custom) or {}
        drone_map = _string_mapping(drone) or {}
        readiness = arm_readiness_from_telemetry(custom_map, drone_map)
        if readiness is None:
            readiness = self._health.poll()
        return VehicleSample(
            x=float(pose["x"]),
            y=float(pose["y"]),
            z=float(pose["z"]),
            armed=bool(drone.get("armed")),
            landed_state=str(drone.get("landed_state", "unknown")),
            flight_mode=str(drone.get("flight_mode", "unknown")),
            failsafe=tuple(str(flag) for flag in flags),
            link_ok=bool(drone.get("link_ok")),
            readiness=readiness,
        )

    def arm(self) -> None:
        self._robot.arm_flight()

    def takeoff(self, alt_m: float) -> None:
        self._robot.takeoff(alt_m)

    def goto(self, x: float, y: float, z: float, yaw: float) -> None:
        self._robot.offboard_position(x, y, z, yaw, self._frame)

    def land(self) -> None:
        self._robot.land()

    def disarm(self) -> None:
        self._robot.disarm_flight()

    def close(self) -> None:
        self._health.close()
        self._robot.disconnect()


def open_sitl() -> OhhoVehicle:
    """Connect ``Robot`` to loopback SITL. Does not wrap ``SafetyGate``.

    ``Robot.connect`` would wrap the mavlink transport in the gate. The gate
    still classifies protocol ``mavlink`` as hardware, and a ``CI`` environment
    refuses arm and takeoff. The transport flag on this URI is the simulation
    target: ``requires_hardware_arming`` is false. CI leaves
    ``OHHO_ARM_HARDWARE`` unset. There is no TTY phrase.
    """
    if "OHHO_ARM_HARDWARE" in os.environ:
        raise Refused(
            "refusing because OHHO_ARM_HARDWARE is set; CI leaves that variable unset"
        )
    from ohho.adapters.mavlink_drone import MavlinkDroneTransport
    from ohho.aerial import Frame
    from ohho.registry import get_spec
    from ohho.robot import Robot
    from ohho.runtime import get_runtime

    spec = get_spec(ROBOT_ID)
    transport = MavlinkDroneTransport(
        spec, SITL_URI, timeout_s=DISCOVERY_TIMEOUT_S
    )
    if transport.mavsdk_url != MAVSDK_URL:
        raise Refused(f"refusing unexpected MAVSDK URL {transport.mavsdk_url}")
    if not transport.simulation_target or transport.requires_hardware_arming:
        raise Refused("refusing a mavlink target that is not loopback SITL")
    runtime = get_runtime("native")
    robot = Robot(spec, transport, runtime)
    transport.connect()
    try:
        runtime.start()
    except Exception:
        transport.disconnect()
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
    vehicle: OhhoVehicle | None = None
    try:
        vehicle = open_sitl()
        report = fly_square(vehicle)
        elapsed = time.perf_counter() - started
        _write_result(args.result, report.as_dict(elapsed_s=elapsed))
        print(f"elapsed_s={elapsed:.3f}", flush=True)
        return 0
    except Refused as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except Exception as exc:
        elapsed = time.perf_counter() - started
        try:
            _write_result(args.result, _failure_payload(exc, elapsed))
        except Exception as write_exc:
            print(f"could not write {args.result}: {write_exc}", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        print(f"elapsed_s={elapsed:.3f}", flush=True)
        return 1
    finally:
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception as exc:
                print(f"disconnect failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
