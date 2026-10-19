#!/usr/bin/env python3
"""Fly arm, 2.5 m, a 4 m square, and land through Ohho on loopback PX4 SITL.

The vehicle commands go through ``Robot`` and the aerial ``SafetyGate``. This
script does not set ``OHHO_ARM_HARDWARE``, does not pass a force-disarm, and
does not accept a non-loopback ``px4+dds://`` URI. Under GitHub Actions it
connects only when ``OHHO_CI_SIM_ARMING`` and ``OHHO_CI_SITL`` are both
exactly ``1``. That opt-in does not bypass the gate.

Nothing in this module imports Ohho at import time, so the mission rules can
be tested without ROS or the simulator.

A failsafe flag that is already set at arm time is recorded and does not fail
the run by itself. Headless SITL has no RC, so ``rc_loss`` is that flag. A
flag that rises after takeoff and before the land command fails the run, as
does ``offboard_loss`` during the takeoff hold or the square. A flag that
rises only after the land command is recorded and does not fail. Landing
stops the offboard stream, so that loss is expected.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
import urllib.parse
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Optional, Sequence

# Plan section 3. These are the pass/fail numbers, not tuning guesses.
TAKEOFF_ALT_M = 2.5
ALT_TOLERANCE_M = 0.4
ALT_HOLD_S = 2.0
SQUARE_SIDE_M = 4.0
CORNER_TOLERANCE_M = 0.8
CRUISE_SPEED_MPS = 0.5
STREAM_HZ = 10.0
STATUS_TIMEOUT_S = 120.0
TAKEOFF_TIMEOUT_S = 90.0
CORNER_TIMEOUT_S = 30.0
LAND_TIMEOUT_S = 90.0
MISSION_DEADLINE_S = 600.0
# Same floor the Ohho disarm gate uses. A higher reading must not be disarmed.
DISARM_ALTITUDE_M = 0.25
# Same battery floor SafetyGate.takeoff uses.
TAKEOFF_MIN_BATTERY = 0.3
DEFAULT_URI = "px4+dds://127.0.0.1:8888"
YAW_NED = 0.0

CRITERIA: dict[str, float] = {
    "takeoff_alt_m": TAKEOFF_ALT_M,
    "alt_tolerance_m": ALT_TOLERANCE_M,
    "alt_hold_s": ALT_HOLD_S,
    "square_side_m": SQUARE_SIDE_M,
    "corner_tolerance_m": CORNER_TOLERANCE_M,
    "cruise_speed_mps": CRUISE_SPEED_MPS,
    "stream_hz": STREAM_HZ,
    "status_timeout_s": STATUS_TIMEOUT_S,
}
# Current preflight lines only. Cleared checks stop printing, so the tail of
# the console is the set that is still failing when wait-status times out.
PREFLIGHT_TAIL_LINES = 100
PREFLIGHT_TAIL_BYTES = 262144
_PREFLIGHT_FAIL = re.compile(r"Preflight Fail:\s*(.+)")
# SafetyGate._human_tty_reason. ci is CI without OHHO_CI_SIM_ARMING=1 and
# OHHO_CI_SITL=1; no_tty is a headless caller outside that sim-only opt-in.
HUMAN_TTY_REASONS = frozenset({"ci", "no_tty"})
# ohho-os maps FailsafeFlags.offboard_control_signal_lost to this name.
OFFBOARD_LOSS_FLAG = "offboard_loss"
# PX4 v1.17 commander failsafe modeFromAction, as ohho-os flight_mode strings
# (_NAV_NAMES in adapters/px4_dds.py). These are not normal phases of this
# script. auto_land is also what land() commands, so it is allowed only then.
_FAILSAFE_NAV_ALWAYS = frozenset({"auto_rtl", "descend", "termination"})
# Hold and the position/altitude/stabilized fallbacks. PX4 leaves multicopter
# takeoff by completing the mode into Hold, so auto_loiter during takeoff is
# not treated as a failsafe. It is one after the square has entered offboard.
_FAILSAFE_NAV_AFTER_OFFBOARD = frozenset(
    {"auto_loiter", "posctl", "altctl", "stabilized"}
)


class MissionError(RuntimeError):
    """The mission cannot continue. The process exits non-zero."""


@dataclass(frozen=True)
class DroneSample:
    """One aerial snapshot the mission loop can judge without Ohho types."""

    x: Optional[float]
    y: Optional[float]
    z: Optional[float]
    pose_frame: Optional[str]
    armed: Optional[bool]
    landed_state: str
    failsafe: tuple[str, ...]
    link_ok: bool
    battery: Optional[float]
    prearm: Optional[bool]
    # ohho-os drone["flight_mode"], the vehicle_status.nav_state name.
    flight_mode: Optional[str] = None
    # vehicle_status.failsafe when telemetry carries it. ohh-290-px4-dds does
    # not, so this stays None on the live path and nav_state is used instead.
    vehicle_failsafe: Optional[bool] = None


@dataclass
class MissionResult:
    """Pass/fail record written as JSON for the CI artifact."""

    passed: bool
    reasons: list[str]
    duration_s: float
    phases: list[str]
    corner_errors_m: list[Optional[float]]
    failsafes: list[str]
    landed_state: Optional[str]
    armed: Optional[bool]
    takeoff_held: bool
    preflight_failures: list[str]
    preexisting_failsafe_flags: list[str] = field(default_factory=list)
    post_land_flags: list[str] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["criteria"] = dict(CRITERIA)
        return payload


def square_corners(
    side_m: float = SQUARE_SIDE_M, z_ned: float = -TAKEOFF_ALT_M
) -> tuple[tuple[float, float, float], ...]:
    """Local NED corners of a closed square, including the return to the origin.

    x is north, y is east, z is down. Altitude is ``-z``.
    """
    side = float(side_m)
    down = float(z_ned)
    return (
        (0.0, 0.0, down),
        (side, 0.0, down),
        (side, side, down),
        (0.0, side, down),
        (0.0, 0.0, down),
    )


def scored_corners(
    side_m: float = SQUARE_SIDE_M, z_ned: float = -TAKEOFF_ALT_M
) -> tuple[tuple[float, float, float], ...]:
    """The four corners that must be reached, in flight order.

    The opening origin is the takeoff hold. It is scored only when the path
    returns there.
    """
    return square_corners(side_m, z_ned)[1:]


def cruise_points(
    side_m: float = SQUARE_SIDE_M,
    speed_mps: float = CRUISE_SPEED_MPS,
    hz: float = STREAM_HZ,
    z_ned: float = -TAKEOFF_ALT_M,
) -> tuple[tuple[float, float, float], ...]:
    """Position setpoints that walk the square at ``speed_mps``.

    Consecutive points are ``speed_mps / hz`` metres apart, and every scored
    corner is included exactly once.
    """
    if speed_mps <= 0.0 or hz <= 0.0 or side_m <= 0.0:
        raise ValueError("side, speed, and stream rate must be positive")
    step = speed_mps / hz
    corners = square_corners(side_m, z_ned)
    points: list[tuple[float, float, float]] = [corners[0]]
    for start, end in zip(corners, corners[1:]):
        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, math.ceil(dist / step - 1e-12))
        for index in range(1, count + 1):
            fraction = index / count
            points.append(
                (
                    start[0] + (end[0] - start[0]) * fraction,
                    start[1] + (end[1] - start[1]) * fraction,
                    z_ned,
                )
            )
    return tuple(points)


def horizontal_error_m(
    x: Optional[float], y: Optional[float], corner: tuple[float, float, float]
) -> Optional[float]:
    """Horizontal distance in metres, or None when the pose is incomplete."""
    if x is None or y is None:
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return math.hypot(x - corner[0], y - corner[1])


def altitude_error_m(z_ned: Optional[float], target_up_m: float = TAKEOFF_ALT_M) -> Optional[float]:
    """``|z_ned + target|`` for a NED down-positive altitude. None if z is missing."""
    if z_ned is None or not math.isfinite(z_ned):
        return None
    return abs(z_ned + target_up_m)


def positive_up_m(sample: DroneSample) -> Optional[float]:
    """Metres above the local origin. Only LOCAL_NED is converted.

    Missing z, a non-finite z, or any other frame stays None so a disarm is
    not authorised from an unknown height.
    """
    if sample.pose_frame != "LOCAL_NED" or sample.z is None:
        return None
    if not math.isfinite(sample.z):
        return None
    return -sample.z


def corner_accepted(sample: DroneSample, corner: tuple[float, float, float]) -> bool:
    """True when the vehicle is inside the corner box at the held altitude."""
    horizontal = horizontal_error_m(sample.x, sample.y, corner)
    vertical = altitude_error_m(sample.z)
    if horizontal is None or vertical is None:
        return False
    return horizontal <= CORNER_TOLERANCE_M and vertical < ALT_TOLERANCE_M


def altitude_held(z_ned: Optional[float], since: Optional[float], now: float) -> tuple[bool, Optional[float]]:
    """Return whether ``z_ned`` has stayed inside the band for ``ALT_HOLD_S``.

    The second value is the new start of the continuous hold, or None when the
    band was left.
    """
    error = altitude_error_m(z_ned)
    if error is None or error >= ALT_TOLERANCE_M:
        return False, None
    start = now if since is None else since
    return (now - start) >= ALT_HOLD_S, start


def flight_refusal(label: str, error: str) -> str:
    """How a SafetyGate flight refusal is written into the result.

    ``ci`` and ``no_tty`` come from ``_human_tty_reason`` inside
    ``_aerial_privileged``. Those reasons still stop the mission unless the
    sim-only opt-in applies (``OHHO_CI_SIM_ARMING=1`` and ``OHHO_CI_SITL=1``,
    plus the rest of the ohho-os conditions). The script does not pretend a
    human is present.
    """
    if error in HUMAN_TTY_REASONS:
        return (
            f"{label} refused by SafetyGate ({error}): ohho-os "
            "_human_tty_reason / _aerial_privileged blocks arm_flight and "
            "takeoff in CI unless OHHO_CI_SIM_ARMING=1 and OHHO_CI_SITL=1 "
            "opt this loopback SITL in, and when stdin is not a TTY outside "
            "that opt-in. This script does not allocate a fake TTY, clear CI, "
            "or set OHHO_ARM_HARDWARE."
        )
    return f"{label} refused by SafetyGate: {error}"


def preflight_failures(
    console_text: str, *, tail_lines: int = PREFLIGHT_TAIL_LINES
) -> list[str]:
    """Distinct ``Preflight Fail:`` texts in the last ``tail_lines`` lines.

    PX4 repeats a check that is still failing and stops once it passes. The
    tail is that current set. A tab that mavlink_log appends is stripped.
    """
    found: list[str] = []
    for line in console_text.splitlines()[-tail_lines:]:
        match = _PREFLIGHT_FAIL.search(line)
        if match is None:
            continue
        text = match.group(1).strip().rstrip("\t").strip()
        if text and text not in found:
            found.append(text)
    return found


def read_preflight_failures(path: str) -> tuple[list[str], str]:
    """Failures from a PX4 console file, plus a note when there are none.

    The note is empty when at least one current failure was found. Only the
    last 256 KiB is read, the same bound as the console tail artifact.
    """
    if not path:
        return [], "PX4 console log was not available"
    from pathlib import Path

    file = Path(path)
    if not file.is_file():
        return [], f"PX4 console log was not available ({path})"
    try:
        data = file.read_bytes()
    except OSError as exc:
        return [], f"PX4 console log could not be read ({exc})"
    text = data[-PREFLIGHT_TAIL_BYTES:].decode("utf-8", errors="replace")
    failures = preflight_failures(text)
    if not failures:
        return [], "PX4 console tail had no Preflight Fail lines"
    return failures, ""


def hardware_arming_block(requires_hardware_arming: bool) -> Optional[str]:
    """Why a loopback flight must stop, or None when the gate treats it as sim.

    The hardware path needs ``OHHO_ARM_HARDWARE``. This script never sets that
    variable, so a gate that still classifies the sim target as hardware cannot
    be armed from here.
    """
    if not requires_hardware_arming:
        return None
    return (
        "SafetyGate.requires_hardware_arming is true for this loopback "
        "px4+dds target. Refusing to set OHHO_ARM_HARDWARE."
    )


def refuse_hardware_arm_env(environ: dict[str, str]) -> None:
    """Stop if ``OHHO_ARM_HARDWARE`` is present at all.

    The sim-only CI exception requires the variable to be absent, including
    when the value is empty or not ``1``. This script never sets it.
    """
    if "OHHO_ARM_HARDWARE" in environ:
        raise MissionError(
            "OHHO_ARM_HARDWARE is set. This SITL script does not run with "
            "hardware arming enabled."
        )


def _github_actions(environ: dict[str, str]) -> bool:
    value = environ.get("GITHUB_ACTIONS", "")
    return isinstance(value, str) and value.strip().lower() == "true"


def require_ci_sim_opt_in(environ: dict[str, str]) -> None:
    """Refuse to connect on GitHub Actions without the sim-only opt-in.

    ``OHHO_CI_SIM_ARMING`` and ``OHHO_CI_SITL`` must both be exactly ``1``.
    This does not arm, does not set ``OHHO_ARM_HARDWARE``, and does not
    bypass ``SafetyGate``. Callers outside GitHub Actions are unchanged.
    """
    if not _github_actions(environ):
        return
    if environ.get("OHHO_CI_SIM_ARMING") == "1" and environ.get("OHHO_CI_SITL") == "1":
        return
    raise MissionError(
        "refusing to connect under GitHub Actions without "
        "OHHO_CI_SIM_ARMING=1 and OHHO_CI_SITL=1. The aerial SafetyGate "
        "stays enforced. This script does not allocate a fake TTY, clear CI, "
        "or set OHHO_ARM_HARDWARE."
    )


def _loopback_host(host: str) -> bool:
    name = host.strip().lower()
    if name in {"localhost", "::1"}:
        return True
    if name.startswith("[") and name.endswith("]"):
        name = name[1:-1]
    parts = name.split(".")
    if len(parts) != 4:
        return False
    numbers: list[int] = []
    for part in parts:
        if not part.isdigit():
            return False
        number = int(part)
        if number > 255:
            return False
        numbers.append(number)
    return numbers[0] == 127


def check_sitl_uri(uri: str) -> None:
    """Accept only a loopback ``px4+dds://`` URI with no credentials."""
    try:
        parsed = urllib.parse.urlparse(uri)
    except ValueError as exc:
        raise MissionError(f"invalid px4+dds URI: {exc}") from exc
    if parsed.scheme != "px4+dds":
        raise MissionError(
            f"URI scheme {parsed.scheme!r} is not px4+dds. This script only "
            "flies a loopback PX4 SITL."
        )
    if parsed.username or parsed.password:
        raise MissionError("px4+dds URI does not take credentials")
    host = parsed.hostname
    if not host or not _loopback_host(host):
        raise MissionError(
            "refusing a non-loopback px4+dds host. This script does not fly hardware."
        )
    try:
        port = parsed.port
    except ValueError as exc:
        raise MissionError(f"invalid px4+dds port: {exc}") from exc
    if port is not None and not 1 <= port <= 65535:
        raise MissionError("px4+dds port must be in 1..65535")


def _same_point(
    left: tuple[float, float, float], right: tuple[float, float, float]
) -> bool:
    return all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(left, right))


def _blank_result(reasons: Sequence[str], duration_s: float = 0.0) -> MissionResult:
    return MissionResult(
        passed=False,
        reasons=list(reasons),
        duration_s=duration_s,
        phases=[],
        corner_errors_m=[],
        failsafes=[],
        landed_state=None,
        armed=None,
        takeoff_held=False,
        preflight_failures=[],
    )


class _Mission:
    """One flight. ``sleep`` must advance ``clock`` or the loop stops."""

    def __init__(
        self,
        vehicle: Any,
        clock: Callable[[], float],
        sleep: Callable[[float], None],
        log: Callable[[str], None],
        deadline_s: float,
        preflight_report: Optional[Callable[[], tuple[list[str], str]]] = None,
    ) -> None:
        self.vehicle = vehicle
        self.clock = clock
        self.sleep = sleep
        self.log = log
        self.preflight_report = preflight_report
        self.started = clock()
        self.deadline_s = deadline_s
        self.phases: list[str] = []
        self.reasons: list[str] = []
        self.failsafes: list[str] = []
        self.preexisting_failsafe_flags: list[str] = []
        self.post_land_flags: list[str] = []
        self._monitor_failsafe = False
        self._have_failsafe_baseline = False
        self._flag_window = False
        self._offboard_phase = False
        self._square_phase = False
        self._saw_offboard_in_square = False
        self._land_commanded = False
        self._prev_flags: set[str] = set()
        self._vehicle_failsafe_tripped = False
        self._nav_failsafe_modes: list[str] = []
        self.sample = DroneSample(
            None, None, None, None, None, "unknown", (), False, None, None
        )
        self.hold_since: Optional[float] = None
        self.takeoff_held = False
        self.corner_errors: list[Optional[float]] = []
        self.preflight_failures: list[str] = []
        self._took_off = False
        self._asked_disarm = False

    def phase(self, name: str) -> None:
        self.phases.append(name)
        self.log(f"phase {name}")

    def _note_sample(self, sample: DroneSample) -> None:
        self.sample = sample
        if not self._monitor_failsafe:
            return
        flags = _flag_names(sample.failsafe)
        if not self._have_failsafe_baseline:
            self.preexisting_failsafe_flags = list(flags)
            self._prev_flags = set(flags)
            self._have_failsafe_baseline = True
            self._check_escalation(sample)
            return
        self._check_escalation(sample)
        self._check_flag_edges(flags)
        self._prev_flags = set(flags)

    def _check_escalation(self, sample: DroneSample) -> None:
        if sample.vehicle_failsafe is True:
            self._vehicle_failsafe_tripped = True
        self._check_nav(sample.flight_mode)

    def _check_nav(self, mode: Optional[str]) -> None:
        if not isinstance(mode, str):
            return
        name = mode.strip().lower()
        if not name or name == "unknown":
            return
        if name == "offboard" and self._square_phase:
            self._saw_offboard_in_square = True
        if name == "auto_land" and self._land_commanded:
            return
        if name in _FAILSAFE_NAV_ALWAYS or name == "auto_land":
            self._record_nav(name)
            return
        if (
            name in _FAILSAFE_NAV_AFTER_OFFBOARD
            and self._square_phase
            and self._saw_offboard_in_square
            and not self._land_commanded
        ):
            self._record_nav(name)

    def _record_nav(self, name: str) -> None:
        if name in self._nav_failsafe_modes:
            return
        self._nav_failsafe_modes.append(name)
        self.log(f"nav_state failsafe mode {name}")

    def _check_flag_edges(self, flags: tuple[str, ...]) -> None:
        if self._flag_window:
            for flag in flags:
                if flag not in self._prev_flags and flag not in self.failsafes:
                    self.failsafes.append(flag)
                    self.log(f"failsafe flag after takeoff: {flag}")
            if (
                self._offboard_phase
                and OFFBOARD_LOSS_FLAG in flags
                and OFFBOARD_LOSS_FLAG not in self.failsafes
            ):
                self.failsafes.append(OFFBOARD_LOSS_FLAG)
                self.log("offboard_loss during offboard flight")
            return
        if not self._land_commanded:
            return
        for flag in flags:
            if flag not in self._prev_flags and flag not in self.post_land_flags:
                self.post_land_flags.append(flag)
                self.log(f"failsafe flag after land command: {flag}")

    def _advance(self, period_s: float) -> None:
        before = self.clock()
        self.sleep(period_s)
        if self.clock() <= before:
            raise MissionError("sleep did not advance the mission clock")
        if self.clock() - self.started > self.deadline_s:
            self.reasons.append("mission deadline exceeded")

    def _timed_out(self, start: float, timeout_s: float) -> bool:
        return self.clock() - start >= timeout_s or bool(self.reasons)

    def read(self) -> DroneSample:
        sample = self.vehicle.snapshot()
        self._note_sample(sample)
        return sample

    def wait_until(
        self,
        ready: Callable[[], bool],
        timeout_s: float,
        *,
        period_s: Optional[float] = None,
        while_waiting: Optional[Callable[[], None]] = None,
    ) -> bool:
        period = (1.0 / STREAM_HZ) if period_s is None else period_s
        start = self.clock()
        while True:
            self.read()
            if while_waiting is not None:
                while_waiting()
            if self.reasons:
                return False
            if ready():
                return True
            if self._timed_out(start, timeout_s):
                return False
            self._advance(period)
            if self.reasons:
                return False

    def call(self, label: str, fn: Callable[[], None]) -> bool:
        try:
            fn()
        except MissionError as exc:
            text = str(exc)
            reason = text if text.startswith(label) else f"{label}: {text}"
            self.reasons.append(reason)
            self.log(reason)
            return False
        if self.vehicle.latched():
            reason = self.vehicle.latch_reason()
            self.reasons.append(f"safety gate latched during {label}: {reason}")
            return False
        return True

    def finish(self) -> MissionResult:
        if self._took_off and not self.vehicle.latched():
            # Close the in-flight window before land() so a flag the land
            # command itself causes is post-land, not an in-flight failure.
            self._flag_window = False
            self._offboard_phase = False
            self._square_phase = False
            self._land_commanded = True
            self.phase("land")
            self.call("land", self.vehicle.land)
            self._wait_landed()
        reasons = list(self.reasons)
        if self.failsafes:
            reasons.append(
                "failsafe after takeoff: " + ", ".join(self.failsafes)
            )
        if self._vehicle_failsafe_tripped:
            reasons.append("vehicle_status.failsafe became true after arming")
        if self._nav_failsafe_modes:
            reasons.append(
                "nav_state changed to a failsafe mode not commanded by the script: "
                + ", ".join(self._nav_failsafe_modes)
            )
        if self._took_off and not self.takeoff_held:
            if not any("takeoff altitude was not inside" in reason for reason in reasons):
                reasons.append(
                    f"takeoff altitude was not inside {ALT_TOLERANCE_M} m "
                    f"for {ALT_HOLD_S} s"
                )
        expected = len(scored_corners())
        if "square" in self.phases and len(self.corner_errors) < expected:
            reasons.append(
                f"reached {len(self.corner_errors)} of {expected} square corners"
            )
        for index, error in enumerate(self.corner_errors):
            if error is None or error > CORNER_TOLERANCE_M:
                reasons.append(
                    f"corner {index + 1} horizontal error {error} m "
                    f"exceeds {CORNER_TOLERANCE_M} m"
                )
        if self._took_off:
            if self.sample.landed_state != "on_ground":
                reasons.append(
                    f"landed_state is {self.sample.landed_state!r}, not on_ground"
                )
            if self.sample.armed is not False:
                reasons.append("motors were not disarmed at the end")
        # A gate latch or an early refusal is already in reasons. De-duplicate
        # while keeping order.
        unique: list[str] = []
        for reason in reasons:
            if reason not in unique:
                unique.append(reason)
        passed = self._took_off and not unique
        return MissionResult(
            passed=passed,
            reasons=unique,
            duration_s=self.clock() - self.started,
            phases=list(self.phases),
            corner_errors_m=list(self.corner_errors),
            failsafes=list(self.failsafes),
            landed_state=self.sample.landed_state,
            armed=self.sample.armed,
            takeoff_held=self.takeoff_held,
            preflight_failures=list(self.preflight_failures),
            preexisting_failsafe_flags=list(self.preexisting_failsafe_flags),
            post_land_flags=list(self.post_land_flags),
        )

    def _wait_landed(self) -> None:
        def ready() -> bool:
            sample = self.sample
            if sample.landed_state != "on_ground":
                return False
            if sample.armed is False:
                return True
            altitude = positive_up_m(sample)
            if altitude is None or altitude > DISARM_ALTITUDE_M:
                return False
            # One normal disarm. The gate refuses this unless the vehicle is
            # on the ground and low. There is no force flag.
            if not self._asked_disarm:
                self._asked_disarm = True
                self.call("disarm", self.vehicle.disarm_flight)
            return self.sample.armed is False

        if not self.wait_until(ready, LAND_TIMEOUT_S):
            if not any("landed_state" in reason or "disarm" in reason for reason in self.reasons):
                self.reasons.append("landing did not end disarmed on the ground")

    def run(self) -> MissionResult:
        try:
            self.phase("wait-status")
            if not self.wait_until(
                lambda: _ready_to_arm(self.sample), STATUS_TIMEOUT_S
            ):
                failures, note = self._collect_preflight()
                self.preflight_failures = failures
                reason = _not_ready_reason(self.sample, failures, note)
                self.reasons.append(reason)
                self.log(reason)
                return self.finish()

            self.phase("arm-gate")
            armed = False
            try:
                armed = bool(self.vehicle.arm_gate())
            except MissionError as exc:
                self.reasons.append(f"arm gate: {exc}")
                return self.finish()
            if not armed:
                detail = self.vehicle.last_error()
                extra = f" ({detail})" if detail else ""
                self.reasons.append(f"SafetyGate.arm() refused the gate{extra}")
                return self.finish()

            # The gate's arm_flight requires an offboard setpoint younger than
            # its command timeout. Publish once and arm before that window
            # closes. Do not sleep between the two calls.
            self.phase("arm-flight")
            if not self.call(
                "arm setpoint",
                lambda: self.vehicle.offboard_position(0.0, 0.0, 0.0, YAW_NED),
            ):
                return self.finish()
            if not self.call("arm_flight", self.vehicle.arm_flight):
                return self.finish()
            self._monitor_failsafe = True
            self.read()

            self.phase("takeoff")
            if not self.call("takeoff", lambda: self.vehicle.takeoff(TAKEOFF_ALT_M)):
                return self.finish()
            self._took_off = True
            # A clear-to-set flag counts from here until the land command.
            # offboard_loss also fails once the takeoff hold starts, and
            # through the square. The climb before the hold is only the
            # transition rule.
            self._flag_window = True

            def held() -> bool:
                ok, since = altitude_held(
                    self.sample.z, self.hold_since, self.clock()
                )
                self.hold_since = since
                if since is not None:
                    self._offboard_phase = True
                return ok

            # Position setpoints are not streamed here. A setpoint would switch
            # PX4 out of NAV_TAKEOFF and back into offboard.
            if not self.wait_until(held, TAKEOFF_TIMEOUT_S):
                self.reasons.append(
                    f"|z_ned + {TAKEOFF_ALT_M}| did not stay under "
                    f"{ALT_TOLERANCE_M} m for {ALT_HOLD_S} s"
                )
                return self.finish()
            self.takeoff_held = True

            self.phase("square")
            self._square_phase = True
            self._offboard_phase = True
            self._fly_square()
            return self.finish()
        except MissionError as exc:
            self.reasons.append(str(exc))
            return self.finish()

    def _fly_square(self) -> None:
        scored = scored_corners()
        next_corner = 0
        period = 1.0 / STREAM_HZ
        for point in cruise_points():
            if self.reasons:
                return
            def publish(
                target: tuple[float, float, float] = point,
            ) -> None:
                self.vehicle.offboard_position(
                    target[0], target[1], target[2], YAW_NED
                )

            if not self.call("offboard_position", publish):
                return
            self._advance(period)
            self.read()
            if next_corner < len(scored) and _same_point(point, scored[next_corner]):
                if not self._hold_corner(scored[next_corner], next_corner):
                    return
                next_corner += 1
        if next_corner != len(scored) and not self.reasons:
            self.reasons.append(
                f"path ended after {next_corner} of {len(scored)} corners"
            )

    def _hold_corner(self, corner: tuple[float, float, float], index: int) -> bool:
        def at_corner() -> bool:
            return corner_accepted(self.sample, corner)

        def republish() -> None:
            self.call(
                "corner setpoint",
                lambda: self.vehicle.offboard_position(
                    corner[0], corner[1], corner[2], YAW_NED
                ),
            )

        ok = self.wait_until(
            at_corner, CORNER_TIMEOUT_S, while_waiting=republish
        )
        error = horizontal_error_m(self.sample.x, self.sample.y, corner)
        self.corner_errors.append(error)
        self.log(
            f"corner {index + 1} horizontal_error_m={error} accepted={ok}"
        )
        if not ok and not self.reasons:
            self.reasons.append(
                f"corner {index + 1} was not inside {CORNER_TOLERANCE_M} m "
                f"horizontal and {ALT_TOLERANCE_M} m vertical"
            )
        return ok

    def _collect_preflight(self) -> tuple[list[str], str]:
        if self.preflight_report is None:
            return [], "PX4 console log was not available"
        try:
            failures, note = self.preflight_report()
        except OSError as exc:
            return [], f"PX4 console log could not be read ({exc})"
        if not isinstance(failures, list) or not all(
            isinstance(item, str) for item in failures
        ):
            return [], "PX4 console report was unusable"
        return list(failures), note if isinstance(note, str) else ""


def _ready_to_arm(sample: DroneSample) -> bool:
    if not sample.link_ok:
        return False
    if sample.battery is None or sample.battery < TAKEOFF_MIN_BATTERY:
        return False
    if sample.prearm is False:
        return False
    return True


def _not_ready_reason(
    sample: DroneSample, failures: Sequence[str], console_note: str
) -> str:
    if not sample.link_ok:
        reason = f"vehicle_status was not visible within {STATUS_TIMEOUT_S:.0f} s"
    elif sample.battery is None or sample.battery < TAKEOFF_MIN_BATTERY:
        reason = f"battery fraction {sample.battery} is below {TAKEOFF_MIN_BATTERY}"
    elif sample.prearm is False:
        reason = "PX4 pre-arm checks did not pass"
    else:
        reason = "simulator was not ready to arm"
    if failures:
        return reason + ": " + "; ".join(failures)
    if console_note:
        return f"{reason} ({console_note})"
    return reason


def run_mission(
    vehicle: Any,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str], None] = print,
    deadline_s: float = MISSION_DEADLINE_S,
    preflight_report: Optional[Callable[[], tuple[list[str], str]]] = None,
) -> MissionResult:
    """Fly the square on ``vehicle`` and return the pass/fail record.

    ``vehicle`` supplies ``snapshot``, ``arm_gate``, ``arm_flight``, ``takeoff``,
    ``offboard_position``, ``land``, ``disarm_flight``, ``latched``,
    ``latch_reason``, and ``last_error``. Flight methods raise
    :class:`MissionError` when the gate refuses them.
    """
    return _Mission(
        vehicle, clock, sleep, log, deadline_s, preflight_report
    ).run()


class OhhoVehicle:
    """``Robot`` plus the aerial SafetyGate, shaped for :func:`run_mission`."""

    def __init__(self, robot: Any) -> None:
        self.robot = robot

    def snapshot(self) -> DroneSample:
        tele = self.robot.telemetry()
        custom = tele.custom if isinstance(getattr(tele, "custom", None), dict) else {}
        drone = custom.get("drone")
        if not isinstance(drone, dict):
            drone = {}
        pose = drone.get("pose")
        if not isinstance(pose, dict):
            pose = {}
        flags = drone.get("failsafe")
        if not isinstance(flags, (list, tuple)):
            flags = ()
        frame = drone.get("pose_frame")
        return DroneSample(
            x=_finite(pose.get("x")),
            y=_finite(pose.get("y")),
            z=_finite(pose.get("z")),
            pose_frame=frame if isinstance(frame, str) else None,
            armed=_optional_bool(drone.get("armed")),
            landed_state=_landed(drone.get("landed_state")),
            failsafe=tuple(str(flag) for flag in flags),
            link_ok=drone.get("link_ok") is True,
            battery=_battery(tele, drone),
            prearm=_prearm(self.robot),
            flight_mode=_flight_mode_name(drone),
            vehicle_failsafe=_vehicle_failsafe_bit(drone),
        )

    def arm_gate(self) -> bool:
        return bool(self.robot.arm())

    def arm_flight(self) -> None:
        self._flight("arm_flight", lambda: self.robot.arm_flight())

    def takeoff(self, alt_m: float) -> None:
        self._flight("takeoff", lambda: self.robot.takeoff(alt_m))

    def offboard_position(self, x: float, y: float, z: float, yaw: float) -> None:
        from ohho.aerial import Frame

        self._flight(
            "offboard_position",
            lambda: self.robot.offboard_position(x, y, z, yaw, Frame.LOCAL_NED),
        )

    def land(self) -> None:
        self._flight("land", lambda: self.robot.land())

    def disarm_flight(self) -> None:
        self._flight("disarm_flight", lambda: self.robot.disarm_flight())

    def latched(self) -> bool:
        gate = self.robot.transport
        return bool(getattr(gate, "latched", False))

    def latch_reason(self) -> Optional[str]:
        gate = self.robot.transport
        reason = getattr(gate, "latch_reason", None)
        return None if reason is None else str(reason)

    def last_error(self) -> Optional[str]:
        gate = self.robot.transport
        error = getattr(gate, "last_error", None)
        return None if not error else str(error)

    def _flight(self, label: str, fn: Callable[[], None]) -> None:
        gate = self.robot.transport
        if hasattr(gate, "last_error"):
            gate.last_error = None
        fn()
        if bool(getattr(gate, "latched", False)):
            raise MissionError(
                f"{label} latched the safety gate ({getattr(gate, 'latch_reason', None)})"
            )
        error = getattr(gate, "last_error", None)
        if error:
            raise MissionError(flight_refusal(label, str(error)))


def _flag_names(flags: Sequence[object]) -> tuple[str, ...]:
    """Failsafe names in first-seen order, without empties or duplicates."""
    names: list[str] = []
    for flag in flags:
        text = str(flag).strip()
        if text and text not in names:
            names.append(text)
    return tuple(names)


def _flight_mode_name(drone: dict[str, Any]) -> Optional[str]:
    """``drone["flight_mode"]`` when ohho-os sent a nav_state name."""
    mode = drone.get("flight_mode")
    if isinstance(mode, str) and mode.strip():
        return mode.strip()
    return None


def _vehicle_failsafe_bit(drone: dict[str, Any]) -> Optional[bool]:
    """``vehicle_status.failsafe`` when telemetry carries the bool.

    ohho-os ``ohh-290-px4-dds`` does not put that bit in ``custom["drone"]``.
    Missing means unknown, not false. Nav-state escalation covers that path.
    """
    value = drone.get("vehicle_failsafe")
    if isinstance(value, bool):
        return value
    return None


def _finite(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number


def _optional_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    return None


def _landed(value: Any) -> str:
    if value in {"on_ground", "in_air", "takeoff", "landing", "unknown"}:
        return str(value)
    return "unknown"


def _battery(tele: Any, drone: dict[str, Any]) -> Optional[float]:
    samples: list[float] = []
    for value in (getattr(tele, "battery", None), drone.get("battery")):
        number = _finite(value)
        if number is not None:
            samples.append(number)
    if not samples:
        return None
    return min(samples)


def _prearm(robot: Any) -> Optional[bool]:
    """Read PX4's pre-arm flag when the transport has received vehicle_status.

    Telemetry does not carry the flag. Arming without it latches the gate, so
    the mission waits on the message the transport already stored.
    """
    gate = robot.transport
    inner = getattr(gate, "_inner", gate)
    status = getattr(inner, "_status_msg", None)
    if status is None or not hasattr(status, "pre_flight_checks_pass"):
        return None
    return getattr(status, "pre_flight_checks_pass", False) is True


def connect_and_run(
    uri: str,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str], None] = print,
    console_path: str = "",
) -> MissionResult:
    """Connect ``px4-sitl`` through the aerial gate and fly the square.

    This is the fly path. Under GitHub Actions it raises before connecting
    unless ``OHHO_CI_SIM_ARMING`` and ``OHHO_CI_SITL`` are both exactly ``1``.
    """
    require_ci_sim_opt_in(dict(os.environ))
    check_sitl_uri(uri)
    try:
        from ohho import capabilities as caps
        from ohho.adapters.px4_dds import is_sim_target, parse_px4_dds_uri
        from ohho.registry import RobotSpec, UnknownRobot, get_spec, register
        from ohho.robot import connect
        from ohho.safety import profile_for
    except ImportError as exc:
        raise MissionError(
            "ohho-os is not importable. Install "
            "git+https://github.com/ohho-robotics/ohho-os.git@ohh-290-px4-dds."
        ) from exc

    host, port, _namespace = parse_px4_dds_uri(uri)
    if not is_sim_target(host, port):
        raise MissionError(
            "ohho.adapters.px4_dds.is_sim_target refused this URI. "
            "Refusing to connect."
        )

    try:
        spec = get_spec("px4-sitl")
    except UnknownRobot:
        spec = register(
            RobotSpec(
                id="px4-sitl",
                name="PX4 SITL",
                category="aerial",
                capabilities=(caps.AERIAL,),
                adapter="px4+dds",
                dof_base=0,
                max_lin=2.0,
                max_ang=0.8,
            )
        )
    if not spec.has(caps.AERIAL):
        raise MissionError("px4-sitl spec has no aerial capability")
    planned = profile_for(spec.id, spec)
    if not getattr(planned, "aerial", False):
        raise MissionError(
            "the safety profile for px4-sitl is not aerial. "
            "The installed ohho-os does not include the OHH-288 drone profile."
        )

    robot = connect("px4-sitl", uri, runtime="ros2")
    try:
        if not getattr(robot, "gated", False):
            raise MissionError("px4+dds connected without a SafetyGate")
        gate = robot.transport
        if not getattr(getattr(gate, "profile", None), "aerial", False):
            raise MissionError("SafetyGate profile is not the aerial profile")
        block = hardware_arming_block(
            bool(getattr(gate, "requires_hardware_arming", False))
        )
        if block:
            raise MissionError(block)
        return run_mission(
            OhhoVehicle(robot),
            clock=clock,
            sleep=sleep,
            log=log,
            preflight_report=lambda: read_preflight_failures(console_path),
        )
    finally:
        try:
            robot.disconnect()
        except Exception as exc:
            log(f"disconnect failed: {exc}")


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fly a 4 m square at 2.5 m on loopback PX4 SITL through Ohho. "
            "There is no hardware-arm switch."
        )
    )
    parser.add_argument("--uri", default=DEFAULT_URI, help=f"default {DEFAULT_URI}")
    parser.add_argument(
        "--result",
        default="",
        help="write the JSON pass/fail record to this path",
    )
    parser.add_argument(
        "--px4-console",
        default=os.environ.get("OHHO_PX4_CONSOLE", "/artifacts/px4-console.log"),
        help="PX4 console log, read if wait-status times out",
    )
    return parser.parse_args(list(argv))


def _write_result(result: MissionResult, path: str) -> None:
    payload = result.to_json()
    text = json.dumps(payload, indent=2, sort_keys=True)
    print(text)
    if not path:
        return
    from pathlib import Path

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text + "\n", encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    result: MissionResult
    try:
        refuse_hardware_arm_env(dict(os.environ))
        check_sitl_uri(args.uri)
        require_ci_sim_opt_in(dict(os.environ))
        result = connect_and_run(args.uri, console_path=args.px4_console)
    except MissionError as exc:
        result = _blank_result([str(exc)])
        print(f"mission failed: {exc}", file=sys.stderr)
    _write_result(result, args.result)
    print(
        f"passed={str(result.passed).lower()} "
        f"duration_s={result.duration_s:.3f} "
        f"phases={','.join(result.phases) or '-'}"
    )
    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
