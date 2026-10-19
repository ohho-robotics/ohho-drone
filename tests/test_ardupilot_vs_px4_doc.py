"""Every parameter named in the diff is linked to ArduPilot or PX4 docs."""

from __future__ import annotations

import unittest
from pathlib import Path

DOC = Path(__file__).resolve().parents[1] / "docs" / "ardupilot-vs-px4.md"

AP = "https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html"
PX = "https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html"

REQUIRED_LINKS = (
    "https://ardupilot.org/copter/docs/ac2_guidedmode.html",
    "https://docs.px4.io/v1.17/en/flight_modes/offboard.html",
    "https://ardupilot.org/copter/docs/flight-modes.html",
    "https://ardupilot.org/copter/docs/rtl-mode.html",
    "https://ardupilot.org/copter/docs/arming_the_motors.html",
    "https://docs.px4.io/v1.17/en/advanced_config/prearm_arm_disarm",
    "https://docs.px4.io/v1.17/en/config/safety",
    "https://ardupilot.org/copter/docs/common-geofencing-landing-page.html",
    "https://ardupilot.org/copter/docs/common-ac2_simple_geofence.html",
    "https://ardupilot.org/copter/docs/radio-failsafe.html",
    "https://ardupilot.org/copter/docs/gcs-failsafe.html",
    "https://ardupilot.org/copter/docs/common-auxiliary-functions.html",
    "https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/libraries/AP_DDS/README.md",
    "https://ardupilot.org/dev/docs/ros2-interfaces.html",
    "https://github.com/ArduPilot/ardupilot_ros/blob/master/README.md",
    f"{AP}#fence-enable-fence-enable-disable",
    f"{AP}#fence-type-fence-type",
    f"{AP}#fence-action-fence-action",
    f"{AP}#fence-alt-max-fence-maximum-altitude",
    f"{AP}#fence-alt-max-tp-altitude-max-frame-type",
    f"{AP}#fence-radius-circular-fence-radius",
    f"{AP}#fence-margin-fence-margin",
    f"{AP}#fence-margin-xy-fence-horizontal-margin",
    f"{AP}#fence-alt-min-fence-minimum-altitude",
    f"{AP}#arming-skipchk-arm-checks-to-skip-bitmask",
    f"{AP}#arming-need-loc-require-vehicle-location",
    f"{AP}#arming-options-arming-options",
    f"{AP}#arming-rudder-arming-with-rudder-enable-disable",
    f"{AP}#rtl-alt-m-rtl-altitude",
    f"{AP}#rtl-alt-final-m-rtl-final-altitude",
    f"{AP}#rtl-climb-min-m-rtl-minimum-climb",
    f"{AP}#rtl-speed-ms-rtl-speed",
    f"{AP}#rtl-loit-time-rtl-loiter-time",
    f"{AP}#rtl-cone-slope-rtl-cone-slope",
    f"{AP}#rtl-alt-type-rtl-mode-altitude-type",
    f"{AP}#rtl-options-rtl-mode-options",
    f"{AP}#fs-thr-enable-throttle-failsafe-enable",
    f"{AP}#fs-thr-value-throttle-failsafe-value",
    f"{AP}#fs-gcs-enable-ground-station-failsafe-enable",
    f"{AP}#fs-gcs-timeout-gcs-failsafe-timeout",
    f"{AP}#fs-options-failsafe-options-bitmask",
    f"{AP}#fs-ekf-action-ekf-failsafe-action",
    f"{AP}#fs-ekf-thresh-ekf-failsafe-variance-threshold",
    f"{AP}#fs-ekf-filt-ekf-failsafe-filter-cutoff",
    f"{AP}#fs-dr-enable-deadreckon-failsafe-action",
    f"{AP}#fs-dr-timeout-deadreckon-failsafe-timeout",
    f"{AP}#fs-vibe-enable-vibration-failsafe-enable",
    f"{AP}#fs-crash-check-crash-check-enable",
    f"{AP}#batt-fs-low-act-low-battery-failsafe-action",
    f"{AP}#batt-fs-crt-act-critical-battery-failsafe-action",
    f"{AP}#disarm-delay-disarm-delay",
    f"{PX}#COM_OF_LOSS_T",
    f"{PX}#COM_OBL_RC_ACT",
    f"{PX}#COM_RC_LOSS_T",
    f"{PX}#NAV_RCL_ACT",
    f"{PX}#COM_DL_LOSS_T",
    f"{PX}#NAV_DLL_ACT",
    f"{PX}#COM_LOW_BAT_ACT",
    f"{PX}#GF_ACTION",
    f"{PX}#GF_MAX_HOR_DIST",
    f"{PX}#GF_MAX_VER_DIST",
    f"{PX}#RTL_RETURN_ALT",
    f"{PX}#RTL_DESCEND_ALT",
    f"{PX}#RTL_LAND_DELAY",
    f"{PX}#RTL_CONE_ANG",
    f"{PX}#RTL_TYPE",
    f"{PX}#RC_MAP_KILL_SW",
    f"{PX}#COM_KILL_DISARM",
)

REQUIRED_NAMES = (
    "GUIDED",
    "Offboard",
    "FENCE_ENABLE",
    "FENCE_TYPE",
    "FENCE_ACTION",
    "FENCE_ALT_MAX",
    "FENCE_RADIUS",
    "FENCE_MARGIN",
    "FENCE_MARGIN_XY",
    "ARMING_SKIPCHK",
    "FS_THR_ENABLE",
    "FS_GCS_ENABLE",
    "FS_OPTIONS",
    "FS_EKF_ACTION",
    "RTL_ALT_M",
    "COM_OF_LOSS_T",
    "COM_OBL_RC_ACT",
    "GF_MAX_HOR_DIST",
    "GF_ACTION",
    "AP_DDS",
    "/ap/experimental/takeoff",
)


class DocTests(unittest.TestCase):
    def setUp(self) -> None:
        self.text = DOC.read_text(encoding="utf-8")

    def test_links_and_names(self) -> None:
        for link in REQUIRED_LINKS:
            self.assertIn(link, self.text, link)
        for name in REQUIRED_NAMES:
            self.assertIn(name, self.text, name)

    def test_fence_radius_boundary_and_honest_runtime(self) -> None:
        self.assertIn("greater than 30", self.text)
        self.assertIn("30 to 10000", self.text)
        self.assertIn("elapsed_s", self.text)
        self.assertIn("not** flown", self.text)
        self.assertIn("Copter-4.7.2", self.text)
        self.assertIn("fb773567896014da007bca58358704d4a0d8516f", self.text)
        self.assertIn(
            "ohho-os[mavlink] @ git+https://github.com/ohho-robotics/ohho-os"
            "@ohh-305-mavlink-backend",
            self.text,
        )
        self.assertNotIn("ohho[mavlink]", self.text)


if __name__ == "__main__":
    unittest.main()
