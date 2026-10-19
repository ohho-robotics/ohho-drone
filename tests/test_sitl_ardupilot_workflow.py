"""The ArduPilot workflow matches the square script and stays off hardware."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import ardupilot_sitl_square as square  # noqa: E402

WORKFLOW = ROOT / ".github" / "workflows" / "sitl-ardupilot.yml"


class WorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.text = WORKFLOW.read_text(encoding="utf-8")

    def test_workflow_file_is_the_ardupilot_one(self) -> None:
        self.assertTrue(WORKFLOW.is_file())
        self.assertIn("name: ArduPilot SITL square", self.text)

    def test_triggers_and_artifacts(self) -> None:
        self.assertIn("workflow_dispatch:", self.text)
        self.assertIn("pull_request:", self.text)
        self.assertIn("paths:", self.text)
        self.assertIn("scripts/ardupilot_sitl_square.py", self.text)
        self.assertIn("docs/ardupilot-vs-px4.md", self.text)
        self.assertIn("actions/upload-artifact@v4", self.text)
        self.assertIn("if: always()", self.text)
        self.assertIn("timeout-minutes: 90", self.text)
        self.assertIn("tail -n 200", self.text)

    def test_pinned_copter_and_sim_vehicle_flags(self) -> None:
        self.assertIn(square.COPTER_TAG, self.text)
        self.assertIn(square.COPTER_COMMIT, self.text)
        self.assertIn("sim_vehicle.py", self.text)
        for flag in square.SIM_VEHICLE_ARGS:
            self.assertIn(flag, self.text)
        self.assertIn(square.SITL_URI, self.text)
        self.assertIn(square.OHHO_PIP_URL, self.text)
        self.assertIn("--fly", self.text)

    def test_install_requirement_matches_ohho_os_metadata(self) -> None:
        expected = (
            "ohho-os[mavlink] @ git+https://github.com/ohho-robotics/ohho-os"
            "@ohh-305-mavlink-backend"
        )
        self.assertEqual(square.OHHO_PIP_URL, expected)
        self.assertIn(f'python -m pip install "{expected}"', self.text)
        self.assertNotIn("ohho[mavlink]", self.text)
        self.assertNotIn("ohho[mavlink]", square.OHHO_PIP_URL)

    def test_does_not_set_hardware_arming_or_pass_arm(self) -> None:
        self.assertNotIn("OHHO_ARM_HARDWARE=", self.text)
        self.assertNotIn("--arm", self.text)
        self.assertIn("OHHO_ARM_HARDWARE+x", self.text)

    def test_venv_packages_match_copter_4_7_2_prereqs(self) -> None:
        expected = (
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
        self.assertEqual(square.ARDUPILOT_VENV_PIP_PACKAGES, expected)
        self.assertNotIn("future", square.ARDUPILOT_VENV_PIP_PACKAGES)
        prereq = self.text.index("Install SITL prerequisites")
        venv = self.text.index("Install ArduPilot venv Python packages")
        build = self.text.index("Build ArduCopter SITL")
        self.assertLess(prereq, venv)
        self.assertLess(venv, build)
        step = self.text[venv:build]
        self.assertIn('venv_python="${HOME}/venv-ardupilot/bin/python3"', step)
        self.assertIn("ARDUPILOT_VENV_PIP_PACKAGES", step)
        self.assertIn('"${venv_python}" -m pip install', step)
        self.assertIn("import em", step)
        self.assertIn("import pexpect", step)
        self.assertIn("from pymavlink import mavextra", step)
        for skipped in ("wxpython", "matplotlib", "opencv-python", "pygame"):
            self.assertNotIn(skipped, step)


if __name__ == "__main__":
    unittest.main()
