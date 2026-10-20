"""Test execution of scripts/px4_sitl_hello.py in mock mode."""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path


class TestHelloScript(unittest.TestCase):
    def test_run_hello_mock_subprocess(self) -> None:
        """Verify px4_sitl_hello.py runs to completion with exit code 0."""
        script_path = (
            Path(__file__).resolve().parent.parent / "scripts" / "px4_sitl_hello.py"
        )
        res = subprocess.run(
            [sys.executable, str(script_path), "--mock", "--timeout", "3.0"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(res.returncode, 0, f"Script failed with output:\n{res.stderr}")
        self.assertIn("Clean shutdown complete", res.stdout)
        self.assertIn("Connected!", res.stdout)


if __name__ == "__main__":
    unittest.main()
