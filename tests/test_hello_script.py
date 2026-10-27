"""Test execution of scripts/px4_sitl_hello.py in mock mode."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import subprocess
import sys
import time
import unittest
from pathlib import Path

from ohho_drone.adapter import MavlinkDroneTransport
from ohho_drone.mavlink.messages import (
    MAV_CMD_DO_SET_MODE,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
)
from ohho_drone.schema import ConnectionState, DroneVelocity
from ohho_drone.sitl.mock_server import MockPx4SitlServer

_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "px4_sitl_hello.py"


def _load_hello():
    spec = importlib.util.spec_from_file_location("px4_sitl_hello", _SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestHelloScript(unittest.TestCase):
    def test_run_hello_mock_subprocess(self) -> None:
        """Verify px4_sitl_hello.py runs to completion with exit code 0."""
        res = subprocess.run(
            [sys.executable, str(_SCRIPT), "--mock", "--takeoff", "--timeout", "3.0"],
            capture_output=True,
            text=True,
            timeout=20,
        )
        self.assertEqual(
            res.returncode,
            0,
            f"Script failed with output:\n{res.stdout}\n{res.stderr}",
        )
        self.assertIn("Clean shutdown complete", res.stdout)
        self.assertIn("Connected!", res.stdout)
        self.assertIn("Sending forward velocity setpoint", res.stdout)
        self.assertIn("On ground and disarmed", res.stdout)
        self.assertNotIn("Requesting OFFBOARD", res.stdout)

    def test_live_forward_streams_offboard_then_stops(self) -> None:
        """Live forward movement streams setpoints and stops before return."""
        hello = _load_hello()
        server = MockPx4SitlServer(host="127.0.0.1", port=0)
        server.start()
        transport = MavlinkDroneTransport(host="127.0.0.1", port=server.port)
        try:
            status = transport.connect(timeout_s=3.0)
            self.assertEqual(status.state, ConnectionState.CONNECTED)
            self.assertTrue(transport.arm(timeout_s=2.0))
            calls: list[float] = []
            original = transport.send_drone_velocity

            def spy(vel: DroneVelocity) -> None:
                calls.append(vel.linear_x)
                original(vel)

            transport.send_drone_velocity = spy  # type: ignore[method-assign]
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                self.assertTrue(hello._fly_forward_offboard(transport))
            self.assertIn("OFFBOARD mode confirmed", stdout.getvalue())
            self.assertGreaterEqual(len(calls), 10)
            self.assertGreater(calls[-1], 0.0)
            stopped = len(calls)
            time.sleep(0.3)
            self.assertEqual(len(calls), stopped)
            mode_cmds = [
                c for c in server.received_commands() if c[0] == MAV_CMD_DO_SET_MODE
            ]
            self.assertTrue(
                any(int(c[2]) == PX4_CUSTOM_MAIN_MODE_OFFBOARD for c in mode_cmds)
            )
            self.assertGreater(server.x, 0.05)
            self.assertEqual(
                transport.read_drone_telemetry().flight_mode.value, "offboard"
            )
        finally:
            transport.disconnect()
            server.stop()


if __name__ == "__main__":
    unittest.main()
