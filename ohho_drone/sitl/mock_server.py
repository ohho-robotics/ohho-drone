"""Pure-Python Mock PX4 SITL server over UDP.

WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY.
Simulates a PX4 quadrotor autopilot responding to MAVLink commands
and streaming telemetry over UDP sockets without requiring external containers.
"""

from __future__ import annotations

import math
import select
import socket
import threading
import time

from ..mavlink.codec import MavlinkCodec
from ..mavlink.messages import (
    MAV_AUTOPILOT_PX4,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_RESULT_ACCEPTED,
    MAV_TYPE_QUADROTOR,
    AttitudeMessage,
    CommandAckMessage,
    CommandLongMessage,
    GlobalPositionIntMessage,
    HeartbeatMessage,
    LocalPositionNedMessage,
    SetPositionTargetLocalNedMessage,
    SysStatusMessage,
)
from ..schema import FlightMode


class MockPx4SitlServer:
    """Simulated PX4 autopilot node communicating via MAVLink over UDP."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 14540,
        sysid: int = 1,
        compid: int = 1,
        initial_lat: float = 47.397742,  # Zurich default PX4 home
        initial_lon: float = 8.545594,
        initial_alt_amsl: float = 488.0,
    ) -> None:
        self.host = host
        self.requested_port = port
        self.sysid = sysid
        self.compid = compid

        self.home_lat = initial_lat
        self.home_lon = initial_lon
        self.home_alt = initial_alt_amsl

        # Flight dynamics state
        self.armed = False
        self.flight_mode = FlightMode.DISARMED
        self.x = 0.0  # North, metres
        self.y = 0.0  # East, metres
        self.z = 0.0  # Down, metres (negative = altitude above ground)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.battery_remaining = 98  # %
        self.voltage_mv = 12400

        # Offboard setpoints
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_yaw_rate = 0.0
        self.target_takeoff_alt = 2.5

        # Socket and threads
        self._sock: socket.socket | None = None
        self._actual_port: int = port
        self._codec = MavlinkCodec(sysid=self.sysid, compid=self.compid)
        self._client_addr: tuple[str, int] | None = None
        self._stop_event = threading.Event()
        self._recv_thread: threading.Thread | None = None
        self._sim_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._start_time = time.time()

    @property
    def port(self) -> int:
        return self._actual_port

    @property
    def is_running(self) -> bool:
        return not self._stop_event.is_set() and self._sock is not None

    @property
    def client_address(self) -> tuple[str, int] | None:
        with self._lock:
            return self._client_addr

    @property
    def altitude(self) -> float:
        """Altitude above ground in metres."""
        with self._lock:
            return -self.z

    def start(self) -> None:
        """Bind socket and spawn receiver and physics ticker threads."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.requested_port))
        self._actual_port = self._sock.getsockname()[1]
        self._sock.setblocking(False)

        self._stop_event.clear()
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="MockPX4-Recv", daemon=True
        )
        self._sim_thread = threading.Thread(
            target=self._sim_loop, name="MockPX4-Sim", daemon=True
        )
        self._recv_thread.start()
        self._sim_thread.start()

    def stop(self) -> None:
        """Signal threads to stop and close socket."""
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._sim_thread is not None and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=1.0)

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._sock is None:
                break
            try:
                r, _, _ = select.select([self._sock], [], [], 0.1)
                if not r:
                    continue
                data, addr = self._sock.recvfrom(4096)
                with self._lock:
                    self._client_addr = addr

                packets = self._codec.feed(data)
                for pkt, msg in packets:
                    if msg is not None:
                        self._handle_message(msg, addr)
            except OSError:
                break
            except Exception:
                pass

    def _handle_message(self, msg: object, addr: tuple[str, int]) -> None:
        with self._lock:
            if isinstance(msg, HeartbeatMessage):
                # Update client knowledge; no response needed beyond regular stream
                pass

            elif isinstance(msg, CommandLongMessage):
                if msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
                    should_arm = bool(int(msg.param1) == 1)
                    self.armed = should_arm
                    if should_arm:
                        if self.flight_mode == FlightMode.DISARMED:
                            self.flight_mode = FlightMode.STABILIZED
                    else:
                        self.flight_mode = FlightMode.DISARMED
                        self.vx = self.vy = self.vz = 0.0
                    self._send_command_ack(
                        MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, addr
                    )

                elif msg.command == MAV_CMD_NAV_TAKEOFF:
                    if self.armed:
                        self.flight_mode = FlightMode.TAKEOFF
                        if msg.param7 > 0:
                            self.target_takeoff_alt = msg.param7
                        self._send_command_ack(
                            MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED, addr
                        )
                    else:
                        self._send_command_ack(
                            MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED, addr
                        )

                elif msg.command == MAV_CMD_NAV_LAND:
                    self.flight_mode = FlightMode.LAND
                    self._send_command_ack(MAV_CMD_NAV_LAND, MAV_RESULT_ACCEPTED, addr)

                elif msg.command == MAV_CMD_DO_SET_MODE:
                    # Switch custom mode
                    self._send_command_ack(
                        MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED, addr
                    )

            elif isinstance(msg, SetPositionTargetLocalNedMessage):
                if self.armed:
                    self.flight_mode = FlightMode.OFFBOARD
                    self.target_vx = msg.vx
                    self.target_vy = msg.vy
                    self.target_vz = msg.vz
                    self.target_yaw_rate = msg.yaw_rate

    def _send_command_ack(
        self, command: int, result: int, addr: tuple[str, int]
    ) -> None:
        ack = CommandAckMessage(command=command, result=result)
        raw = self._codec.encode(ack)
        if self._sock is not None:
            try:
                self._sock.sendto(raw, addr)
            except Exception:
                pass

    def _sim_loop(self) -> None:
        last_tick = time.time()
        last_hb = 0.0

        while not self._stop_event.is_set():
            time.sleep(0.05)  # 20 Hz loop
            now = time.time()
            dt = now - last_tick
            last_tick = now

            with self._lock:
                addr = self._client_addr
                if addr is None:
                    continue

                # Step dynamics
                if self.armed:
                    if self.flight_mode == FlightMode.TAKEOFF:
                        climb_rate = 1.0  # m/s
                        self.z -= climb_rate * dt
                        if -self.z >= self.target_takeoff_alt:
                            self.z = -self.target_takeoff_alt
                            self.flight_mode = FlightMode.HOLD
                        self.vz = -climb_rate if -self.z < self.target_takeoff_alt else 0.0

                    elif self.flight_mode == FlightMode.OFFBOARD:
                        self.vx = self.target_vx
                        self.vy = self.target_vy
                        self.vz = self.target_vz
                        self.x += self.vx * dt
                        self.y += self.vy * dt
                        self.z += self.vz * dt
                        self.yaw += self.target_yaw_rate * dt

                    elif self.flight_mode == FlightMode.LAND:
                        descent_rate = 0.8  # m/s
                        self.z += descent_rate * dt
                        self.vz = descent_rate
                        if self.z >= 0.0:
                            self.z = 0.0
                            self.vz = 0.0
                            self.vx = self.vy = 0.0
                            self.armed = False
                            self.flight_mode = FlightMode.DISARMED

                time_boot_ms = int((now - self._start_time) * 1000) & 0xFFFFFFFF

                # Heartbeat at 1 Hz
                if now - last_hb >= 1.0:
                    last_hb = now
                    base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    if self.armed:
                        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
                    hb = HeartbeatMessage(
                        custom_mode=0,
                        type=MAV_TYPE_QUADROTOR,
                        autopilot=MAV_AUTOPILOT_PX4,
                        base_mode=base_mode,
                        system_status=4,
                    )
                    self._send_to(hb, addr)

                # Periodic Telemetry (every ~100ms)
                loc_msg = LocalPositionNedMessage(
                    time_boot_ms=time_boot_ms,
                    x=self.x,
                    y=self.y,
                    z=self.z,
                    vx=self.vx,
                    vy=self.vy,
                    vz=self.vz,
                )
                self._send_to(loc_msg, addr)

                att_msg = AttitudeMessage(
                    time_boot_ms=time_boot_ms,
                    roll=self.roll,
                    pitch=self.pitch,
                    yaw=self.yaw,
                    rollspeed=0.0,
                    pitchspeed=0.0,
                    yawspeed=self.target_yaw_rate,
                )
                self._send_to(att_msg, addr)

                # Fused GNSS calculation from local x,y,z
                lat = int((self.home_lat + (self.x / 111320.0)) * 1e7)
                lon = int(
                    (
                        self.home_lon
                        + (self.y / (111320.0 * math.cos(math.radians(self.home_lat))))
                    )
                    * 1e7
                )
                alt_amsl = int((self.home_alt - self.z) * 1000)
                rel_alt = int(-self.z * 1000)
                gps_msg = GlobalPositionIntMessage(
                    time_boot_ms=time_boot_ms,
                    lat=lat,
                    lon=lon,
                    alt=alt_amsl,
                    relative_alt=rel_alt,
                    vx=int(self.vx * 100),
                    vy=int(self.vy * 100),
                    vz=int(self.vz * 100),
                    hdg=int((math.degrees(self.yaw) % 360) * 100),
                )
                self._send_to(gps_msg, addr)

                sys_msg = SysStatusMessage(
                    voltage_battery=self.voltage_mv,
                    battery_remaining=self.battery_remaining,
                )
                self._send_to(sys_msg, addr)

    def _send_to(self, msg: object, addr: tuple[str, int]) -> None:
        if self._sock is not None:
            raw = self._codec.encode(msg)
            try:
                self._sock.sendto(raw, addr)
            except Exception:
                pass


__all__ = ["MockPx4SitlServer"]
