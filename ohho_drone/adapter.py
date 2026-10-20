"""OhhO OS MAVLink adapter prototype.

==============================================================================
WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY
DO NOT USE FOR REAL AIRCRAFT FLIGHT OR PHYSICAL HARDWARE CONTROL.
==============================================================================

Implements the OhhO OS Transport port for MAVLink-enabled flight controllers
(such as PX4 running in SITL).

Emergency stop lands or holds. It does not kill motors in the air.
Connections are refused unless the target host is a local simulation endpoint.
"""

from __future__ import annotations

import abc
import ipaddress
import select
import socket
import threading
import time
from collections.abc import Callable

from .mavlink.codec import MavlinkCodec
from .mavlink.messages import (
    MAV_AUTOPILOT_PX4,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    MAV_FORCE_DISARM_MAGIC,
    MAV_FRAME_BODY_NED,
    MAV_LANDED_STATE_IN_AIR,
    MAV_LANDED_STATE_LANDING,
    MAV_LANDED_STATE_ON_GROUND,
    MAV_LANDED_STATE_TAKEOFF,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_RESULT_ACCEPTED,
    MAV_TYPE_QUADROTOR,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_MODE_AUTO_LOITER,
    AttitudeMessage,
    CommandAckMessage,
    CommandLongMessage,
    ExtendedSysStateMessage,
    GlobalPositionIntMessage,
    HeartbeatMessage,
    LocalPositionNedMessage,
    SetPositionTargetLocalNedMessage,
    SysStatusMessage,
)
from .schema import (
    ConnectionState,
    DronePose,
    DroneTelemetry,
    DroneVelocity,
    FlightMode,
    GlobalCoordinates,
    Odometry,
    Telemetry,
    TransportStatus,
    Velocity,
    clamp,
)

Unsubscribe = Callable[[], None]

# Altitude above ground, in metres, treated as airborne when landed_state
# has not confirmed ON_GROUND. Used only to avoid disarming; it never
# authorizes a disarm by itself.
_AIRBORNE_ALTITUDE_M = 0.25

_LOOPBACK_NAMES = frozenset({"localhost"})
_DOCKER_LOCAL_NAMES = frozenset(
    {
        "host.docker.internal",
        "gateway.docker.internal",
        "host.containers.internal",
    }
)
_AIRBORNE_LANDED_STATES = frozenset(
    {
        MAV_LANDED_STATE_IN_AIR,
        MAV_LANDED_STATE_TAKEOFF,
        MAV_LANDED_STATE_LANDING,
    }
)


def _wire_host(host: str) -> str:
    """Host string suitable for sendto, with surrounding brackets removed."""
    text = str(host).strip()
    if text.startswith("[") and text.endswith("]"):
        return text[1:-1]
    return text


def _normalize_host(host: str) -> str:
    text = _wire_host(host).lower()
    return text


def _parse_ip(host: str) -> ipaddress.IPv4Address | ipaddress.IPv6Address | None:
    try:
        return ipaddress.ip_address(_normalize_host(host))
    except ValueError:
        return None


def _is_loopback_host(host: str) -> bool:
    """True for 127.0.0.0/8, ::1, and the name localhost."""
    text = _normalize_host(host)
    if text in _LOOPBACK_NAMES:
        return True
    ip = _parse_ip(text)
    if ip is None:
        return False
    mapped = getattr(ip, "ipv4_mapped", None)
    if mapped is not None:
        return bool(mapped.is_loopback)
    return bool(ip.is_loopback)


def _is_docker_local_host(host: str) -> bool:
    """True for well-known local container DNS names, not routable LAN addresses."""
    text = _normalize_host(host)
    if text in _DOCKER_LOCAL_NAMES:
        return True
    return text.endswith(".docker.internal") or text.endswith(".containers.internal")


def simulation_target_allowed(host: str, *, sitl: bool = False) -> bool:
    """Return whether connect() may open a socket to host.

    Loopback is always allowed. Docker-local names are allowed only when
    the explicit SITL flag is set. Every other host is refused.
    """
    if _is_loopback_host(host):
        return True
    return bool(sitl and _is_docker_local_host(host))


def simulation_refusal_reason(host: str, *, sitl: bool = False) -> str | None:
    """Clear error string when host is outside the simulation allow-list."""
    if simulation_target_allowed(host, sitl=sitl):
        return None
    if sitl:
        return (
            f"Refusing connection to {host!r}: SITL mode is simulation-only and "
            "still requires a loopback host (127.0.0.0/8, ::1, localhost) or a "
            "docker-local host such as host.docker.internal. Other hosts are refused."
        )
    return (
        f"Refusing connection to {host!r}: this transport is simulation-only and "
        "only accepts loopback hosts (127.0.0.0/8, ::1, localhost). "
        "Pass sitl=True to also allow docker-local hosts. Other hosts are refused."
    )

# ── Optional OhhO Transport base import ──────────────────────────────────────
try:
    from ohho.transport import BaseTransport, Transport
except ImportError:
    # Standalone abstract Transport interface matching ohho.transport
    class Transport(abc.ABC):  # type: ignore[no-redef]
        protocol: str = "base"

        @abc.abstractmethod
        def connect(self) -> TransportStatus: ...

        @abc.abstractmethod
        def disconnect(self) -> None: ...

        @abc.abstractmethod
        def status(self) -> TransportStatus: ...

        @abc.abstractmethod
        def read(self) -> Telemetry: ...

        @abc.abstractmethod
        def send_velocity(self, vel: Velocity) -> None: ...

        @abc.abstractmethod
        def send_joint_command(self, name: str, position: float) -> None: ...

        @abc.abstractmethod
        def emergency_stop(self) -> None: ...

        @abc.abstractmethod
        def release_stop(self) -> None: ...

        @abc.abstractmethod
        def on_telemetry(self, cb: Callable[[Telemetry], None]) -> Unsubscribe: ...

        @abc.abstractmethod
        def on_status(self, cb: Callable[[TransportStatus], None]) -> Unsubscribe: ...

    class BaseTransport(Transport):  # type: ignore[no-redef]
        def __init__(self) -> None:
            self._tele_subs: list[Callable[[Telemetry], None]] = []
            self._status_subs: list[Callable[[TransportStatus], None]] = []

        def on_telemetry(self, cb: Callable[[Telemetry], None]) -> Unsubscribe:
            self._tele_subs.append(cb)

            def off() -> None:
                try:
                    self._tele_subs.remove(cb)
                except ValueError:
                    pass

            return off

        def on_status(self, cb: Callable[[TransportStatus], None]) -> Unsubscribe:
            self._status_subs.append(cb)

            def off() -> None:
                try:
                    self._status_subs.remove(cb)
                except ValueError:
                    pass

            return off

        def _emit_telemetry(self, t: Telemetry) -> None:
            for cb in list(self._tele_subs):
                try:
                    cb(t)
                except Exception:
                    pass

        def _emit_status(self, s: TransportStatus) -> None:
            for cb in list(self._status_subs):
                try:
                    cb(s)
                except Exception:
                    pass


class MavlinkDroneTransport(BaseTransport):
    """MAVLink adapter for drone flight controllers in OhhO OS.

    Simulation only. Supports PX4 SITL over UDP, odometry and battery telemetry,
    3D velocity setpoints, and arm / takeoff / land.

    ``connect()`` fails closed unless ``host`` is loopback (127.0.0.0/8, ::1,
    or localhost). With ``sitl=True``, docker-local names such as
    ``host.docker.internal`` are also allowed. Every other host is refused
    before a socket is opened.

    ``emergency_stop()`` does not kill motors in flight. An airborne vehicle
    is commanded to land (``MAV_CMD_NAV_LAND``). If landed state is unknown,
    the vehicle is commanded to hold (``MAV_CMD_DO_SET_MODE`` to AUTO.LOITER)
    and then to land. Motors are disarmed only when ``landed_state`` confirms
    ``ON_GROUND``. The force-disarm magic value 21196 is never sent.
    """

    protocol: str = "mavlink"

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 14540,
        sysid: int = 255,  # GCS / companion computer ID
        compid: int = 191,  # MAV_COMP_ID_ONBOARD_COMPUTER
        target_sysid: int = 1,
        target_compid: int = 1,
        sitl: bool = False,
    ) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.target_sysid = target_sysid
        self.target_compid = target_compid
        # Explicit SITL flag. Does not allow routable hosts; see connect().
        self._sitl = sitl

        self._state = ConnectionState.IDLE
        self._connected_since: float | None = None
        self._error_msg: str | None = None
        self._e_stop = False

        # Internal state & telemetry snapshot
        self._telemetry = DroneTelemetry()
        self._codec = MavlinkCodec(sysid=self.sysid, compid=self.compid)
        self._sock: socket.socket | None = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._recv_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

        # Stats
        self._msg_count = 0
        self._last_msg_time = 0.0
        self._last_heartbeat_recv = 0.0
        self._heartbeat_event = threading.Event()
        # None until EXTENDED_SYS_STATE arrives. Never inferred as ON_GROUND.
        self._landed_state: int | None = None

        # Command acknowledgment tracking
        self._pending_acks: dict[int, tuple[threading.Event, int]] = {}

    # ── Transport Interface ──────────────────────────────────────────────────

    @property
    def landed_state(self) -> int | None:
        """Latest MAV_LANDED_STATE, or None if the vehicle has not reported one."""
        with self._lock:
            return self._landed_state

    def connect(self, timeout_s: float = 3.0) -> TransportStatus:
        """Open a UDP socket to a simulation host and wait for a heartbeat.

        Fails closed, with ``TransportStatus.state`` ERROR and a clear
        ``error``, unless the host is loopback. With ``sitl=True``, a
        docker-local host is also accepted. No socket is opened on refusal.
        """
        with self._lock:
            if self._state == ConnectionState.CONNECTED:
                return self.status()

            refusal = simulation_refusal_reason(self.host, sitl=self._sitl)
            if refusal is not None:
                self._error_msg = refusal
                self._sock = None
                self._set_state(ConnectionState.ERROR)
                return self.status()

            self._set_state(ConnectionState.CONNECTING)

            try:
                self._sock = socket.socket(self._socket_family(), socket.SOCK_DGRAM)
                self._sock.setblocking(False)
            except Exception as e:
                self._error_msg = f"Socket creation failed: {e}"
                self._set_state(ConnectionState.ERROR)
                return self.status()

            self._stop_event.clear()
            self._heartbeat_event.clear()

            # Start background threads
            self._recv_thread = threading.Thread(
                target=self._recv_loop, name="Mavlink-Recv", daemon=True
            )
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop, name="Mavlink-Heartbeat", daemon=True
            )
            self._recv_thread.start()
            self._heartbeat_thread.start()

            # Send initial heartbeat so target knows we are here
            self._send_heartbeat()

        # Wait for vehicle heartbeat
        got_heartbeat = self._heartbeat_event.wait(timeout=timeout_s)
        with self._lock:
            if got_heartbeat:
                self._connected_since = time.time()
                self._error_msg = None
                self._set_state(ConnectionState.CONNECTED)
            else:
                self._error_msg = f"Timeout waiting for heartbeat from {self.host}:{self.port}"
                self._set_state(ConnectionState.ERROR)

            return self.status()

    def disconnect(self) -> None:
        """Disconnect and stop background worker threads."""
        with self._lock:
            if self._state == ConnectionState.DISCONNECTED:
                return

            self._stop_event.set()
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

            self._set_state(ConnectionState.DISCONNECTED)

        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

    def status(self) -> TransportStatus:
        """Return current status of the MAVLink transport link."""
        with self._lock:
            rate = 0.0
            if self._last_msg_time > 0:
                elapsed = time.time() - self._last_msg_time
                if elapsed < 2.0 and self._msg_count > 0:
                    rate = float(self._msg_count)

            latency = 0.0
            if self._last_heartbeat_recv > 0:
                latency = max(0.0, (time.time() - self._last_heartbeat_recv) * 1000.0)

            return TransportStatus(
                protocol=self.protocol,
                state=self._state,
                label="ohho-drone-mavlink (experimental/not-flight-ready)",
                latency_ms=round(latency, 2),
                msg_rate=rate,
                error=self._error_msg,
                connected_since=self._connected_since,
            )

    def read(self) -> Telemetry:
        """Return current telemetry converted to OhhO OS core Telemetry."""
        with self._lock:
            return self._telemetry.to_ohho_telemetry()

    def read_drone_telemetry(self) -> DroneTelemetry:
        """Return full drone telemetry snapshot."""
        with self._lock:
            return self._telemetry

    def send_velocity(self, vel: Velocity) -> None:
        """Send planar velocity mapped to local DroneVelocity."""
        drone_vel = DroneVelocity.from_ohho_velocity(vel, linear_z=0.0)
        self.send_drone_velocity(drone_vel)

    def send_drone_velocity(self, vel: DroneVelocity) -> None:
        """Send 3D velocity setpoint in local/body coordinate frame."""
        if self._e_stop or self._state != ConnectionState.CONNECTED:
            return

        # Mask: ignore position (bits 0,1,2), acceleration (bits 6,7,8), yaw (bit 10)
        # Type mask: 0b0000_0101_1100_0111 = 0x05C7
        setpoint = SetPositionTargetLocalNedMessage(
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            vx=vel.linear_x,
            vy=vel.linear_y,
            vz=-vel.linear_z,  # In NED, negative z is upwards
            yaw_rate=vel.yaw_rate,
            type_mask=0x05C7,
            target_system=self.target_sysid,
            target_component=self.target_compid,
            coordinate_frame=MAV_FRAME_BODY_NED,
        )
        self._send_message(setpoint)

    def send_joint_command(self, name: str, position: float) -> None:
        """Drones have no articulated arm joints; no-op prototype stub."""
        pass

    def emergency_stop(self) -> None:
        """Land or hold. Disarm only when the vehicle is confirmed on the ground.

        This is not a motor kill. In-air vehicles (EXTENDED_SYS_STATE
        landed_state of IN_AIR, TAKEOFF, or LANDING, or altitude that shows
        the vehicle is airborne) receive MAV_CMD_NAV_LAND. If landed state
        is unknown, the vehicle is commanded to HOLD / AUTO.LOITER via
        MAV_CMD_DO_SET_MODE and then to land. MAV_CMD_COMPONENT_ARM_DISARM
        with param1=0 is sent only when landed_state confirms ON_GROUND.
        The force-disarm magic value 21196 is never sent.

        Further velocity setpoints are blocked until release_stop().
        """
        with self._lock:
            self._e_stop = True
            action = self._estop_action_locked()

        if action == "disarm":
            self._send_message(self._ground_disarm_command())
        elif action == "land":
            self._send_message(self._land_command())
        else:
            self._send_message(self._hold_command())
            self._send_message(self._land_command())

    def release_stop(self) -> None:
        """Release emergency stop lock."""
        with self._lock:
            self._e_stop = False

    # ── Drone Specific Commands ──────────────────────────────────────────────

    def arm(self, timeout_s: float = 2.0) -> bool:
        """Arm the flight controller motors."""
        if self._e_stop or self._state != ConnectionState.CONNECTED:
            return False

        cmd = CommandLongMessage(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1 = arm
            target_system=self.target_sysid,
            target_component=self.target_compid,
        )
        return self._send_command_and_wait_ack(cmd, timeout_s=timeout_s)

    def disarm(self, timeout_s: float = 2.0) -> bool:
        """Disarm motors with a normal disarm (param2=0).

        This is the explicit disarm request, separate from emergency_stop().
        It never sends the force-disarm magic value 21196. Prefer
        emergency_stop() in flight: that path lands or holds instead of
        disarming.
        """
        if self._state != ConnectionState.CONNECTED:
            return False

        return self._send_command_and_wait_ack(
            self._ground_disarm_command(), timeout_s=timeout_s
        )

    def takeoff(self, altitude_m: float = 2.5, timeout_s: float = 2.0) -> bool:
        """Send takeoff command with desired climb altitude."""
        if self._e_stop or self._state != ConnectionState.CONNECTED:
            return False

        cmd = CommandLongMessage(
            command=MAV_CMD_NAV_TAKEOFF,
            param7=float(altitude_m),
            target_system=self.target_sysid,
            target_component=self.target_compid,
        )
        return self._send_command_and_wait_ack(cmd, timeout_s=timeout_s)

    def land(self, timeout_s: float = 2.0) -> bool:
        """Command vehicle to land at current location."""
        if self._state != ConnectionState.CONNECTED:
            return False

        cmd = CommandLongMessage(
            command=MAV_CMD_NAV_LAND,
            target_system=self.target_sysid,
            target_component=self.target_compid,
        )
        return self._send_command_and_wait_ack(cmd, timeout_s=timeout_s)

    # ── Internal Communication & Threads ─────────────────────────────────────

    def _set_state(self, new_state: ConnectionState) -> None:
        if self._state != new_state:
            self._state = new_state
            self._emit_status(self.status())

    def _socket_family(self) -> int:
        ip = _parse_ip(self.host)
        if isinstance(ip, ipaddress.IPv6Address) and ip.ipv4_mapped is None:
            return socket.AF_INET6
        return socket.AF_INET

    def _estop_action_locked(self) -> str:
        """Choose disarm, land, or hold-then-land. Caller holds ``_lock``."""
        if self._confirmed_on_ground_locked():
            return "disarm"
        if self._is_airborne_locked():
            return "land"
        return "hold_then_land"

    def _altitude_agl_m_locked(self) -> float | None:
        """Highest reported height above ground, or None if no position yet."""
        samples: list[float] = []
        if self._telemetry.global_pos is not None:
            samples.append(self._telemetry.global_pos.relative_alt)
        if self._telemetry.pose is not None:
            # LOCAL_POSITION_NED z is Down; negative z is above the origin.
            samples.append(-self._telemetry.pose.z)
        if not samples:
            return None
        return max(samples)

    def _is_airborne_locked(self) -> bool:
        if self._landed_state in _AIRBORNE_LANDED_STATES:
            return True
        altitude = self._altitude_agl_m_locked()
        return altitude is not None and altitude > _AIRBORNE_ALTITUDE_M

    def _confirmed_on_ground_locked(self) -> bool:
        """True only when landed_state is ON_GROUND and altitude agrees."""
        if self._landed_state != MAV_LANDED_STATE_ON_GROUND:
            return False
        altitude = self._altitude_agl_m_locked()
        if altitude is not None and altitude > _AIRBORNE_ALTITUDE_M:
            return False
        return True

    def _target_command(self, command: int, **params: float) -> CommandLongMessage:
        return CommandLongMessage(
            command=command,
            target_system=self.target_sysid,
            target_component=self.target_compid,
            **params,
        )

    def _ground_disarm_command(self) -> CommandLongMessage:
        """Normal disarm. param2 stays 0 so the force-disarm magic is not sent."""
        return self._target_command(
            MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=0.0,
        )

    def _land_command(self) -> CommandLongMessage:
        return self._target_command(MAV_CMD_NAV_LAND)

    def _hold_command(self) -> CommandLongMessage:
        """MAV_CMD_DO_SET_MODE to PX4 AUTO.LOITER (HOLD)."""
        return self._target_command(
            MAV_CMD_DO_SET_MODE,
            param1=float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            param2=float(PX4_MODE_AUTO_LOITER),
            param3=float(PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        )

    def _send_message(self, msg: object) -> None:
        if (
            isinstance(msg, CommandLongMessage)
            and msg.command == MAV_CMD_COMPONENT_ARM_DISARM
            and int(msg.param2) == MAV_FORCE_DISARM_MAGIC
        ):
            # Hard stop: never put the force-disarm magic on the wire.
            return
        sock = self._sock
        if sock is None:
            return
        try:
            raw = self._codec.encode(msg)
            sock.sendto(raw, (_wire_host(self.host), self.port))
        except Exception:
            pass

    def _send_heartbeat(self) -> None:
        hb = HeartbeatMessage(
            custom_mode=0,
            type=MAV_TYPE_QUADROTOR,
            autopilot=MAV_AUTOPILOT_PX4,
            base_mode=0,
            system_status=4,
        )
        self._send_message(hb)

    def _send_command_and_wait_ack(
        self, cmd: CommandLongMessage, timeout_s: float = 2.0
    ) -> bool:
        event = threading.Event()
        with self._lock:
            self._pending_acks[cmd.command] = (event, -1)

        self._send_message(cmd)
        got_ack = event.wait(timeout=timeout_s)

        with self._lock:
            if got_ack and cmd.command in self._pending_acks:
                _, result = self._pending_acks.pop(cmd.command)
                return result == MAV_RESULT_ACCEPTED
            self._pending_acks.pop(cmd.command, None)
            return False

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.is_set():
            self._send_heartbeat()
            self._stop_event.wait(1.0)

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                r, _, _ = select.select([sock], [], [], 0.1)
                if not r:
                    continue
                data, _ = sock.recvfrom(4096)
                packets = self._codec.feed(data)
                for pkt, msg in packets:
                    if msg is not None:
                        self._process_incoming(msg)
            except OSError:
                break
            except Exception:
                pass

    def _process_incoming(self, msg: object) -> None:
        now = time.time()
        with self._lock:
            self._msg_count += 1
            self._last_msg_time = now

            if isinstance(msg, HeartbeatMessage):
                self._last_heartbeat_recv = now
                self._telemetry.armed = msg.is_armed
                if msg.is_armed:
                    if self._telemetry.flight_mode == FlightMode.DISARMED:
                        self._telemetry.flight_mode = FlightMode.STABILIZED
                else:
                    self._telemetry.flight_mode = FlightMode.DISARMED

                self._heartbeat_event.set()

            elif isinstance(msg, ExtendedSysStateMessage):
                self._landed_state = int(msg.landed_state)
                self._telemetry.landed_state = self._landed_state

            elif isinstance(msg, CommandAckMessage):
                if msg.command in self._pending_acks:
                    event, _ = self._pending_acks[msg.command]
                    self._pending_acks[msg.command] = (event, msg.result)
                    event.set()

            elif isinstance(msg, LocalPositionNedMessage):
                # Convert NED to OhhO 2D Odometry (x=North, y=East, theta=yaw, vx, vy)
                theta = self._telemetry.pose.yaw if self._telemetry.pose else 0.0
                omega = self._telemetry.pose.yaw if self._telemetry.pose else 0.0
                self._telemetry.odom = Odometry(
                    x=msg.x,
                    y=msg.y,
                    theta=theta,
                    vx=msg.vx,
                    vy=msg.vy,
                    omega=omega,
                )
                self._telemetry.velocity = DroneVelocity(
                    linear_x=msg.vx,
                    linear_y=msg.vy,
                    linear_z=-msg.vz,  # Invert Down to Up
                    yaw_rate=omega,
                )
                current_pose = self._telemetry.pose or DronePose()
                self._telemetry.pose = DronePose(
                    x=msg.x,
                    y=msg.y,
                    z=msg.z,
                    roll=current_pose.roll,
                    pitch=current_pose.pitch,
                    yaw=current_pose.yaw,
                )

            elif isinstance(msg, AttitudeMessage):
                current_pose = self._telemetry.pose or DronePose()
                self._telemetry.pose = DronePose(
                    x=current_pose.x,
                    y=current_pose.y,
                    z=current_pose.z,
                    roll=msg.roll,
                    pitch=msg.pitch,
                    yaw=msg.yaw,
                )
                if self._telemetry.odom:
                    self._telemetry.odom.theta = msg.yaw
                    self._telemetry.odom.omega = msg.yawspeed

            elif isinstance(msg, GlobalPositionIntMessage):
                self._telemetry.global_pos = GlobalCoordinates(
                    lat=msg.lat_deg,
                    lon=msg.lon_deg,
                    alt_amsl=msg.alt_m,
                    relative_alt=msg.relative_alt_m,
                )

            elif isinstance(msg, SysStatusMessage):
                self._telemetry.battery = clamp(
                    msg.battery_remaining / 100.0, 0.0, 1.0
                )
                self._telemetry.voltage_v = msg.voltage_v
                self._telemetry.current_a = msg.current_a

            self._telemetry.timestamp = now

        # Emit telemetry snapshot to listeners
        self._emit_telemetry(self.read())


__all__ = [
    "MavlinkDroneTransport",
    "simulation_refusal_reason",
    "simulation_target_allowed",
]
