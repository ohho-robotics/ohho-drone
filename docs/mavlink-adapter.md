# OhhO OS MAVLink Adapter Prototype Specification

> **WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY**  
> Do not use for real aircraft flight or physical hardware control.

---

## 1. Scope & Purpose

The **OhhO OS MAVLink Adapter** (`ohho_drone.adapter.MavlinkDroneTransport`) bridges aerial robot flight controllers running MAVLink with the unified OhhO OS `Transport` interface.

In OhhO OS, all robots (wheeled bases like OmniBot, quadrupeds, arms, drones) expose a uniform runtime surface:
- Lifecycle management: `connect()`, `disconnect()`, `status()`
- Telemetry: `read() -> Telemetry`, `on_telemetry(callback)`
- Control commands: `send_velocity()`, `send_joint_command()`, `emergency_stop()`, `release_stop()`

This adapter enables high-level OhhO autonomy and teleoperation pipelines to control simulated drones without embedding flight-controller-specific protocols into application code.

---

## 2. Coordinate Frames & Transformations

Aerial platforms and flight controllers conventionally operate in the **NED** (North-East-Down) frame, whereas standard robotics software (including ROS and OhhO planar bases) operates in an **ENU** (East-North-Up) or robot-centric forward-left-up frame.

The adapter performs bidirectional mapping between these frames:

```text
┌────────────────────────────┐              ┌────────────────────────────┐
│      OhhO OS Drone         │              │      MAVLink / PX4         │
│     (Body / Local)         │              │     (NED Frame)            │
├────────────────────────────┤              ├────────────────────────────┤
│ linear_x (Forward)         │ ───────────► │ vx (North / Body X)        │
│ linear_y (Right)           │ ───────────► │ vy (East / Body Y)         │
│ linear_z (Climb Rate, Up)  │ ───────────► │ vz = -linear_z (Down)      │
│ yaw_rate (Clockwise)       │ ───────────► │ yaw_rate                   │
└────────────────────────────┘              └────────────────────────────┘
```

When receiving `LOCAL_POSITION_NED`:
- `x` (North) and `y` (East) map directly to planar `Odometry.x` and `Odometry.y`.
- `z` (Down) is negated to compute altitude above takeoff position: `altitude = -z`.
- `vz` (Descent rate) is negated to yield upward climb velocity: `linear_z = -vz`.

---

## 3. MAVLink Message Set

The prototype implements a pure-Python codec supporting both MAVLink v1 and v2 wire frames:

### Inbound Messages (Autopilot to OhhO)

| MAVLink Message | Message ID | Mapped Telemetry Field | Description |
|---|---|---|---|
| `HEARTBEAT` | 0 | `armed`, `flight_mode` | Verifies connection liveness and autopilot state. |
| `SYS_STATUS` | 1 | `battery`, `voltage_v`, `current_a` | Battery remaining (0..1 fraction) and bus voltage. |
| `ATTITUDE` | 30 | `pose.roll`, `pitch`, `yaw` | Vehicle Euler orientation and angular rotation rates. |
| `LOCAL_POSITION_NED` | 32 | `odom`, `pose.x, y, z`, `velocity` | Local position and linear velocity estimates. |
| `GLOBAL_POSITION_INT` | 33 | `global_pos` | Fused GNSS coordinates (WGS84 latitude, longitude, AMSL). |
| `EXTENDED_SYS_STATE` | 245 | `landed_state` | Landed state used by emergency stop (on ground, in air, or unknown). |
| `COMMAND_ACK` | 77 | internal command handlers | Confirms acceptance or rejection of commands. |

### Outbound Messages (OhhO to Autopilot)

| MAVLink Message | Message ID | Source Method | Description |
|---|---|---|---|
| `HEARTBEAT` | 0 | background thread | Broadcast at 1 Hz from companion/GCS system ID. |
| `COMMAND_LONG` (400) | 76 | `arm()`, `disarm()`, on-ground `emergency_stop()` | Arms, or disarms with `param1 = 0` and `param2 = 0`. Never sends the force-disarm value 21196. Emergency stop uses this only when landed state is `ON_GROUND`. |
| `COMMAND_LONG` (22) | 76 | `takeoff(altitude_m)` | Commands vehicle takeoff to specified altitude. |
| `COMMAND_LONG` (21) | 76 | `land()`, in-air `emergency_stop()` | Initiates descent and touchdown at the current location. |
| `COMMAND_LONG` (176) | 76 | `emergency_stop()` when landed state is unknown | `MAV_CMD_DO_SET_MODE` to HOLD / AUTO.LOITER, followed by land. |
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | `send_velocity()`, `send_drone_velocity()` | Streams 3D velocity vectors with position/acceleration ignored. |

---

## 4. Offboard Control Protocol & Type Masking

When streaming velocity setpoints via `SET_POSITION_TARGET_LOCAL_NED`, the adapter sets the `type_mask` bitmask to instruct the flight controller to ignore position and acceleration setpoints, tracking velocity only:

```text
Bits 0..2  : Ignore Position (x, y, z)       -> 1 1 1
Bits 3..5  : Use Velocity (vx, vy, vz)       -> 0 0 0
Bits 6..8  : Ignore Acceleration (afx..afz)  -> 1 1 1
Bit 9      : Ignore Force                    -> 0
Bit 10     : Ignore Yaw Angle                -> 1
Bit 11     : Use Yaw Rate                    -> 0
Resulting mask: 0x05C7
```

This ensures predictable velocity-based guidance from high-level planners without fighting the flight controller's position hold loops.

---

## 5. Failsafe & Emergency Stop Handling

Safety is critical when interfacing with aerial vehicles, including simulated ones:

1. **Simulation-only transport:** `connect()` fails closed unless the host is loopback (`127.0.0.0/8`, `::1`, or `localhost`). An explicit `sitl=True` flag also allows docker-local names such as `host.docker.internal`. Every other host is refused before a socket is opened, with `TransportStatus.state = ERROR` and a clear error string.
2. **Emergency stop (`emergency_stop()`) lands or holds. It does not kill motors in the air.**
   - Sets an internal software lock `_e_stop = True`, which blocks later velocity commands until `release_stop()`.
   - If `EXTENDED_SYS_STATE.landed_state` is `IN_AIR`, `TAKEOFF`, or `LANDING`, or if altitude shows the vehicle is airborne, sends `MAV_CMD_NAV_LAND`.
   - If landed state is unknown, sends `MAV_CMD_DO_SET_MODE` to HOLD / `AUTO.LOITER`, then `MAV_CMD_NAV_LAND`.
   - Sends `MAV_CMD_COMPONENT_ARM_DISARM` with `param1 = 0` and `param2 = 0` only when `landed_state` confirms `ON_GROUND` and altitude does not show the vehicle is airborne.
   - Never sends the force-disarm magic value `21196`.

---

## 6. Python Usage Example

```python
from ohho_drone.adapter import MavlinkDroneTransport
from ohho_drone.schema import DroneVelocity

# Create adapter targeting SITL endpoint
# Send to PX4 SITL's offboard local port (14580), not the 14540 remote port.
transport = MavlinkDroneTransport(host="127.0.0.1", port=14580)

# Connect and wait for heartbeat
status = transport.connect(timeout_s=3.0)
print(f"Status: {status.state.value}")

# Subscribe to continuous telemetry stream.
# pose_6dof.z is NED Down, so altitude above the origin is -z.
def on_telemetry(tele):
    z = tele.custom.get("drone", {}).get("pose_6dof", {}).get("z")
    altitude = None if z is None else -z
    print(f"Altitude: {altitude}m")

unsub = transport.on_telemetry(on_telemetry)

# Arm and takeoff (in SITL simulation)
transport.arm()
transport.takeoff(altitude_m=2.5)

# Send 3D velocity command (forward 0.5 m/s, climbing 0.2 m/s)
transport.send_drone_velocity(DroneVelocity(linear_x=0.5, linear_z=0.2))

# Land and disconnect
transport.land()
transport.disconnect()
unsub()
```
