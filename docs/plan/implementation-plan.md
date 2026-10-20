# Drone implementation plan

Plan only. No flight code is in this repository, and this document does not authorise buying hardware, arming an aircraft, or setting `OHHO_ARM_HARDWARE`. Versions and prices are the ones recorded in [docs/research/drone-stacks.md](../research/drone-stacks.md) on 19 October 2026. If a fact is missing there, it is missing here too.

The form-factor repo (`ohho-drone`) stays the place for this research, the SITL workflow, and the hardware checklist. The robot API stays in **ohho-os**, which is the pattern already used for the Unitree Go2: `ohho-quadruped` is still a concept README, and the backend is `ohho/adapters/unitree.py` plus a `legged.pose` capability.

## 1. Who owns what

| Repo | Owns | Does not own |
|---|---|---|
| `ohho-os` | `aerial` capability, `Robot` methods, registry spec, `SafetyGate` profile, transport URIs, the four backends, `sim-serve` fields, episode schema, MCP tools, CLI | Gazebo worlds, airframe parameter files, a forked PX4 |
| `ohho-drone` | These docs, later a GitHub Actions workflow and a thin SITL script that shells out to the public PX4 image, the bring-up checklist | A second copy of the transport classes |

Draft PR #2 (`ohh-67-px4-sitl-mavlink`, OHH-67) is **superseded** as the milestone 1 base. Cherry-pick its schema, e-stop decision, loopback refusal, mock command log, and tests into ohho-os. Delete the hand-written codec rather than wrapping it. Do not merge PR #2 first.

## 2. One interface, four backends

### Capability

Add to `ohho/capabilities.py` (planned, not present on `origin/main` `280fedfa`):

```text
AERIAL = "aerial"
```

`is_known` must include it. A robot without `aerial` ignores every method below, the same way a robot without `legged.pose` ignores `stand()` on branch `ohh-118-legged-api` (`ohho/robot.py` `_send_legged_mode`).

`base.drive` stays planar. `Robot.drive(vx, vy, w)` does not climb.

### Robot methods (ohho-os `ohho/robot.py`)

All of these no-op, and return the robot, when `aerial` is absent. When a `SafetyGate` is wrapped around the transport they go through the gate, not around it.

| Method | Meaning |
|---|---|
| `arm_flight()` | Ask the **autopilot** to arm motors. Refused unless the gate is armed, pre-arm checks passed, and a setpoint stream is alive for offboard. Distinct from `Robot.arm()`, which only unlocks the gate. |
| `disarm_flight()` | Normal disarm. Refused when airborne. Never a force-disarm. |
| `takeoff(alt_m)` | Climb to `alt_m` above the local origin in the autopilot's takeoff mode, then hold. `alt_m` is positive up. |
| `land()` | Land at the current horizontal position. |
| `rtl()` | Autopilot return-to-launch. Ohho does not fly the return itself. |
| `hold()` | Autopilot loiter / hover. |
| `offboard_position(x, y, z, yaw, frame)` | Position setpoint. |
| `offboard_velocity(vx, vy, vz, yaw_rate, frame)` | Velocity setpoint. |
| `set_yaw(yaw, frame)` | Absolute yaw, same frame enum. |

`frame` is an enum, not a comment:

| Frame | Axes |
|---|---|
| `LOCAL_NED` | x north, y east, z down. Positive `vz` descends. This is PX4 `TrajectorySetpoint`. |
| `LOCAL_ENU` | x east, y north, z up. Convert at the backend before publish. ROS convention. |
| `BODY_FRD` | x forward, y right, z down. PX4 / MAVLink body frame (`MAV_FRAME_BODY_NED`). Velocity teleop uses this. |

There is no silent default that mixes ENU and NED. Callers pass the frame. The VR and Android adapters pass `BODY_FRD` and convert stick-up into negative body-z (climb) only at that edge.

### Telemetry

`ohho/schema.py` `Telemetry` stays the common type. Aerial fields live in `custom["drone"]`, which is what PR #2's `DroneTelemetry.to_ohho_telemetry` already does and what Go2 does for roll and pitch.

Required keys once any aerial backend has connected:

| Key | Unit |
|---|---|
| `armed` | bool, autopilot armed, not the gate |
| `flight_mode` | string from the backend's mode table |
| `landed_state` | `on_ground`, `in_air`, `takeoff`, `landing`, or `unknown`. Unknown is never treated as on the ground. |
| `pose` | `{x, y, z, roll, pitch, yaw}` in the frame named by `pose_frame` |
| `pose_frame` | `LOCAL_NED` or `LOCAL_ENU`, set by the backend, never inferred by the caller |
| `velocity` | `{vx, vy, vz, yaw_rate}` in `velocity_frame` |
| `battery` | fraction on `Telemetry.battery`, plus `voltage_v` and `current_a` |
| `gps_fix` | `none`, `2d`, `3d`, `dgps`, `rtk_float`, `rtk_fixed`. Crazyflie sends `none`. |
| `failsafe` | list of active flags (`offboard_loss`, `rc_loss`, `geofence`, `battery`, `datalink`) |
| `link_ok` | bool, plus `link_quality` 0..1 when the backend has one |
| `experimental` | true until a real flight is logged by a person |
| `flight_ready` | false in this plan. Nothing sets it true. |

`Telemetry.odom` may carry a planar shadow (x, y, yaw, vx, vy, yaw rate) so existing recorder and sim-serve code keeps working. The shadow is not the source of truth for altitude.

### Backends

| Backend id | Module (planned) | Speaks | Milestone |
|---|---|---|---|
| `drone-sim` | `ohho/adapters/drone_sim.py` | In-process kinematic stub behind `sim://` | M1 |
| `px4-dds` | `ohho/adapters/px4_dds.py` | `px4_msgs` on the rclpy node from `ohho/runtime/ros2.py` | M1 |
| `mavlink` | `ohho/adapters/mavlink_drone.py` | Native `mavsdk` 4.x, pymavlink only for gaps | M4, skeleton tested against SITL in M1 if the DDS job slips |
| `crazyflie` | `ohho/adapters/crazyflie.py` | cflib, or ROS 2 topics from Crazyswarm2 | M3 |

`ohho/adapters/__init__.py` `resolve_transport` gains:

```text
px4+dds://HOST:PORT?ns=NAME
mavlink+udp://HOST:PORT
mavlink+serial://DEVICE?baud=921600
crazyflie://radio://0/80/2M/E7E7E7E7E7
crazyflie+ros2://cf21
```

`sim://` with robot id `px4-sitl` or `crazyflie-sim` selects `drone-sim`. Do not reuse `dds://`; that scheme is Unitree CycloneDDS (`ohho/adapters/unitree.py`).

Registry specs to add, both `aerial`, neither `manipulation`:

- `px4-sitl` — default transport `sim://`, max horizontal 2 m/s, max climb 1 m/s, max yaw rate 0.8 rad/s. These are Ohho caps, not PX4's `MPC_XY_VEL_MAX`.
- `crazyflie-cf21` — same caps, GPS fix always `none`.
- `px4-x500` — same caps, `requires_hardware_arming` true for `px4+dds://` and `mavlink+serial://`. Used only when a human is present. Not created by CI.

### E-stop, reconciled with PR #2

Three layers. They are not synonyms.

| Layer | Who | In flight, what happens |
|---|---|---|
| Ohho `emergency_stop` | `SafetyGate` latches, disarms the gate, calls `inner.emergency_stop()` (`ohho/safety.py`) | **px4-dds and mavlink:** send land if `landed_state` is in air, takeoff, or landing. If landed state is unknown, send hold then land. Disarm the autopilot only after `landed_state == on_ground` and altitude ≤ 0.25 m. Never send force-disarm `21196`. Never send flight termination. This is PR #2's policy (`ohho_drone/adapter.py` `emergency_stop`). **crazyflie:** call the Crazyswarm2 emergency service, which cuts motors. A 30 g aircraft in a net is the exception, and the backend name is in the audit line. **drone-sim:** freeze integration and set landed after a simulated descent. Do not model a free-fall. |
| Autopilot failsafe | PX4 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`, `COM_RC_LOSS_T`, `GF_*`, battery. ArduPilot `FENCE_*`, `FS_*`. | Runs even if the Ohho process is dead. Ohho must not disable it to make a demo smoother. |
| Hardware kill | PX4 RC kill switch, QGroundControl flight-mode setup | Motor outputs stop. The vehicle falls. Pilot's hand. Not an Ohho function. Not exposed to MCP. |

`release_stop()` clears the Ohho latch only. It does not arm the autopilot and does not resume offboard. The existing hardware rule stays: a TTY and the release phrase (`ohho/safety.py` `RELEASE_PHRASE`).

On `px4-dds`, "send land" means a `VehicleCommand` for the land mode, not a zero velocity setpoint. A zero setpoint in offboard is "hover here", which is hold, not land. PR #2 got this distinction right and a naive ROS `Twist` zero would get it wrong.

### Process topology for `px4-dds`

```text
PX4 SITL or Pixhawk
  uxrce_dds_client  --udp 8888-->  MicroXRCEAgent
                                      |
                                      v
                               Ros2Runtime node
                               Px4DdsTransport
                               SafetyGate (hardware only)
                               Robot
```

SITL command the workflow will grow, not run from this branch:

```sh
docker run --rm -d --name ohho-px4 --network host \
  -e HEADLESS=1 -e PX4_SIM_MODEL=gz_x500 \
  px4io/px4-sitl-gazebo:v1.17.0
```

Agent (if the image does not already start one):

```sh
MicroXRCEAgent udp4 -p 8888
```

The ROS 2 image `px4io/px4-sitl-gazebo-ros2` already bundles Jazzy, agent 2.4.3, and matching `px4_msgs`. Prefer that image for CI so the agent and the firmware share a network namespace. Pin a digest in the workflow after the first successful pull. Do not pin `latest`.

QoS on every PX4 subscription and publisher is best-effort. Publishers that PX4 subscribes to use volatile. Subscriptions to PX4 publishers use transient-local. Depth 1 is enough. `Ros2Transport`'s depth-10 reliable QoS is the wrong default (`ohho/adapters/ros2.py`).

Message package: `px4_msgs` branch `release/1.17` checked out next to the node, or the copy baked into the pinned image. A startup check compares `OffboardControlMode` fields (`thrust_and_torque`, `direct_actuator`, no `actuator`) and refuses to arm on mismatch.

## 3. SITL CI

Workflow file, when written: `ohho-drone/.github/workflows/sitl-px4.yml`. It calls `python -m ohho.drone_sitl` (module planned in ohho-os) or a script `scripts/sitl_square.py` in this repo that imports that module. The script does not live in this commit.

Job shape:

1. `runs-on: ubuntu-latest`.
2. Restore a Docker layer cache keyed by `px4io/px4-sitl-gazebo-ros2@<digest>`.
3. Start the container headless, model `gz_x500`, `PX4_SIM_SPEED_FACTOR=1` for the first green run. Speed-up is a later change, because lockstep hides timing bugs that a real UART will not.
4. Wait until `/fmu/out/vehicle_status` is visible, timeout 120 s.
5. Stream `OffboardControlMode` at 10 Hz.
6. Arm, take off to **2.5 m** (inside the 2–3 m band), wait until `|z_ned + 2.5| < 0.4` m for 2 s.
7. Fly a square of **4 m** sides in local NED at 0.5 m/s, altitude held. Corners within **0.8 m** horizontal.
8. Land. Pass when `landed_state` is on ground, motors disarmed, and no failsafe flag was set after takeoff.
9. Always dump the last 200 lines of the PX4 console as the job artifact, pass or fail.

Pass/fail is that sequence, not "the container stayed up". A timeout, a failsafe flag, a missing disarm, or a corner error outside 0.8 m fails the job.

Runtime budget to **set and then measure**: 20 minutes including a cold image pull, 8 minutes with a warm cache. If three runs miss it, move Gazebo to a nightly workflow and run `px4io/px4-sitl` (SIH, about 100 MB) on pull requests. Those budgets are not measurements.

ArduPilot `sim_vehicle.py` is a second workflow, `sitl-ardupilot.yml`, in M4, same square, MAVLink backend, not DDS.

CI never sets `OHHO_ARM_HARDWARE`. The SITL URI is loopback. `requires_hardware_arming` is false for loopback DDS on a non-zero domain today (`ohho/safety.py`); `px4+dds://127.0.0.1:8888` must be classified the same way, by an explicit sim-target check, not by hoping the scheme string misses the hardware heuristic.

Props-off rule for humans does not apply to CI. CI also must not be copy-pasted onto a lab machine with a USB cable attached.

## 4. Teleop

### Sticks

Mode 2, matching `OhhO-VR/Assets/Scripts/Control/Drive/AerialDriveScheme.cs` and a mode-2 ELRS radio:

| Stick | Axis | Setpoint |
|---|---|---|
| Left Y | up positive | body climb, `BODY_FRD` vz negative |
| Left X | right positive | yaw rate. Sign: match the radio, and write it down in the test. The VR scheme currently does **not** negate left-X (`AerialDriveScheme.ComputeVelocity`). `BaseController` **does** negate right-X for ground yaw. Do not copy both. |
| Right Y | up positive | body forward vx |
| Right X | right positive | body right vy |

Dead zone 0.1, the VR constant. No turbo on an aircraft. The VR 2× grip turbo stays a ground-robot feature.

Caps inside `SafetyGate`, stricter than `DriveSpec.Quadrotor`'s catalog 12 m/s:

| Axis | Cap |
|---|---|
| horizontal speed | 1.5 m/s |
| climb / descent | 1.0 m/s |
| yaw rate | 0.8 rad/s |
| takeoff altitude, teleop | 3 m unless a per-flight ceiling is set lower |

`DriveSpec.cs` is not changed in M1. The cap that matters is the one in Python, because VR can be bypassed.

### Deadman, arm, takeoff, land

`ohho sim-serve` already zeros velocity when `deadman` is false or the socket is quiet for 300 ms (`docs/sim-serve.md` on ohho-os `origin/main`). Aerial teleop keeps that, and extends the message:

```json
{
  "velocity": {"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"z": 0.0}},
  "frame": "BODY_FRD",
  "deadman": true,
  "estop": false,
  "flight": {"request": "takeoff", "alt_m": 2.5, "confirm": "I am physically present, the area is clear, the remote is in my hand"}
}
```

`flight.request` is one of `takeoff`, `land`, `rtl`, `hold`, `arm`, `disarm`. On `backend=sim` the server may accept `takeoff` without the hardware phrase. On `backend=robot` with a non-sim URI it requires the same phrase `SafetyGate` already uses (`ARM_PHRASE` in `ohho/safety.py`), and it still requires `OHHO_ARM_HARDWARE=1` in the server's environment, set by the human in their own terminal. The WebSocket does not set that variable.

Deadman false does not land. It stops sending fresh setpoints, which trips PX4's offboard-loss failsafe (`COM_OBL_RC_ACT`). Recommended bench setting, written into the airframe notes when hardware exists: Hold if RC is present, Land if RC is gone. The numeric parameter values are applied by a person in QGroundControl, not by CI against a real board.

### Clients

| Client | Today | Planned change |
|---|---|---|
| VR `AerialDriveScheme` | ROSBridge `Twist` with `linear.z`, both-grips e-stop, no deadman (`DriveSchemeBase.cs`) | Add a grip deadman: no grip held means publish zero velocity and `deadman: false`. Both grips remain e-stop and must call land, not motor-kill, when the profile is an X500. A Crazyflie profile may cut motors. Takeoff and land are buttons with the confirmation phrase, not stick gestures. |
| Android `ControlsFragment.kt` | One stick, no vertical axis, `/cmd_vel/teleop` | A second control for climb, or an explicit "altitude hold, no climb" mode. Until that exists, the Android app must not be pointed at a drone URI. |
| Console `PilotConsole.tsx` | rosbridge `ws://localhost:9090` | A sim view that subscribes to `ohho sim-serve` on port 8765 and draws pose from `custom.drone`. Marketing copy in `console/lib/products.tsx` already mentions mode-2 sticks. That copy is not an implementation. |
| sim-serve | planar velocity | The JSON above. `backend` stays `sim`, `ros2`, or `robot`, as on branch `ohh-130-go2-teleop-robot-backend`. |

Link loss from the stick to the companion: the 300 ms sim-serve watchdog stops the setpoint stream. PX4 then runs offboard loss. The companion does not keep the last non-zero setpoint.

## 5. Recording

`ohho/data/recorder.py` on `origin/main` records a planar state and action and writes LeRobot v2.1 with `robot_type: "omnibot"`. Branch `ohh-129-go2-episode-logging` (`6c3dce8`) already generalises `robot_type` and optional videos. Aerial episodes should land on top of that branch's writer, not the older hard-coded names.

Planned feature names:

| Column | Contents |
|---|---|
| `observation.state` | pose xyz, roll, pitch, yaw, velocity xyz, yaw rate, battery fraction. 11 floats. Names stored in `info.json`, not sliced from the arm list. |
| `action` | the velocity setpoint that was sent: `vx, vy, vz, yaw_rate`, plus the frame as a constant in `info.json` (`BODY_FRD`). 4 floats. |
| `observation.images.front` | optional, only when a camera is attached. Absent in M1 sim. |
| `observation.lidar` | optional, not in M1. A point cloud does not fit the v2.1 image column; store a path or a downsampled vector only after the writer on `ohh-129` has a place for non-image blobs. Until then, do not pretend LiDAR is in the dataset. |

`Recorder._wrap_robot` today wraps `drive` and `move_joints`. It must also wrap `offboard_velocity` or the action column will be zeros during a flight. Sim teleop episodes are the M1 dataset. No hardware episodes until a person has flown and chosen to record.

Task string examples: `"sim takeoff hover land"`, `"sim square"`. Not `"inspect turbine"`.

## 6. MCP

Follow the OHH-278 contract even where commit `6acd7e1` has not implemented it yet. See the research note for the gap.

Planned tools, namespaced so they cannot be confused with `drive`:

| Tool | Sim | Hardware (`--allow-hardware` set by a human on the CLI) |
|---|---|---|
| `drone_connect` | `sim://` only by default | Allowed only when the process was started with `--allow-hardware`. Still does not arm. |
| `drone_telemetry` | yes | yes, read-only |
| `drone_velocity` | yes, clamped | yes, clamped, and only after a human has armed from a TTY. The tool does not arm. |
| `drone_hold` | yes | yes |
| `drone_land` | yes | yes, always, including when hardware was not allowed |
| `emergency_stop` | yes | yes, always. Maps as in section 2. |
| `drone_arm`, `drone_takeoff`, `drone_rtl`, `release_stop` | **not registered** | **not registered**. Denylist even if a config file names them. |

Every result JSON includes `"backend": "sim"` or `"backend": "hardware"`.

Clamps applied before the backend sees the setpoint: the table in section 4, plus an altitude ceiling (default 3 m sim, and a geofence ceiling on hardware). A setpoint that would exit the software fence is rejected and a hold is sent instead. Rejection is an error result, not a silent clip past the fence. Velocity magnitude may clip; position may not cross the fence.

`RobotSession.connect` on `6acd7e1` calls `robot.arm()` for a gated robot. The drone session must not. Gate arming stays a human at a TTY.

`--allow-hardware` does not exist on that commit. Adding it is ohho-os work, shared with the ground robots, not a drone-only flag with a second meaning.

## 7. Safety

| Control | Where it lives |
|---|---|
| Software fence | Ohho, before publish. Cylinder: radius and max altitude in the safety profile. Default sim radius 10 m, max alt 3 m. A setpoint outside is refused. |
| Autopilot fence | PX4 `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, `GF_ACTION=Hold` for the first flights, with margin for stopping distance. The PX4 safety guide says the action runs **after** the breach. ArduPilot `FENCE_ENABLE`, `FENCE_TYPE`, `FENCE_RADIUS` (> 30 m on the Copter page cited), `FENCE_ALT_MAX`. |
| RTL | Autopilot RTL. Ohho `rtl()` requests it. Home must be set and checked in QGroundControl before outdoor flight. |
| RC kill | Transmitter switch, PX4 kill. Always available. Not software. |
| Arming | Pre-arm on the autopilot (position, battery, fence, RC). Then `OHHO_ARM_HARDWARE=1`, TTY, typed phrase, then `arm_flight()`. Allowlist of aerial actions: `hold`, `land`, `rtl`, `offboard_velocity`, `offboard_position`. `arm_flight` and `takeoff` are not in the MCP allowlist. |
| Offboard loss | Stop streaming. PX4 leaves offboard via `COM_OBL_RC_ACT`. |
| Battery and datalink | Autopilot failsafes left at documented defaults until a person has read them. Ohho additionally refuses takeoff when `battery` is below 0.3 or `link_ok` is false. |
| Props-off bench | No propellers fitted until the checklist item that says props may be fitted. Software cannot detect this. The checklist is the control. |
| Audit | Existing JSONL audit in `~/.ohho/safety/`. Add the flight request, the frame, and the backend id. |

`SafetyProfile` on `origin/main` is built around planar caps and a Go2 action allowlist that rejects unknown actions (`ohho/safety.py` `profile_for`). A drone profile that reuses `GO2_ACTIONS` will refuse `land`. The profile needs its own allowlist. That is real ohho-os design work, not a JSON file of Go2 actions with the speeds turned down.

Tilt latch at 0.6 rad (`TILT_LIMIT_RAD`) will trip during a normal aggressive attitude and maybe during a fast stop. For M1 sim, do not enable `require_orientation` until the stub publishes a stable attitude. For hardware, start with the gate's tilt latch and expect to tune it after tethered hover, with a person watching. Do not disable it to get a demo.

## 8. Hardware bring-up checklist

Every line is **needs Varun present** and **needs hardware not yet bought**, except the legal IDs, which need Varun and do not need the airframe.

M2, X500 + Pixhawk 6C, props off until step 7:

1. Unbox against the Holybro kit list. Confirm the flight controller is a 6C (or 6X if that option was ordered) and the GPS is an M10. Needs Varun. Needs hardware.
2. Install QGroundControl on the laptop. USB to the Pixhawk. Firmware tab shows the PX4 version. Needs Varun. Needs hardware.
3. Airframe selection X500, sensor calibration, radio calibration, flight-mode switch, kill switch, battery chemistry. No motors plugged in, or motors plugged in with props off. Needs Varun. Needs hardware.
4. Failsafe bench: disconnect RC, confirm the configured RC-loss action. Disconnect the companion stream, confirm offboard-loss action. Geofence breach can be tested in SITL first; on the bench, only confirm the parameters were written. Needs Varun. Needs hardware.
5. Companion (Pi 5) powers separately. UART or Ethernet to the Pixhawk TELEM port. `MicroXRCEAgent` shows the client. `ros2 topic echo` shows `/fmu/out/vehicle_status` with `armed` false. Needs Varun. Needs hardware.
6. Ohho `px4+dds://` connects read-only (telemetry, no arm). Model this on `ohho connect --read-only` from branch `ohh-120-cli-go2-backend`. Needs Varun. Needs hardware.
7. Props fitted only here. Tether or a cage. A second person. Kill switch in hand. Arm from QGroundControl, not from MCP. Hover, land. Needs Varun. Needs hardware.
8. First outdoor flight only after Flyer ID and Operator ID, aircraft labelled, Open category, far from people, visual line of sight, geofence set, RTL proven in the cage. Not a turbine. Needs Varun. Needs hardware. Needs the IDs.

M3, Crazyflie, indoors:

1. Buy decision is still Varun's. Needs hardware not yet bought.
2. Firmware and radio address set from Bitcraze's client, in a room with the props on only when the aircraft is in a net. Needs Varun. Needs hardware.
3. `backend:=sim` script green before the radio script. The sim backend does not implement every velocity command (research section 9). Needs Varun to confirm the script matches the backend table. The sim half can be agent-written.
4. First flight in a net, Flow deck or Lighthouse, e-stop cuts motors. Needs Varun. Needs hardware.

No step authorises setting `OHHO_ARM_HARDWARE` from an agent session.

## 9. Milestones

### M1 — SITL

PX4 v1.17.0 + Gazebo Harmonic `gz_x500`, Ohho `px4-dds` and `drone-sim`, sim-serve, CI square, teleop against sim, MCP sim tools, one recorded sim episode.

Exit:

- CI job green on the square criteria in section 3.
- `sim://` drone spec takeoff / square / land without PX4, so laptop tests do not need Docker.
- `emergency_stop` in the PX4 job ends in a landing and a disarm, and the test asserts force-disarm was not sent.
- MCP cannot arm or take off.
- A LeRobot v2.1 directory from a sim episode opens and has the 11-D state and 4-D action.
- README of `ohho-drone` still does not claim a flight.

### M2 — X500 V2 + Pixhawk 6C

Exit: checklist section 8 through the tethered hover, done by Varun, with a note in the repo that says the date and the firmware hash. Outdoor flight is a separate exit and may slip without failing M2's tethered exit. Both need hardware that is not bought.

### M3 — Crazyflie indoor

Exit: one netted hover and land through the Ohho backend, plus the sim backend test. Needs hardware.

### M4 — ArduPilot via MAVLink

Exit: the same square in ArduPilot SITL through `mavlink+udp://`, and a documented parameter diff versus PX4. Not a second hardware purchase.

## 10. Epic and issues

### Epic

**Drone track, sim first.** Build an `aerial` capability in ohho-os with a PX4 uXRCE-DDS backend and a kinematic sim, prove a square flight in GitHub Actions, and only then sit a person in front of an X500 that has not been bought yet. Crazyflie and ArduPilot follow. Closed vendor SDKs are out. Draft PR #2 is a source of tests and an e-stop policy, not the flight stack.

Rough points are effort hints for sequencing, not a commitment.

### Table

| ID | Title | Repo | MS | Parallel | Who | Pts | Depends |
|---|---|---|---|---|---|---|---|
| D-01 | Port PR #2 schema, e-stop table, and tests; do not port the codec | ohho-os | M1 | yes, with D-02 | agent | 3 | none |
| D-02 | Add `aerial` capability, frames, and `Robot` methods | ohho-os | M1 | yes, with D-01 | agent | 5 | none |
| D-03 | Kinematic `drone-sim` backend and `px4-sitl` spec | ohho-os | M1 | after D-02 | agent | 5 | D-02 |
| D-04 | Drone safety profile, fence reject, e-stop land/hold | ohho-os | M1 | after D-02 | agent | 5 | D-01, D-02 |
| D-05 | `sim-serve` aerial JSON, deadman, flight requests | ohho-os | M1 | after D-03 | agent | 5 | D-03, D-04 |
| D-06 | `px4-dds` transport, QoS, message-version check | ohho-os | M1 | after D-02 | agent | 8 | D-02, D-04 |
| D-07 | SITL CI square on the pinned PX4 image | ohho-drone | M1 | after D-06 | agent | 5 | D-06 |
| D-08 | LeRobot v2.1 aerial episode from sim teleop | ohho-os | M1 | after D-03 | agent | 3 | D-03, D-05 |
| D-09 | MCP drone tools, denylist, `backend` field | ohho-os | M1 | after D-04 | agent | 5 | D-03, D-04 |
| D-10 | Read-only `ohho connect` and `doctor` for `px4+dds` | ohho-os | M1 | after D-06 | agent | 3 | D-06 |
| D-11 | Console sim view of `sim-serve` pose | console | M1 | after D-05 | agent | 3 | D-05 |
| D-12 | VR deadman and confirmation gates for `AerialDriveScheme` | OhhO-VR | M1 | after D-05 | agent for the script, Varun for a headset check | 5 | D-05 |
| D-13 | Android: refuse a drone URI until climb exists | ohho.apk | M1 | yes | agent | 2 | none |
| D-14 | Purchase decision recorded; no order placed by an agent | ohho-drone | M2 | yes | Varun | 1 | M1 exit |
| D-15 | Bench, props off, QGC calibration, failsafe parameter check | ohho-drone | M2 | no | Varun | 5 | D-14 |
| D-16 | Companion agent and read-only Ohho link on the bench | ohho-os | M2 | after D-15 | Varun, agent only for the read-only command | 3 | D-10, D-15 |
| D-17 | Tethered or caged hover | ohho-drone | M2 | no | Varun | 5 | D-16 |
| D-18 | First outdoor flight under the CAA rules in the research note | ohho-drone | M2 | no | Varun | 3 | D-17, Flyer ID, Operator ID |
| D-19 | Crazyflie backend against `backend:=sim` | ohho-os | M3 | after M1, parallel with M2 paperwork | agent | 5 | D-02, D-04 |
| D-20 | Netted Crazyflie hover | ohho-drone | M3 | no | Varun | 5 | D-19, hardware |
| D-21 | MAVLink backend on MAVSDK 4, PX4 SITL first | ohho-os | M4 | after M1 | agent | 8 | D-04 |
| D-22 | ArduPilot SITL square | ohho-drone | M4 | after D-21 | agent | 5 | D-21 |

### List

**D-01. Port PR #2 schema, e-stop table, and tests.** Repo ohho-os. Milestone M1. Bring `DroneTelemetry` and the land/hold/disarm decision into `ohho/` without `ohho_drone/mavlink/codec.py`. Acceptance: unit tests fail if a disarm is sent while `landed_state` is unknown or in the air, and if param2 equals 21196. Parallel with D-02. Agent. 3.

**D-02. `aerial` capability and Robot methods.** Repo ohho-os. M1. `ohho/capabilities.py`, `ohho/robot.py`. Acceptance: a spec without `aerial` no-ops; a spec with it dispatches `takeoff`, `land`, `rtl`, `hold`, `offboard_position`, `offboard_velocity` with an explicit frame. Parallel with D-01. Agent. 5.

**D-03. Kinematic sim.** Repo ohho-os. M1. `ohho/adapters/drone_sim.py`, registry id `px4-sitl`. Acceptance: takeoff to 2.5 m, a 4 m square, land, using `step(dt)` only. Depends D-02. Agent. 5.

**D-04. Safety profile and e-stop.** Repo ohho-os. M1. New allowlist, fence reject, e-stop mapping per backend. Acceptance: Go2 tests still pass; a drone profile accepts `land` and rejects `flip`; an in-air e-stop does not disarm. Depends D-01, D-02. Agent. 5.

**D-05. sim-serve.** Repo ohho-os. M1. Extend the WebSocket documented in `docs/sim-serve.md`. Acceptance: deadman false or 300 ms silence sends a zero velocity; `estop: true` lands in sim; `takeoff` on a robot backend without the phrase is refused. Depends D-03, D-04. Agent. 5.

**D-06. px4-dds.** Repo ohho-os. M1. `ohho/adapters/px4_dds.py`. Acceptance: against a mock DDS graph (no Gazebo in the unit test), arming is refused until `OffboardControlMode` has been published, QoS is best-effort, and a message-field mismatch refuses connect. Depends D-02, D-04. Agent. 8.

**D-07. SITL CI.** Repo ohho-drone. M1. Workflow in section 3. Acceptance: the square criteria, artifacts on failure, no `OHHO_ARM_HARDWARE`. Depends D-06. Agent. 5.

**D-08. Episodes.** Repo ohho-os. M1. Acceptance: `meta/info.json` has `codebase_version` v2.1, `robot_type` `px4-sitl`, state names and action names as in section 5. Depends D-03, D-05. Agent. 3.

**D-09. MCP.** Repo ohho-os. M1. Acceptance: tests prove `drone_arm` and `drone_takeoff` are absent; `drone_land` and `emergency_stop` work on sim; a hardware URI without `--allow-hardware` errors; every payload has `backend`. Depends D-03, D-04. Agent. 5.

**D-10. Read-only connect and doctor.** Repo ohho-os. M1. Mirror `ohh-120`. Acceptance: read-only publishes nothing to `/fmu/in`. Depends D-06. Agent. 3.

**D-11. Console sim view.** Repo console. M1. Acceptance: a sim page shows x, y, z from a fake sim-serve socket. No claim of a live aircraft. Depends D-05. Agent. 3.

**D-12. VR gates.** Repo OhhO-VR. M1. Deadman and phrase for takeoff/land. Headset check needs Varun. Depends D-05. 5.

**D-13. Android refusal.** Repo ohho.apk. M1. Acceptance: selecting a drone profile does not publish `/cmd_vel/teleop` until a climb axis exists. Parallel. Agent. 2.

**D-14. Purchase decision.** Repo ohho-drone. M2. A checked-in note that names what was bought, by Varun. An agent does not order. 1.

**D-15. Bench, props off.** Repo ohho-drone. M2. Checklist steps 1–4 signed by Varun. Needs hardware. 5.

**D-16. Companion link, read-only.** Repo ohho-os. M2. Agent up, Ohho connected, armed flag false. Needs Varun. 3.

**D-17. Tethered hover.** Repo ohho-drone. M2. Needs Varun. 5.

**D-18. First outdoor flight.** Repo ohho-drone. M2. Needs Varun, Flyer ID, Operator ID, and a site that is Open-category far-from-people. Not a turbine. 3.

**D-19. Crazyflie sim backend.** Repo ohho-os. M3. Acceptance: high-level takeoff/land against the commands the `sim` backend actually implements. Agent. 5.

**D-20. Netted hover.** Repo ohho-drone. M3. Needs Varun and hardware. 5.

**D-21. MAVSDK backend.** Repo ohho-os. M4. Acceptance: PX4 SITL square through `mavlink+udp://127.0.0.1:14540` with native `mavsdk` 4.x, no `mavsdk_server`. Agent. 8.

**D-22. ArduPilot SITL.** Repo ohho-drone. M4. Same square, Copter stable pinned in the workflow. Agent. 5.

D-11, D-12, and D-13 can run in parallel once D-05 exists. D-19 can start once D-04 exists and does not need the X500. D-15 onward cannot.

## 11. Bill of materials

**Not purchased.** Prices from the research note, 19 October 2026. Re-check the page on the day of an order.

### M2

| Line | Price | In the $609 kit? |
|---|---|---|
| Holybro PX4 Development Kit X500 v2 | $609 | this is the kit |
| Pixhawk 6C Mini | from $179 | no, and not needed if the kit's 6C is present |
| RadioMaster TX15 ELRS mode 2 | $153.99 | no |
| RP1 V2 ELRS receiver | $18.99 | no |
| Raspberry Pi 5, 16 GB | $305 official page | no |
| Raspberry Pi 5, 8 GB | not confirmed from the official page in this pass | no |
| 4S 3000–5000 mAh XT60 and a balance charger | price not verified | no |
| Jetson Orin Nano Super dev kit | $399 only via a secondary 15 Sep 2026 article | no; do not buy for M2 |

Kit plus TX15 plus RP1 plus Pi 5 16 GB is $609 + $153.99 + $18.99 + $305 = **$1,086.98** before tax, shipping, battery, and charger. The battery line is deliberately missing. A 6C Mini is not added.

### M3

| Line | Price |
|---|---|
| Crazyflie 2.1+ | $240 |
| Crazyradio 2.0 | $43 |
| Flow deck v2 | $55 |
| Subtotal for a netted Flow-deck aircraft | **$338** |
| Alternative: 2.1 Brushless getting-started bundle | $500 |
| Lighthouse explorer bundle, brushless, if Flow deck is not enough indoors | $1,050 |

A net is required and was not priced.

### Not in either milestone

Livox Mid-360 (£569 DJI UK, out of stock, or $1,350 at a US reseller, sold out), a global-shutter camera, a downward rangefinder, Starling 2 (from $2,949.99), Starling 2 Max ($4,199.99), VOXL 2 ($1,399.99).

## 12. Risks and open questions

| Risk | Why it matters | Mitigation in this plan |
|---|---|---|
| Hand-rolled MAVLink in PR #2 looks done | It covers nine messages and mis-labels frames | Supersede it. Keep the e-stop tests. |
| `SafetyGate.arm` confused with motor arm | MCP on `6acd7e1` calls `robot.arm()` | Separate `arm_flight`. MCP does not call either on hardware. |
| Offboard stream dies, last setpoint sticks | Aircraft keeps the last velocity | Watchdog stops streaming. PX4 failsafe owns the outcome. |
| Tilt latch trips in normal flight | `TILT_LIMIT_RAD` is 0.6 rad, written for a quadruped | Do not copy the Go2 profile. Tune only after a tethered hover. |
| Gazebo CI is too slow or too large | Image is about 2 GB; no measurement yet | 20 min budget, then SIH on pull requests. |
| v1.18 RC is tempting | Message and Kconfig differences | Pin v1.17.0. |
| ArduPilot DDS looks like PX4 DDS | Takeoff service is experimental; README of `ardupilot_ros` is stale | MAVLink first, in M4. |
| VR catalog speed is 12 m/s | `DriveSpec.Quadrotor` | Python cap is 1.5 m/s. VR is not the safety layer. |
| UK class mark and radio band | Kit may not be UK-marked; 915 vs 433 | Varun checks the Drone Code and Ofcom before an outdoor flight. |
| VOXL Ubuntu 18.04 | Not Jazzy | Not M2. |
| Inspection demo pressure | A turbine flight is a different legal category | First outdoor flight is Open category, empty field, or it does not happen. |

Open questions carried from the research note: `COM_*` numeric defaults, v1.17 versus the v1.18 DDS Kconfig, MAVSDK 4 URL grammar, Copter 4.7.2 fence defaults, Pi 5 8 GB official price, Jetson price on nvidia.com, battery price, VOXL ROS distro, and whether Jazzy runs on the Pi as Ubuntu 24.04 or in a container.
