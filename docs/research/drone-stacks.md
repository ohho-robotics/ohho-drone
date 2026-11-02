# Drone stacks

Research notes for a general Ohho aerial track. Written 19 October 2026. This repository still has no flight code. Nothing here has been flown, bought, or run on a simulator from this machine.

Every version below is tied to a page read on that date. Where a page did not print a fact, the gap is named. Prices are shelf prices from the linked page, not a quote, and they exclude shipping, tax, and spares unless the page says otherwise. Nothing in the bill of materials has been purchased.

Ohho's own track is a robot backend, not a wind-turbine product. The inspection mapping is called out only where the same mechanisms (companion computer, payload, ground link, geofence, orbit) would matter to that job.

## 1. What exists in Ohho today

Read-only. No file outside this worktree was modified.

| Piece | Where it was read | What it actually is |
|---|---|---|
| This repo on `main` | `ohho-drone` `origin/main` `ef6d188`, and this branch before these notes | README, Apache-2.0 `LICENSE`, `scripts/check-credibility.sh`. The README says the tree does not contain a robot description, flight controller, simulator, or model. |
| Draft flight prototype | `ohho-drone` worktree `C:\Users\varun\ohho\ohho-drone-ohh-67`, branch `ohh-67-px4-sitl-mavlink`, draft PR #2, Linear OHH-67 | Pure-Python MAVLink adapter, hand-written codec, in-process mock, tests. Not merged. Assessed in section 3. |
| Robot API | `ohho-os` clone `C:\Users\varun\ohho\ohho-sdk`, `git show origin/main` at `280fedfa` | `ohho/robot.py`, `ohho/transport.py`, `ohho/capabilities.py`, `ohho/registry.py` |
| Transports | same ref | `ohho/adapters/__init__.py` resolves `sim://`, `serial://`, `feetech://`, `dds://<iface>`, `ros2://<namespace>`. No aerial scheme. |
| ROS 2 bridge | `ohho/adapters/ros2.py`, `ohho/runtime/ros2.py` | One rclpy node. Publishes `geometry_msgs/Twist` on `/cmd_vel` with `linear.z` forced to 0. Subscribes to `/odom` and, if the spec has `manipulation`, `/arm/joint_states`. |
| Go2 DDS path | `ohho/adapters/unitree.py` | `dds://<iface>?domain=<n>` over Unitree CycloneDDS, not PX4 XRCE-DDS. `velocity_to_move` is planar `(vx, vy, vyaw)`. |
| Safety | `ohho/safety.py`, module docstring | `SafetyGate` wraps every transport whose `protocol != "simulated"`. Disarmed until `arm()`. Hardware arming needs `OHHO_ARM_HARDWARE=1`, a TTY, and the typed phrase. Watchdog: 300 ms without a velocity command zeros velocity; 500 ms without state while armed latches. E-stop latches, calls the inner `emergency_stop`, and leaves the gate disarmed after release. This is a software guard in-process. It says so itself: it does not replace a hardware e-stop. |
| Capabilities | `ohho/capabilities.py` | `base.drive`, `base.holonomic_drive`, `legged.pose`, `manipulation`, `perception.rgb`, `perception.depth`, `perception.scan`. No aerial capability. |
| Robots | `ohho/registry.py` | `omnibot`, `unitree-go2`, `sim`. No drone spec. |
| sim-serve | `docs/sim-serve.md` on that same `origin/main` | WebSocket JSON: `velocity.linear.{x,y}`, `velocity.angular.z`, `joints`, `deadman`, `estop`. Deadman false, or 300 ms of silence, commands zero. Backends on this ref: `sim`, `ros2` via rosbridge. `isaac` and `mujoco` exit non-zero and print `not built`. |
| Episodes | `ohho/data/recorder.py`, `ohho/data/writer.py` | LeRobot **v2.1** (`CODEBASE_VERSION = "v2.1"`). State and action are arm joints plus planar base velocity. `robot_type` is hard-coded `"omnibot"`. No camera or LiDAR column on this ref. |
| Profiles | `ohho/profiles.py` | `pi_workstation` (Pi 5 control+perception, workstation inference), `jetson_single`, `workstation_single`, `mac_dev`, `edge_cpu`. These are compute roles, not a flight stack. |
| Self-test | `ohho/selftest.py` | Serial, wheel encoders, IMU rate, STS3215 servos, camera frames. No airframe checks. |
| CLI | `ohho/cli.py` | `ohho sim-serve`, `ohho connect`, `ohho drive`. `drive` is planar. |

`SafetyGate.arm()` enables motion commands inside Ohho. It does not spin propellers. A later drone `arm()` on the autopilot is a second action and must not be folded into the gate.

### Go2 pattern this track should mirror

These branches are in progress or stacked. They are not all on `origin/main`. Cite the commit that was checked out.

| Worktree | Branch / commit | Pattern to copy |
|---|---|---|
| `C:\Users\varun\ohho\ohho-os-ohh-118` | `ohh-118-legged-api` `8399616` | A capability (`legged.pose`) gates `Robot.stand` / `lie_down` / `sit` / `recover`. The transport grows `send_mode`. Callers that lack the capability no-op. |
| `C:\Users\varun\ohho\ohho-os-ohh-120` | `ohh-120-cli-go2-backend` `490bd29` | `ohho connect --read-only` subscribes to DDS state and does not construct the sport client. `ohho doctor` is the bring-up probe. `--arm` is explicit and still needs `OHHO_ARM_HARDWARE`. |
| `C:\Users\varun\ohho\ohho-os-ohh-127` | `ohh-127-go2-agent-brain` `90079c9` | `ohho/brains.py` registers tools from capabilities. `emergency_stop` is always registered. A travel-limit wrapper can refuse `drive` past a radius. |
| `C:\Users\varun\ohho\ohho-os-ohh-129` | `ohh-129-go2-episode-logging` `6c3dce8` | Recorder grows a robot-specific state vector (12 Go2 leg joints) and optional `observation.images.*` video. `robot_type` becomes `unitree-go2`. This is ahead of `origin/main`. |
| `C:\Users\varun\ohho\ohho-os-ohh-130` | `ohh-130-go2-teleop-robot-backend` `910b598` | `ohho sim-serve --backend robot --robot unitree-go2 --transport ...`. Server messages gain `backend` and optional `posture`. |
| `C:\Users\varun\ohho\ohho-os-ohh-265` | `ohh-265-go2-mujoco-backend` `a5907db` | Loopback sim is `dds://lo?domain=1`. Domain 0 stays the hardware domain and still requires the hardware arming phrase. `docs/sim-serve.md` on that branch shows `ohho sim-serve --backend robot`. `--backend mujoco` still prints `not built`. The MuJoCo simulator is an external process, not an in-process backend. |
| `C:\Users\varun\ohho\ohho-quadruped` | README on that clone | The form-factor repo is still a concept placeholder. The backend lives in ohho-os. The drone repo should follow that split. |

### MCP, as the branch actually reads

`C:\Users\varun\ohho\ohho-os-ohh-278`, branch `ohh-278-mcp-safety`, commit `6acd7e1`.

What is in the tree:

- `docs/mcp.md` documents `ohho mcp serve` as sim-only for the five Link v0 tools `connect`, `drive`, `telemetry`, `nav_goto`, `look`.
- `ohho/mcp_tools.py` `_normalize_sim_transport` rejects any scheme other than `sim` for those tools. `tests/test_mcp.py` `test_connect_rejects_hardware` covers `serial:///dev/ttyUSB0` and `dds://eth0`.
- `connect_emergency_stop` calls `session.robot.emergency_stop()` and is not behind `_require_sim` in the handler itself; it errors only when nothing is connected.
- `RobotSession.connect` calls `self._robot.arm()` when the robot is gated. On `sim://` the gate is not wrapped (`Robot.connect` skips `SafetyGate` for `protocol == "simulated"`), so this is a no-op arm for the current sim path. It is still an arm call in the MCP process.
- The on-robot catalog tool `connect.connect` writes `session.transport` from the caller and does **not** call `_normalize_sim_transport`.
- Tool results do not carry a `backend: sim|hardware` field. `sim-serve` does, on a different API.
- No `--allow-hardware` flag was found under `ohho/cli.py` or `ohho/mcp_*.py` at this commit.
- No denylist of tool names `arm`, `release`, or `limit` was found in `ohho/mcp_tools.py`. The safety-module denylist (`flip`, `jump`, `free*`, and the rest in `ohho/safety.py`) is a sport-action denylist, not an MCP tool denylist.

The drone tool surface in the implementation plan follows the stricter contract the OHH-278 brief asks for (sim unless a human passes `--allow-hardware`; MCP never arms or takes off on hardware; e-stop/land always allowed; denylist; `backend` on every result; clamps; geofence before setpoints). That contract is **not** fully present in commit `6acd7e1`. The plan must not pretend it is.

### Teleop clients, as they read today

| Client | Path | What it does |
|---|---|---|
| VR ground drive | `OhhO-VR/Assets/Scripts/Input/BaseController.cs` | Left stick Y forward, left stick X strafe, right stick X yaw. Both grips publish a bool e-stop. Right grip alone is a 2× turbo, still clamped. Publishes `Twist` on `/cmd_vel/teleop` through `ROSBridgeClient`. `linear.z` is 0. |
| VR aerial drive | `OhhO-VR/Assets/Scripts/Control/Drive/AerialDriveScheme.cs` | Mode 2: left stick Y → `linear.z` (up positive), left stick X → `angular.z` (right positive, not negated), right stick Y → `linear.x`, right stick X → `linear.y`. Idle mode string is `"offboard"`. `DriveSchemeBase.cs` uses both grips as e-stop. There is no "hold to fly" deadman. |
| VR catalog speeds | `OhhO-VR/Assets/Scripts/Control/DriveSpec.cs` | `DriveKind.Quadrotor` is stored as max linear **12 m/s** and max angular **3 rad/s**. That is a catalog default, not a safety cap Ohho should fly. |
| Android | `ohho.apk/.../ui/controls/ControlsFragment.kt` | One stick: `linearX = y * 1.0`, `angularZ = -x * 1.5`, strafe 0, no vertical axis. `Constants.kt` publishes `/cmd_vel/teleop` over ROSBridge. E-stop is a toggle button. |
| Console | `console/components/pilot/PilotConsole.tsx` | Default provider URL `ws://localhost:9090` (rosbridge). `console/lib/products.tsx` describes mode-2 sticks for a drone as product copy. No drone physics backend was found in `console/lib/pilot/sim/`. |
| sim-serve consumers | `docs/sim-serve.md` on ohho-os `origin/main` | The socket Ohho already has for browser teleop is `ohho sim-serve` (default port 8765), with `deadman` and `estop`. The VR and Android apps do not speak that socket today; they speak ROSBridge. |

## 2. Recommendation in one paragraph

Use **PX4 on a Pixhawk, ROS 2 over uXRCE-DDS**, as the primary reference, pinned to the current stable firmware rather than `main` or the v1.18 release candidate. Use **MAVSDK's native Python binding (or pymavlink underneath it)** as the second backend so the same Ohho calls can talk to PX4 and ArduPilot without a hand-written dialect. Treat ArduPilot's own DDS path as real but younger, and schedule it after the PX4 sim loop works. Use **Crazyswarm2** for a small indoor aircraft. Leave DJI, Parrot, and Skydio as a paragraph, not a milestone. Draft PR #2 is not the base of milestone 1. Keep its schema, its e-stop policy, its mock, and its tests. Replace the codec.

## 3. Draft PR #2 (OHH-67)

Read at `C:\Users\varun\ohho\ohho-drone-ohh-67`. Not merged. The README on that branch still says "Concept — not started" and then documents an experimental simulation sketch. `AGENTS.md` there lists `ohho_drone/adapter.py`, `ohho_drone/mavlink/`, `ohho_drone/sitl/mock_server.py`, `scripts/px4_sitl_hello.py`, and `scripts/run_smoke_test.py`.

### Reuse

- `ohho_drone/schema.py`: `FlightMode`, `DroneVelocity` (body forward / right / up, yaw rate), `DronePose`, `GlobalCoordinates`, `DroneTelemetry`, and `to_ohho_telemetry()` stuffing the aerial fields into `Telemetry.custom["drone"]`. That matches how Go2 puts roll, pitch, and `body_height` into `Telemetry.custom` (`ohho/adapters/unitree.py` `sportstate_to_telemetry`).
- E-stop policy in `ohho_drone/adapter.py` `emergency_stop` and `docs/mavlink-adapter.md` section 5. In the air it sends `MAV_CMD_NAV_LAND`. If landed state is unknown it sends hold (`MAV_CMD_DO_SET_MODE` to PX4 `AUTO.LOITER`) and then land. It disarms with `MAV_CMD_COMPONENT_ARM_DISARM` param1=0 param2=0 only when `EXTENDED_SYS_STATE` says on the ground and altitude agrees. It never sends force-disarm magic `21196`. The software latch blocks later velocity until `release_stop()`. This is the right Ohho-level e-stop. It is not a motor kill.
- `simulation_target_allowed`: `connect()` refuses anything that is not loopback, plus a short docker-local name list when `sitl=True`. A hardware backend must not weaken this by accident. Hardware belongs behind `SafetyGate`, on an explicit URI, with the arming phrase.
- `ohho_drone/sitl/mock_server.py` and the tests (`tests/test_adapter.py`, `tests/test_mock_sitl.py`, `tests/test_mavlink_*.py`, `tests/test_hello_script.py`). They are a contract for "arm, take off, velocity, land, e-stop does not force-disarm". They are not a dynamics model.
- The port map in `docs/px4-sitl.md`: the transport sends to PX4 SITL's local UDP port 14580. PX4 sends its offboard stream to UDP 14540. QGroundControl uses UDP 14550.

### Replace

- `ohho_drone/mavlink/codec.py`, `packet.py`, and `messages.py`. The codec understands nine messages (heartbeat, sys status, attitude, local position, global position, command long, command ack, position target, extended sys state). A real PX4 stream also carries GPS status, battery status, estimator flags, statustext, ping, and parameter traffic. CRC extras are hand-copied. This will drift from the XML dialect. **pymavlink 2.4.50** generates codecs from the XML (`https://pypi.org/project/pymavlink/`, 19 Oct 2026). **MAVSDK 4** sits above that and already implements arm, takeoff, land, RTL, offboard velocity, and telemetry.
- The SITL instructions that pull `px4io/px4-sitl:v1.14.0` and `HEADLESS=1 make px4_sitl none_iris`. v1.14 is two stables behind v1.17.0. Current containers are in section 8.
- A few mappings that should not be copied forward:
  - `DronePose.z` keeps NED down, while altitude for the e-stop is `-z` and `GlobalCoordinates.relative_alt`. Callers can mix them.
  - `send_drone_velocity` labels the frame `MAV_FRAME_BODY_NED` but copies `linear_x` straight into `vx` with no yaw rotation. That is body-forward only if the autopilot treats the frame as body FRD. The comment diagram in `docs/mavlink-adapter.md` also calls `linear_x` "North / Body X", which conflates two frames.
  - Heartbeat parsing sets `FlightMode.STABILIZED` for any armed heartbeat. It does not decode PX4 custom mode.
  - Takeoff uses `MAV_CMD_NAV_TAKEOFF` param7 as a single altitude and does not switch to a setpoint stream. PX4 offboard will not track that the way section 4 describes.
  - The type mask `0x05C7` does ignore position, acceleration, and yaw, and does use velocity and yaw rate. The comment that bit 9 means "ignore force" is loose: in the MAVLink mask, bit 9 selects force-instead-of-acceleration. Leaving it clear is what a velocity setpoint wants. The mask can stay; the comment should not.

### Verdict

**Supersede PR #2 as the milestone 1 base.** Do not merge it and then rip the codec out. Milestone 1 should speak PX4 uXRCE-DDS. A later MAVLink backend should use MAVSDK or pymavlink. Port the schema, the e-stop decision table, the loopback refusal, the mock's command log, and the tests that lock those behaviours. The hand-written codec does not ship.

## 4. Primary stack: PX4 on a Pixhawk, ROS 2 via uXRCE-DDS

### Versions, as of 19 October 2026

| Item | Version | Date on the page | Source |
|---|---|---|---|
| Latest stable autopilot | **v1.17.0** | Published 2026-05-13 | https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0 |
| Previous stable | v1.16.0, point release v1.16.2 | v1.16.0 on 2025-08-06; v1.16.2 listed 2026-04-22 on a release index | https://docs.px4.io/main/en/releases/1.17 and https://releasealert.dev/github/PX4/PX4-Autopilot |
| In test, not the pin | v1.18.0-rc1 | 10 Sep 2026, marked pre-release | https://github.com/PX4/PX4-Autopilot/releases/ |
| `px4_msgs` line for v1.17 | branch `release/1.17`, changelog entry 1.17.0 dated 2026-03-22 | The changelog date is earlier than the firmware tag. Treat the branch name, not the changelog date, as the match. | https://github.com/PX4/px4_msgs/blob/main/CHANGELOG.rst and https://docs.ros.org/en/humble/p/px4_msgs/ |
| `px4_msgs` for v1.16 | `release/1.16`, package versions 1.16.0 through 1.16.2 with identical definitions | 1.16.0 changelog date 2025-08-07 | same changelog |
| ROS 2 PX4 recommends | **Jazzy on Ubuntu 24.04**. Humble on Ubuntu 22.04 is the documented alternative. | main user guide, read 19 Oct 2026 | https://docs.px4.io/main/en/ros2/user_guide |
| Micro XRCE-DDS Agent | Humble → agent **2.4.2**; Jazzy → agent **2.4.3**. The main guide's table is headed for PX4 **v1.18** and says the default client targets agent 2.x unless `UXRCE_DDS_CLIENT_USE_DDS_V3` is set. Kilted uses 2.4.3. Lyrical and Rolling use agent 3.0.2 and need the v3 flag. | main middleware guide | https://docs.px4.io/main/en/middleware/uxrce_dds |
| SITL image to pin | `px4io/px4-sitl-gazebo` tag `v1.17.0` (Gazebo Harmonic). `latest` moves. | main prebuilt-package guide | https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages |

v1.18 is a release candidate. This track pins **v1.17.0** until 1.18.0 is a stable tag. The v1.18 Kconfig flag must not be assumed present on the v1.17 tree; the table that mentions it is on the `main` guide and is labeled for v1.18.

`px4_msgs` must match the firmware. The ROS 2 user guide says applications need the same definitions the firmware was built with, and that from v1.16 a translation node can bridge differing versions. The v1.16 translation-node page is marked experimental and says topic names gain a `_v<version>` suffix when `MESSAGE_VERSION` is non-zero. Version 0 has no suffix. Service translation is not fully available on Humble (the main translation page says the service API it needs is not in Humble; topic translation is). Source: https://docs.px4.io/v1.16/en/ros2/px4_ros2_msg_translation_node and https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.

The `px4_msgs` ROS doc says the package builds on Humble, Jazzy, Kilted, Rolling, and Lyrical, and that an apt package exists once the build farm has published it. It also says building from the matching git branch is the recommended path today. Source: https://docs.ros.org/en/humble/p/px4_msgs/.

### How the bridge is wired

uXRCE-DDS replaced the Fast-RTPS bridge used in PX4 v1.13. The client `uxrce_dds_client` is generated at firmware build time from `msg/` and the subset listed in `src/modules/uxrce_dds_client/dds_topics.yaml`. It runs on the flight controller. The **Micro XRCE-DDS Agent** runs on the companion (or in SITL, on the host) and is the DDS participant that ROS 2 sees. The agent does not need the client sources.

SITL starts the client on UDP port **8888** unless the Zenoh target is used. Companion command, from the same guide:

```sh
MicroXRCEAgent udp4 -p 8888
```

On hardware the client is in the default firmware and is **not** started automatically except in simulator builds. It is started from an airframe startup script or the MAVLink shell, for example serial to a companion or `uxrce_dds_client start -t udp -p 8888 -h <companion>`. Source: https://docs.px4.io/main/en/middleware/uxrce_dds.

Topics the agent exposes are `/fmu/in/...` (ROS publishes, PX4 subscribes) and `/fmu/out/...` (PX4 publishes). A namespace prefix is possible (`-n drone` → `/drone/fmu/...`).

v1.17 also has an experimental Zenoh build (`make px4_sitl_zenoh`, and a default Zenoh firmware only on FMU-v6xRT). That is a second middleware, not the reference for milestone 1. Source: the v1.17.0 release notes, https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0.

### Offboard control

PX4 treats a stream of `OffboardControlMode` (ROS 2) or a MAVLink setpoint as proof of life at about **2 Hz**. The stream has to be running **before** arming in offboard or switching into offboard. If it stops for longer than `COM_OF_LOSS_T`, PX4 leaves offboard and runs `COM_OBL_RC_ACT`. The default number of `COM_OF_LOSS_T` was not copied off the parameter page in this pass; do not invent it. Source: https://docs.px4.io/main/en/flight_modes/offboard.

The ROS 2 example publishes both messages on a **100 ms** timer, sends `VehicleCommand` `VEHICLE_CMD_DO_SET_MODE` with param1=1, param2=6 (offboard), then arms with `VehicleCommand`. Position setpoints are `TrajectorySetpoint` in **local NED**: the example hovers at `{0, 0, -5}` (5 m above the local origin) with `yaw` in radians, range `[-pi, pi]`. There is no implicit NED↔ENU conversion. Source: https://docs.px4.io/main/en/ros2/offboard_control. The page still says the example was tested on Ubuntu 20.04, ROS 2 Foxy, and PX4 v1.14. The message fields it prints (`thrust_and_torque`, `direct_actuator`) match the v1.16+ `OffboardControlMode` (the `actuator` flag was replaced). Source for the field change: https://github.com/PX4/px4_msgs/releases v1.16.0 notes, and https://docs.px4.io/v1.16/en/msg_docs/OffboardControlMode.

`OffboardControlMode` flags select which setpoint PX4 will accept: `position`, `velocity`, `acceleration`, `attitude`, `body_rate`, `thrust_and_torque`, `direct_actuator`. For a velocity teleop, `velocity` is true and the rest are false, and `TrajectorySetpoint.velocity` is filled. Unused position fields must be NaN, not zero, or PX4 will treat them as a position command. The offboard page says this; the example only shows the position case. Implementers should read that page before writing the publisher. This note is the requirement, not a coded publisher.

`VehicleCommand` / `VehicleCommandAck` carry arm, disarm, mode, takeoff, land, and RTL. Those commands match MAVLink `MAV_CMD_*` numbers. Takeoff and land are modes, not offboard setpoints. The offboard page says operations such as takeoff, land, and RTL are better handled in their own modes.

By default PX4 will not arm with no ground station and no RC. The offboard example says to connect QGroundControl first. `COM_RC_IN_MODE = 4` can disable the manual-control requirement. This track should not set that on a real aircraft. SITL CI can, because there is no pilot.

### QoS

PX4 publishers use best-effort, transient-local, keep-last, depth 0. PX4 subscribers use best-effort, volatile, keep-last. ROS 2 defaults are reliable, volatile, keep-last 10. A ROS 2 subscription with the default QoS will often not connect to a PX4 publisher. Source: https://docs.px4.io/main/en/middleware/uxrce_dds section "PX4 ROS 2 QoS Settings".

Ohho's current `Ros2Transport` uses the rclpy default depth-10 reliable QoS (`ohho/adapters/ros2.py` `create_publisher(..., 10)`). A PX4 adapter cannot reuse that call unchanged.

### `px4_ros_com` and the interface library

The offboard example still lives in `PX4/px4_ros_com` and the user guide still tells you to clone it. A PX4 developer reply on 1 October 2026 says `px4_ros_com` is not kept in step with `px4_msgs` `main` (`sensor_gps` renamed to `sensor_gnss` is the example they gave) and points people at the PX4 ROS 2 Interface Library. Source: https://discuss.px4.io/t/unable-to-build-ros-2-workspace-due-to-missing-px4-msgs-dependency/49565.

That library (`Auterion/px4-ros2-interface-lib`) is the maintained C++ API, with Python bindings marked incomplete. It registers ROS 2 modes with PX4, sends typed setpoints, and can forward a navigation estimate. The PX4 guide marks parts of it experimental. It checks message compatibility at startup and has `release/1.16` (and the README's pattern for other releases). `rmw_zenoh` with this library needs Jazzy or newer. Sources: https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib and https://github.com/Auterion/px4-ros2-interface-lib.

Milestone 1 should publish the uORB topics directly from a small ROS 2 node, the same way the offboard example does, against `px4_msgs` `release/1.17`. The interface library is an option once a mode needs to register itself. It is not required to arm, take off, fly a square, and land.

### How this lines up with Ohho's ROS 2

`ohho/runtime/ros2.py` `Ros2Runtime` owns one rclpy node and can subscribe and publish. `Ros2Transport` already attaches publishers to that node. A PX4 transport should take the same node, add a best-effort QoS profile, and use `/fmu/in` and `/fmu/out` instead of `/cmd_vel` and `/odom`.

It should not pretend `Twist.linear.z = 0` is a drone. `Robot.drive(vx, vy, w)` stays the planar call. Aerial setpoints are new methods behind a new capability, the same way `legged.pose` added `stand()` without overloading `drive()`.

The Go2 `dds://` scheme is CycloneDDS to a Unitree sport client. PX4's DDS session is a different process (the XRCE agent) and a different type system (`px4_msgs`, not `unitree_sdk2py`). Do not overload `dds://`.

## 5. MAVLink backends: MAVSDK, MAVROS, pymavlink

These are the broad path for "a flight controller that speaks MAVLink", which includes PX4 and ArduPilot. They are the second Ohho backend, not the primary one. PX4's own ROS 2 topics are a closer match to Ohho's existing ROS 2 runtime and avoid a second protocol stack on the companion.

### MAVSDK

As of 19 Oct 2026 the PyPI package `mavsdk` is **4.0.6** and is a binding to the C++ library through a C wrapper. It does **not** start `mavsdk_server`. It exposes `mavsdk` (threads) and `mavsdk.asyncio`. Source: https://pypi.org/project/mavsdk/.

The GitHub releases page lists **v4.0.6** as the newest release, stamped 8 Oct, with v4.0.0 stamped 22 Sep on the same page. Source: https://github.com/mavlink/MAVSDK/releases. The v4.0.0 notes describe new Python, C, and Kotlin bindings and say the gRPC server is no longer the Python package.

The old gRPC wrapper is PyPI `mavsdk-grpc` **4.0.5**, uploaded 2026-10-07, import name `mavsdk_grpc`. It still needs the C++ `mavsdk_server` process, default gRPC port 50051. Source: https://pypi.org/project/mavsdk-grpc/.

Prefer `mavsdk` 4.x in-process. One less process to supervise, and it matches "who runs mavsdk_server?": nobody, if we stay on the native binding. Keep `mavsdk-grpc` as a fallback only if a plugin we need is missing from the native wrappers. That gap was not audited message-by-message in this pass.

MAVSDK speaks both PX4 and ArduPilot. Connection URLs in the v3 era used forms such as `udpin://`. The Ohho URI should be `mavlink+udp://127.0.0.1:14540` and the adapter should translate it. Confirm the v4 string at implementation time against https://mavsdk.mavlink.io/main/en/python/ rather than freezing a v3 example here.

SITL's offboard port remains UDP 14540. QGroundControl stays on 14550. Source: https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages.

### pymavlink

PyPI lists **pymavlink 2.4.50** (LGPLv3, Python 3). Source: https://pypi.org/project/pymavlink/. The GitHub releases page shows v2.4.50 as the top release, stamped 24 Sep. Source: https://github.com/ArduPilot/pymavlink/releases. Use it when Ohho needs a message MAVSDK does not wrap, or inside tests that want to assert on raw frames. Do not make it the public adapter. The PR #2 codec is a partial reimplementation of this library.

### MAVROS

The ROS package index lists **mavros 2.16.0**, license GPLv3, build type `ament_cmake`, VCS branch `ros2`, last updated **2026-09-27**, dev status DEVELOPED. The page says it is the ROS 2 node: serial, UDP, or TCP to PX4 or ArduPilot, a GCS proxy, and an OFFBOARD plugin. It also says "Only for Linux." Source: https://index.ros.org/p/mavros/.

MAVROS is alive on ROS 2. It is a poor fit as Ohho's primary backend: GPLv3 next to Ohho's Apache-2.0 tree needs a conscious decision, it adds a node between Ohho and the autopilot, and the uXRCE-DDS path is the one PX4 documents first. It remains a valid way to attach an existing ROS graph. Not a milestone.

## 6. ArduPilot's own DDS / ROS 2

ArduPilot has a DDS client in-tree, `libraries/AP_DDS`, using the Micro XRCE-DDS client. The companion runs `micro_ros_agent`, not PX4's agent binary, typically:

```sh
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

Parameters from the AP_DDS readme: `DDS_ENABLE` (the readme's table says default 1 when DDS is compiled in), `SERIAL1_PROTOCOL = 45` for DDS on that port, `DDS_UDP_PORT` (examples use 2019), `DDS_DOMAIN_ID` must match `ROS_DOMAIN_ID`. Source: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md and https://ardupilot.org/dev/docs/ros2-sitl.html.

SITL with DDS, from that readme:

```sh
sim_vehicle.py -w -v ArduCopter --console -DG --enable-DDS
```

Services the current dev docs list:

| Service | Type |
|---|---|
| `/ap/arm_motors` | `ardupilot_msgs/srv/ArmMotors` |
| `/ap/mode_switch` | `ardupilot_msgs/srv/ModeSwitch` |
| `/ap/prearm_check` | `std_srvs/srv/Trigger` |
| `/ap/experimental/takeoff` | `ardupilot_msgs/srv/Takeoff` |
| `/ap/get_parameters`, `/ap/set_parameters` | `rcl_interfaces` parameter services |

Guided mode in the copter takeoff demo is mode **4**. Takeoff is under `experimental`. Source: https://ardupilot.org/dev/docs/ros2-interfaces.html and `Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/copter_takeoff.py` in the ArduPilot tree. Topics include `/ap/geopose/filtered` (`geographic_msgs/GeoPoseStamped`) with best-effort QoS in that demo. Publish rates are capped by `SCHED_LOOP_RATE` (the interfaces page says copter is typically 400 Hz and other vehicles 50 Hz, and that a topic's configured rate cannot exceed the scheduler).

Multi-vehicle namespacing landed in March 2026: `DDS_USE_NS=1` prefixes `/ap/v<sysid>/...`. Source: https://github.com/ArduPilot/ardupilot/pull/32510 (merged 2026-03-25). The PR's test checklist marks hardware as not tested.

`ArduPilot/ardupilot_ros` README still says requirements are ROS Humble, Gazebo Garden, and Cartographer. Source: https://github.com/ArduPilot/ardupilot_ros. That README is behind both Gazebo Harmonic and Copter 4.7. Treat it as a SLAM demo, not as the compatibility matrix.

### Firmware version

Newest stable tag confirmed on the ArduPilot releases page on 19 Oct 2026: **Copter-4.7.2**, "released as the stable/official version ... on 11-Oct-2026", GitHub publish stamp 14 Oct. Source: https://github.com/ArduPilot/ardupilot/releases. Copter-4.7.0 was 21 Jul 2026. Copter-4.6.2 was 18 Jul 2025. The Copter parameter and fence pages used below are the 4.6.2 stable docs where a 4.7.2 parameter page was not opened. Recheck `FENCE_*` names against the 4.7.2 parameter list before a MAVLink milestone writes them into a config file.

### Maturity against PX4

PX4's DDS bridge is the default companion API: generated from the same `msg/` tree as `px4_msgs`, wired into offboard mode, documented as the ROS 2 user guide, and covered by SITL on port 8888. ArduPilot's bridge is in the firmware and has arm, mode, prearm, and an experimental takeoff service, plus a geopose stream. It does not have PX4's `OffboardControlMode` plus `TrajectorySetpoint` contract. The takeoff service is named experimental. The ROS demo repo's README is stale. Hardware multi-vehicle DDS was not checked off in the namespace PR.

Use ArduPilot through **MAVLink** (milestone 4) before investing in `AP_DDS`. Revisit `AP_DDS` when a second native-DDS backend is worth more than one MAVLink backend that already talks to both autopilots.

Fence parameters to record now, from the Copter fence pages (4.6-era docs): `FENCE_ENABLE`, `FENCE_ACTION`, `FENCE_TYPE` (copter default bits include altitude max, home circle, and polygon), `FENCE_RADIUS` (cylindrical fence; the page says the radius must exceed `FENCE_MARGIN` and be greater than 30 m), `FENCE_ALT_MAX`, `FENCE_MARGIN`. Source: https://ardupilot.org/copter/docs/common-geofencing-landing-page.html and https://ardupilot.org/copter/docs/common-ac2_simple_geofence.html.

Gazebo for ArduPilot is the `ardupilot_gazebo` plugin, launched from `sim_vehicle.py` or from `ardupilot_gz`. A second CI job can wait until the MAVLink backend exists. This pass did not time an ArduPilot SITL container.

## 7. Closed vendor SDKs (not a milestone)

Mention only.

- **DJI.** Onboard SDK 4.x and Payload SDK 2.x were folded into Payload SDK 3.x. DJI's migration note says OSDK and PSDK are no longer updated separately, and PSDK 3.x is not compatible with OSDK expansion content. Aircraft ports differ by model (M300 still has an OSDK port; M30 and M3E expose an E-Port that only supports PSDK 3.x). Source: https://developer.dji.com/doc/payload-sdk-tutorial/en/faq.html. The SDK is proprietary, bound to DJI airframes, and a poor match for an Apache-2.0 robot runtime.
- **Parrot Olympe** and **Skydio** are closed, vehicle-specific SDKs. No current version was fetched for this note. They stay out of the issue breakdown until someone has a vehicle and a written SDK licence that allows the integration Ohho wants.
- **US procurement.** Blue UAS moved from DIU to DCMA under a 10 July 2025 memo. The old portal redirects to `https://bluelist.dcma.mil`. Source: https://www.diu.mil/blue-uas-portal. Being on a cleared list is not a blanket authority to operate. The American Security Drone Act's federal-funds restrictions are widely described as effective 22 Dec 2025; this note did not open the statute text. A 2026 Pillsbury note on the FCC Covered List says FY2025 NDAA section 1709 names DJI and Autel and that the FCC has been adding foreign-produced UAS and critical components to the Covered List. Source: https://www.pillsburylaw.com/print/v2/content/1065402/fcc-categorical-prohibition-foreign-produced-uas-critical-components.pdf. That is a US federal-procurement constraint. It is not a UK flying rule.
- **UK.** A UK university or a UK company buying a development kit is not automatically under the US NDAA. Import, radio (2.4 GHz / 433 / 915 MHz), and CAA operating rules still apply. Holybro's X500 kit is an open Pixhawk airframe, which avoids a vendor SDK, and it is still a product that may contain radios that need the right regional firmware (915 MHz vs 433 MHz). Do not describe it as "NDAA compliant" or "not compliant"; this pass did not audit Holybro's bill of materials against the Covered List.

Ohho should not take a dependency on DJI, Parrot, or Skydio for the first four milestones.

## 8. Simulation first

### PX4 SITL and Gazebo

PX4's current Gazebo is **Gazebo Harmonic**, model name `gz_x500`. Headless:

```sh
HEADLESS=1 make px4_sitl gz_x500
```

or, without compiling PX4:

```sh
docker run --rm -it --network host \
  -e HEADLESS=1 -e PX4_SIM_MODEL=gz_x500 \
  px4io/px4-sitl-gazebo:v1.17.0
```

`--network host` is the Linux form. Docker Desktop on Windows or macOS sends MAVLink to `host.docker.internal` instead. Source: https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages and https://docs.px4.io/main/en/sim_gazebo_gz/.

The prebuilt-package page lists approximate sizes: `px4io/px4-sitl` (SIH, headless, no Gazebo) about 100 MB, `px4io/px4-sitl-gazebo` about 2 GB. Tags follow PX4 versions (`v1.17.0`). Ports: 14550 QGroundControl, 14540 MAVSDK, **8888 uXRCE-DDS**, 19410 Hawkeye SIH visualisation.

Lockstep: Gazebo and PX4 step together. The GZBridge sets PX4 time on each sim step. Lockstep cannot be turned off. `PX4_SIM_SPEED_FACTOR=2` runs at twice real time until the machine runs out of CPU. The guide's informal note is roughly 6–10× on a strong desktop and 3–4× on a notebook. That is their observation, not a measurement on this PC. Source: https://docs.px4.io/main/en/sim_gazebo_gz/.

There is a ROS 2 image, `px4io/px4-sitl-gazebo-ros2`, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, agent **v2.4.3**, prebuilt `px4_msgs` matching the image, and the PX4 ROS 2 Interface Library. The Docker Hub text says `amd64` and `arm64`, headless by `HEADLESS=1`, and that PX4, Gazebo, and the agent talk inside the container. Pin a digest rather than `latest`. Source: https://hub.docker.com/r/px4io/px4-sitl-gazebo-ros2 (page said it had been updated 26 days before this research).

SIH (`px4io/px4-sitl`) is the right fallback if Gazebo is too heavy for a pull-request runner. It will not render a camera or a LiDAR. The square-flight CI test does not need either.

### GitHub-hosted runners

No Ohho job has run this. The following is a plan, not a measured runtime.

`ubuntu-latest` is a Linux VM without a GPU. `HEADLESS=1` is required. A 2 GB image pull dominates a cold job. Cache the image layer. A flight at 1× real time (arm, takeoff to 3 m, a small square, land) is on the order of one to two minutes of sim time if the solver keeps up, plus startup. A job timeout of 20 minutes is a budget to set, then tighten after the first green run. If cold pulls blow the budget, run Gazebo on a nightly schedule and run SIH on pull requests. Do not claim either number until a run exists.

### ArduPilot SITL

`sim_vehicle.py` from `Tools/autotest`, vehicle `-v ArduCopter`, `--enable-DDS` when the DDS path is under test. Gazebo integration is `ardupilot_gazebo` / `ardupilot_gz`. Not scheduled before the PX4 job is green.

### Windows and WSL2 on this PC

PX4 documents WSL2 as a supported environment for Ubuntu 22.04 and 24.04, including Gazebo SITL and ROS 2 with Gazebo. Source: https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl and https://docs.px4.io/main/en/dev_setup/dev_env.

Constraints they state:

- Clone and build on the Linux filesystem (`cd ~`), not under `/mnt/c`. Builds on the Windows mount are slow and hit permission errors.
- USB serial to a Pixhawk is not native. `usbipd-win` can attach a COM port. The guide says that from PX4 v1.18 the `upload` target can use the Windows Python interpreter from WSL. v1.18 is not the stable pin, so a v1.17 upload-from-WSL path was not confirmed here.
- QGroundControl for Linux inside WSL attaches to SITL on its own. QGroundControl for Windows needs a manual UDP connection to SITL in WSL.
- Gazebo's GUI wants WSLg. Headless does not.

This research did not install PX4, Gazebo, or ArduPilot, and did not start a container.

## 9. Crazyflie and Crazyswarm2

Crazyswarm2 is the ROS 2 port of Crazyswarm, MIT licensed, for Crazyflie 2.1, 2.1+, 2.1 Brushless, Flapper Nimble+, and Crazyflie Bolt. Source: https://imrclab.github.io/crazyswarm2/ and https://github.com/IMRCLab/crazyswarm2. The docs title still says "1.0a1". The GitHub README shows CI badges for Humble, Jazzy, Rolling, and Lyrical. No GitHub release version was fetched beyond those docs.

The server has three backends (`https://imrclab.github.io/crazyswarm2/overview.html`):

| | `cpp` (crazyflie-link-cpp) | `cflib` (crazyflie-lib-python) | `sim` |
|---|---|---|---|
| Parameters, default pose/scan/odom/status/imu logs | yes | yes | no |
| Broadcasts | yes | no | n/a |
| `cmd_full_state` | yes | yes | yes |
| `cmd_position`, `cmd_hover`, `cmd_velocity_world` | yes | yes | no |
| High-level takeoff, land, go-to, trajectories | yes | yes | yes |
| Motion capture, Flow deck, Lighthouse, LPS | yes | yes | no |
| Arming | yes | yes | n/a |

`cpp` is the one to target for a real indoor flight. `cflib` is the one that matches Ohho's Python runtime if we want a direct adapter without a ROS graph. `sim` is firmware-in-the-loop and, per the install page, needs the Crazyflie firmware tag **2025.02** built as Python bindings and put on `PYTHONPATH`. Source: https://imrclab.github.io/crazyswarm2/installation.html.

Launch form they document:

```sh
ros2 launch crazyflie launch.py backend:=cpp
ros2 launch crazyflie_examples launch.py script:=hello_world backend:=sim
```

Radio URI shape used across Bitcraze tools is `radio://0/80/2M/E7E7E7E7E7`. The `crazyflies.yaml` `type: cf21` block is where Flow deck versus Lighthouse is selected. The usage page says to turn motion-capture off for onboard positioning (`mocap:=False`) and points at Bitcraze's Lighthouse and Flow deck tutorials. Source: https://imrclab.github.io/crazyswarm2/usage.html.

The overview page warns that unicast packet loss is a known firmware bug. Do not describe Crazyswarm2 as a finished product.

Crazyflie emergency in that stack is a service that turns motors off. That is a motor kill, unlike PR #2's land-then-disarm policy. For a 30-gram aircraft in a net, cutting motors is the usual e-stop. For an X500 it is not. The Ohho e-stop mapping has to depend on the backend (section 6 of the plan).

### Prices (USD, Bitcraze store, 19 Oct 2026)

| Item | Price | URL |
|---|---|---|
| Crazyflie 2.1+ | $240 | https://store.bitcraze.io/products/crazyflie-2-1-plus |
| Crazyflie 2.1 Brushless | $480 | https://store.bitcraze.io/products/crazyflie-2-1-brushless |
| Getting started bundle, 2.1 Brushless | $500 (list $523 on the same collection page) | https://store.bitcraze.io/collections/bundles-crazyflie-2-1-brushless |
| Lighthouse explorer bundle, 2.1 Brushless | $1,050 (list $1,098) | same collection |
| Flow deck v2 | $55 | https://store.bitcraze.io/products/flow-deck-v2 |
| Crazyradio 2.0 | $43 | https://store.bitcraze.io/products/crazyradio-2-0 |

The 2.1+ page says 29 g and Bluetooth LE plus Crazyradio. The brushless page says about 10 minutes on the stock battery and 40 g recommended payload. A 2.1 (not "+") single-unit price was not on the pages opened; the 2.1+ is the current non-brushless board they sell.

## 10. Reference hardware and prices

Not purchased. Confirm on the day anyone actually orders. GBP is listed only where the page priced in GBP.

### Milestone 2 airframe: Holybro X500 V2 + Pixhawk 6C

| Item | Listed price | What the page says is in the box | URL |
|---|---|---|---|
| PX4 Development Kit X500 v2 | **$609** | Pixhawk 6C (plastic) or 6X, M10 GPS, SiK Telemetry Radio V3, X500 V2 frame, 2216 KV920 motors, 20 A BLHeli_S ESCs, PDB, 1045 props. Shipped with PX4. **Battery not included.** Companion mount for a Pi or a Jetson Nano. Stated hover about 18 min with a 5000 mAh pack and no extra payload. Stated frame weight 610 g. Payload claim 1500 g without battery at 70% throttle. | https://holybro.com/products/px4-development-kit-x500-v2 |
| X500 V2 ARF kit, no FC | $329 (collection page also shows "from $135", which is a different partial kit) | Frame, motors, ESCs, PDB, props. No FC, GPS, or radio. | https://holybro.com/products/x500-v2-kits |
| Pixhawk 6C | from $199 | Standalone | https://holybro.com/collections/pixhawk-6c-series |
| Pixhawk 6C Mini | from $179 | Standalone. The $609 kit's 6C option is the full-size plastic 6C, not the Mini. | same collection |
| SiK Telemetry Radio V3 | from $58.99 | Included in the $609 kit | https://holybro.com/collections/all-product |
| M10 GPS | $43.99 | Included in the $609 kit | same |
| M9N GPS | $54.99 | Not in the kit | same |

The $609 kit is the coherent M2 purchase. Buying a 6C Mini as well would duplicate the flight controller.

### RC and a hardware kill

PX4's kill switch is an RC switch mapped in QGroundControl. It stops motor outputs immediately. If the switch is returned within 5 seconds the motors can restart; after that the vehicle disarms. There is also a kill gesture that cannot be reverted. Source: https://docs.px4.io/main/en/config/safety section "Kill Switch".

A transmitter with a spring-loaded switch, bound to that function, is the hardware cut. Prices seen on RadioMaster's site:

| Item | Price | URL |
|---|---|---|
| TX15, ELRS, mode 2 | $153.99 | https://radiomasterrc.com/collections/new-arrivals |
| TX16S MK3, ELRS, mode 2 | $199.99 | same |
| RP1 V2 ExpressLRS 2.4 GHz receiver | $18.99 | same |
| ER16 Safety Switch Kit | $14.99 | same |

The ER16 kit is a RadioMaster accessory at that price. This pass did not open its manual, so it is **not** claimed to be a Pixhawk flight-termination input. The kill function is the PX4 RC switch above. Mode 2 matches `AerialDriveScheme`.

915 MHz versus 433 MHz matters in the UK. The Holybro SiK radio is listed as 433/915. Pick the legal ISM option for the UK before ordering. This note does not state which one Ofcom currently allows at which power.

### Companion computer

The X500 kit's mount is described for a Raspberry Pi or a Jetson Nano, not specifically an Orin Nano carrier. Check mechanical fit before buying an Orin dev kit for that plate.

| Item | Price found | Confidence |
|---|---|---|
| Raspberry Pi 5, 16 GB | **$305** on the official product page | Fetched 19 Oct 2026 from https://www.raspberrypi.com/products/raspberry-pi-5/. The page also lists 1, 2, 4, and 8 GB variants and says production until at least January 2036. Wi-Fi is dual-band 802.11ac. Power is 5 V / 5 A USB-C. |
| Raspberry Pi 5, 8 GB | $175 in a 15 Sep 2026 secondary article that says it is quoting Raspberry Pi's official price; £161.53 ex VAT at RS (stock 219-255) | The official product page fetch did **not** print the 8 GB dollar price. Do not treat $175 as confirmed from raspberrypi.com on 19 Oct 2026. RS: https://uk.rs-online.com/web/c/raspberry-pi-arduino-development-tools/raspberry-pi-shop/raspberry-pi/ |
| Jetson Orin Nano Super developer kit | $399 in that same 15 Sep 2026 article, described there as NVIDIA's listed price, up from a $249 launch price | Not re-fetched from nvidia.com in this session. Confirm before purchase. |

For this track a Pi 5 is the companion that matches `pi_workstation` in `ohho/profiles.py` and matches a ground-station link over Wi-Fi: the Pi runs the XRCE agent and Ohho, the aircraft's FC stays the Pixhawk, and a laptop runs QGroundControl plus `ohho sim-serve` or a rosbridge. A Jetson is the right board if onboard detection is actually in the loop. It is not required to arm, take off, and fly a square.

Pi 5 OS: the product page says Raspberry Pi OS Trixie and Bookworm work, and older than Bookworm does not. ROS 2 Jazzy's usual host is Ubuntu 24.04, not Raspberry Pi OS. Whether Jazzy on Pi OS, Ubuntu 24.04 on the Pi, or a Docker image is the companion install is an open question (section 13).

### Batteries and charger

The X500 page recommends a 4S 3000–5000 mAh 20C+ pack with an XT60, and says it is not included. No pack or charger SKU was priced in this pass. Leave a line in the bill of materials as price unknown rather than inventing one.

### Turnkey alternative: ModalAI

| Item | Listed price | Notes | URL |
|---|---|---|---|
| Starling 2 | from $2,949.99 | Collection page: 220 mm diagonal, 280 g, flight time stated as >40 min, QRB5165, PX4 and ArduPilot, Wi-Fi and ELRS | https://www.modalai.com/collections/drones |
| VOXL 2 Starling (older listing still on the site) | from $2,949.99, drone-only C6 example $2,949.99 | 210 mm, 275 g, >30 min, PX4 | https://www.modalai.com/products/starling-v1 |
| Starling 2 Max | $4,199.99 on the product page | 500 g payload claim, ~55 min on Amprius cells, GPS note: "performance issues with GPS on the current iteration ... an updated GPS is in development" | https://www.modalai.com/products/starling-2-max |
| VOXL 2 board | $1,399.99 | PX4 or ArduPilot on the sensors DSP, "ROS 1 and ROS 2", Ubuntu **18.04**, kernel 4.19, NDAA '20 section 848, assembled in the USA | https://www.modalai.com/products/voxl-2 |

A 22 Apr 2024 press release priced Starling 2 from $2,949.99 and Starling 2 Max from $2,999.99. The Max product page now shows $4,199.99. Use the product page, and treat the press release as history. Source: https://www.businesswire.com/news/home/20240422726107/en/ModalAI-Launches-Next-Generation-Starling-2-and-Starling-2-Max-NDAA-Compliant-Development-Drones.

VOXL 2 is the interesting turnkey case: PX4 is on the same board as the companion, so the "companion runs the XRCE agent" picture may be a socket on-board rather than a Pi on a UART. The published OS is Ubuntu 18.04. That is not Ubuntu 24.04 / Jazzy. Integrating Ohho there means their VOXL SDK userspace, not a stock ROS 2 Jazzy apt install. That is why it is an alternative, not M2.

### Optional inspection payloads

Not part of M2 or M3 exit criteria.

| Payload | What was verified | Price |
|---|---|---|
| Livox Mid-360 | 360° × 59° FOV, 40 m at 10% reflectivity, 10 cm blind zone, on the DJI store page | DJI store UK **£569**, out of stock, https://store.dji.com/mt/product/livox-mid-360. A US reseller listed **$1,350** and sold out, https://store.vertexunmanned.com/products/livox-mid-360. Two prices, both "not in stock". |
| Downward rangefinder | Needed for a honest height-above-ground in the e-stop path and for a vertical inspection offset. No SKU was priced. | Unknown |
| Global-shutter camera | Useful for motion on a turbine tower or any textured structure. No SKU was priced. A rolling-shutter Pi camera is a lab substitute and a poor inspection sensor. | Unknown |

The X500 page's 1500 g payload figure is the vendor's, at 70% throttle, battery not counted. A Mid-360 plus a Pi 5 plus a camera can eat a large part of a 5000 mAh 4S hover budget. The Holybro hover number (~18 min) is with no extra payload. Do not plan an inspection orbit duration from that number.

## 11. UK CAA, for the hardware milestone

Sources opened 19 Oct 2026:

- https://www.caa.co.uk/drones/open-category/drone-code/getting-what-you-need-to-fly-legally/
- https://www.caa.co.uk/drones/open-category/getting-started-with-drones-and-model-aircraft/registering-to-fly-drones-and-model-aircraft/
- https://www.caa.co.uk/drones/specific-category/remote-pilot-qualifications/a2-certificate-of-competency-a2-cofc/
- https://www.caa.co.uk/drones/open-category/moving-on-to-more-advanced-flying/introduction-to-more-advanced-flying/

The pages say the rules they describe apply from 1 January 2026.

Registration:

| Aircraft | Flyer ID | Operator ID |
|---|---|---|
| Under 100 g | Not required (page says recommended) | Optional |
| 100 g to under 250 g, no camera | Required | Optional |
| 100 g to under 250 g, with a camera | Required | Required |
| 250 g to under 25 kg, including UK1–UK4 | Required | Required |

Flyer ID is the theory test. Operator ID is the person responsible for the aircraft, must be 18 or over, and the ID is labelled on the aircraft. Flying without the required ID is an offence on the CAA's own page.

An X500 frame at 610 g plus battery, flight controller, and companion is in the 250 g–25 kg band. Both IDs are required before any outdoor flight. A Crazyflie 2.1+ at 29 g is under 100 g **if it has no camera**. A Crazyflie with a camera deck may cross the 100 g camera rule even when it stays under 250 g. Weigh the aircraft as it will fly, including battery and decks, and do not rely on the bare-board figure.

Open category is the low-risk category. Sub-categories A1 / A2 / A3 are now described as over people / near people / far from people. Flyer ID covers the basic Open privileges. Flying in the near-people (A2) subcategory needs an **A2 Certificate of Competency** as well as both IDs. The A2 CofC page: self-directed practical flying, a closed-book theory test of at least 30 questions, entry condition is a Flyer ID, valid five years. The AMC says the test is at least 75 minutes and the pass mark is at least 75%, taken at a recognised assessment entity.

New models placed on the UK market from 1 January 2026 need a UK class mark (UK0–UK6). Aircraft bought before that date can still be flown on the weight rules. A C-class mark can be treated as the matching UK class until 31 December 2027. A Holybro kit or a ModalAI board may or may not carry a UK class mark. This pass did not inspect a unit. Outdoor milestone text should say: read the current Drone Code for the actual aircraft, do not assume a class mark, and do not fly outdoors on the strength of this document.

A first outdoor flight for this track, if it happens, belongs in Open category, far from people, within visual line of sight, with a pilot who has a Flyer ID and an Operator ID on the aircraft. Inspecting a wind turbine, flying near uninvolved people, or flying beyond visual line of sight is not that flight. It is likely a Specific-category operation with an operational authorisation. That work is out of scope for these milestones. The geofence and the orbit controller are still the right software, and they do not authorise the flight.

## 12. How Ohho would connect

Proposed URIs. None of these schemes exist in `ohho/adapters/__init__.py` today.

### PX4 over uXRCE-DDS (primary)

| | |
|---|---|
| URI | `px4+dds://127.0.0.1:8888` in SITL. On a companion, `px4+dds://serial0` or `px4+dds://<fc-ip>:8888`, query `ns=` for the ROS namespace. Exact serial form is an implementation choice; the agent is still a separate process. |
| Processes | Pixhawk (or SITL) runs `uxrce_dds_client`. Companion or CI container runs `MicroXRCEAgent udp4 -p 8888`. Ohho uses `Ros2Runtime`'s rclpy node in that DDS domain. One ROS domain, one agent. |
| Arm | `VehicleCommand` arm, only after pre-arm checks pass and `OffboardControlMode` is already streaming. This is not `SafetyGate.arm()`. |
| Offboard position | Publish `OffboardControlMode.position = true` at ≥10 Hz and `TrajectorySetpoint.position` in local NED, yaw in NED radians. Other flags false. Unused fields NaN. |
| Offboard velocity | `OffboardControlMode.velocity = true`. `TrajectorySetpoint.velocity` in the frame the method names: `LOCAL_NED` or `BODY_FRD`. Ohho's public arguments stay SI and documented. ENU is accepted only at the `Robot` edge and converted before publish. |
| Telemetry | Subscribe `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_status`, battery and GPS topics from `dds_topics.yaml` for the pinned firmware. Map into `Telemetry.custom["drone"]` using the PR #2 schema, with z documented as NED down **or** converted once at the boundary. Do not store both conventions in the same field. |
| Failsafe owner | PX4 owns offboard loss (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`), RC loss (`COM_RC_LOSS_T`, `COM_FAIL_ACT_T`), geofence (`GF_ACTION`, `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`), battery, and the RC kill switch. Ohho owns the software latch, the setpoint clamp, and the decision to stop streaming. Stopping the stream **is** how Ohho hands the aircraft back. |
| Latency | UART-to-agent plus DDS in the same machine is typically a few milliseconds; a Wi-Fi ground station in the loop is tens of milliseconds and is not the offboard link. Offboard setpoints should be generated on the companion, not on a laptop across Wi-Fi. |

### MAVLink (PX4 and ArduPilot)

| | |
|---|---|
| URI | `mavlink+udp://127.0.0.1:14540` for SITL. `mavlink+serial:///dev/ttyAMA0?baud=921600` for a Pi UART. `mavlink+tcp://...` only if a specific radio requires it. |
| Processes | Native `mavsdk` 4.x inside the Ohho process. No `mavsdk_server`. QGroundControl may listen on UDP 14550 at the same time. If `mavsdk-grpc` is ever used, `mavsdk_server` is a supervised child and Ohho is its client. |
| Arm | MAVSDK action arm, after pre-arm. Same human gate as above. |
| Offboard | MAVSDK offboard plugin: start a setpoint stream, then set offboard mode. Velocity frame is an explicit argument (`BODY_FRD` matches `MAV_FRAME_BODY_NED`; `LOCAL_NED` matches local NED). ArduPilot guided velocity is the same Ohho call with a different plugin path inside this backend. |
| Telemetry | Position, velocity, attitude, battery, GPS fix, flight mode, armed, failsafe flags, link quality from the MAVSDK telemetry plugin (or pymavlink `SYS_STATUS` / `RADIO_STATUS` if a field is missing). |
| Failsafe owner | Same split. Ohho e-stop sends land, or hold-then-land, per PR #2. It does not send force-disarm and it does not send flight termination. The RC kill switch does the motor cut, and it is the pilot's hand, not an Ohho API. |
| Latency | Direct UDP on localhost is the SITL case (sub-millisecond to a couple of milliseconds). A SiK radio at 57600 baud is a slow telemetry pipe and a bad offboard pipe. Offboard on hardware should be the UART or Ethernet to the companion, not the SiK link. |

### Crazyflie

| | |
|---|---|
| URI | `crazyflie://radio://0/80/2M/E7E7E7E7E7` for a direct cflib backend. `crazyflie+ros2://cf21` when Crazyswarm2 is already running. |
| Processes | Either Ohho calls cflib in-process, or the Crazyswarm2 server (`backend:=cpp` or `cflib`) owns the radio and Ohho is a ROS 2 client of `crazyflie_py`. Do not open the radio from both. |
| Arm | Crazyflie arming where the firmware requires it (`cpp` and `cflib` rows in their table; sim is n/a). Still behind the Ohho hardware phrase when the URI is a radio rather than `backend:=sim`. |
| Setpoints | High-level takeoff / go-to / land for scripts. `cmd_velocity_world` or `cmd_hover` for sticks, on the `cpp` or `cflib` backend. The `sim` backend's table says it does **not** implement `cmd_velocity_world`. A teleop test against `backend:=sim` cannot assume velocity setpoints work. |
| Telemetry | Pose from the state estimate (Flow deck, Lighthouse, or motion capture). Battery and link quality from the cflib logs. GPS fix is absent. |
| Failsafe owner | The Crazyflie firmware owns radio-loss hover/land behaviour (Bitcraze's default is a commander timeout; confirm the parameter on the firmware tag we pin). Ohho e-stop on this backend calls the Crazyswarm2 emergency service, which cuts motors. That is intentional for a netted 30 g aircraft and is a different mapping than the X500. |
| Latency | Crazyradio is designed for low-latency sticks. A ROS 2 hop adds a little. Keep the radio server on the machine with the dongle. |

### In-process sim

| | |
|---|---|
| URI | `sim://` with a drone robot spec, same scheme as `omnibot`. No new scheme is required for the kinematic stub. |
| Processes | `SimTransport` inside `ohho sim-serve`. No agent, no MAVSDK. |
| Arm / takeoff | Methods update the stub state. `Robot.connect` does not wrap `sim://` in `SafetyGate`, which is the current code. The drone sim should still refuse takeoff until a sim-level armed flag is set, so tests exercise the same call order as hardware. That flag is not `OHHO_ARM_HARDWARE`. |
| Setpoints | Integrate velocity in a named frame at the sim step (the existing `SimTransport.step`). |
| Telemetry | The same `Telemetry.custom["drone"]` dict the hardware backends fill. |
| Failsafe owner | The sim applies the software fence and a setpoint timeout. It does not pretend to be PX4's `GF_ACTION`. |

## 13. Not verified, or only partly verified

- Default numeric values of `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, `COM_RC_LOSS_T`, and battery failsafe actions. Names are documented; defaults were not copied into this note.
- Whether `UXRCE_DDS_CLIENT_USE_DDS_V3` exists on the v1.17.0 tag. The table that names it is on the `main` guide and is labeled PX4 v1.18.
- A fresh `apt install ros-jazzy-px4-msgs` version string. The docs say the Debian package appears after the build farm release and still recommend a source build.
- Plugin-by-plugin parity of native `mavsdk` 4.0.6 versus `mavsdk-grpc` 4.0.5.
- The exact MAVSDK 4 connection-string grammar.
- ArduPilot 4.7.2 `FENCE_*` defaults versus the 4.6.2 pages cited.
- A newer Crazyflie firmware tag than the Crazyswarm2 install page's `2025.02`.
- Raspberry Pi 5 8 GB official USD price on 19 Oct 2026 (not printed in the fetched product page). Jetson Orin Nano Super price (secondary article only).
- Any 4S battery or charger price.
- Ofcom limits for 433 MHz vs 915 MHz telemetry.
- Whether a Holybro X500 kit carries a UK class mark.
- ModalAI's actual ROS 2 distro inside VOXL SDK, beyond the product page saying "ROS 2" and Ubuntu 18.04.
- End-to-end timing of `px4io/px4-sitl-gazebo:v1.17.0` on `ubuntu-latest`.
- That no open ohho-os pull request touches PX4. The task brief said that was already checked on 19 Oct 2026. This session did not re-list every open ohho-os PR.
- Wind-turbine airspace, NOTAM, and site permission. Out of scope, and not researched.

## 14. Inspection mapping, without turning this into a product

A turbine inspection aircraft is still a multicopter with a companion, a camera, often a LiDAR, a geofence around a structure, and a ground station. The pieces that transfer are:

- Companion on the aircraft (Pi 5 or a Jetson), autopilot on a Pixhawk or a VOXL DSP, link between them a UART or UDP, not a laptop in the control loop.
- Offboard position setpoints are how an orbit is flown. The orbit planner is not milestone 1.
- A software fence plus `GF_*` / `FENCE_*` is how "do not leave the turbine pad" is enforced. PX4's own note says the failsafe fires **after** the breach, so the radius needs stopping distance (`v^2 / 2a`) as margin. Source: https://docs.px4.io/main/en/config/safety.
- Video to a ground station over Wi-Fi is a payload stream. It is not the offboard link. Losing Wi-Fi must not be the thing that lands the aircraft if the companion is still alive; losing the companion stream must.
- LiDAR (Mid-360 class) and a global-shutter camera are optional payloads. They are how you would do close visual work. They are not required to prove the backend.

Nothing in Ohho flies an orbit today. Saying the stack "supports inspection" would be a false capability claim.
