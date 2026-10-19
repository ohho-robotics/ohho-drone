# ArduPilot and PX4, for the MAVLink square (OHH-306)

Copter firmware pin: **Copter-4.7.2**, commit `fb773567896014da007bca58358704d4a0d8516f`. That tag was the newest `Copter-4.7.*` tag on 19 Oct 2026 (`Copter-4.7.0`, `Copter-4.7.1`, `Copter-4.7.2`). PX4 names below are from the **v1.17.0** tag, which is the SITL image pin in the drone plan.

Parameter names were read from the Copter-4.7.2 tree (`libraries/AC_Fence/AC_Fence.cpp`, `libraries/AP_Arming/AP_Arming.cpp`, `ArduCopter/Parameters.cpp`, `ArduCopter/mode_rtl.cpp`) and checked against the [Copter stable V4.7.0 parameter list](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html). That page is the 4.7.x list; it carries the same names as the tag, including the 4.7 renames. PX4 parameters were read from the v1.17.0 sources and linked on the [v1.17 parameter reference](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html).

The square in `.github/workflows/sitl-ardupilot.yml` was **not** flown while this page was written. No ArduPilot process, Docker image, or MAVLink socket was started in that worktree. The workflow's 90 minute limit is a cap for a cold `Copter-4.7.2` build plus the flight at `--speedup=1`. The first Actions run writes `square-result.json` with `elapsed_s`. That file is the measurement.

Ohho install in CI, until [ohho-os PR #62](https://github.com/ohho-robotics/ohho-os/pull/62) merges:

`ohho-os[mavlink] @ git+https://github.com/ohho-robotics/ohho-os@ohh-305-mavlink-backend`

That is branch `ohh-305-mavlink-backend`. The project name on that branch is `ohho-os`, and the optional extra in its pyproject is `mavlink`. pip discards a requirement named `ohho` because the built metadata name is `ohho-os`. pip 25.1 also rejects a `#egg=` fragment that is not a plain project name, so CI uses the name @ url form.

After that merge, install `ohho-os[mavlink]` from `main`. CI leaves `OHHO_ARM_HARDWARE` unset. The SITL URI is `mavlink+udp://127.0.0.1:14540` (`udpin://127.0.0.1:14540`).

## Square

Same criteria as D-07, through the Ohho MAVLink backend:

1. Request `SYS_STATUS`, `EKF_STATUS_REPORT`, and `GPS_RAW_INT` on the open link. Copter's `SR0_*` rates are 0 Hz and this job starts `sim_vehicle.py` with `--no-mavproxy`, so nothing else asks for those streams. Wait until the `SYS_STATUS` pre-arm bit is present, enabled, and healthy, the `EKF_STATUS_REPORT` flags match an absolute-position ready-to-arm, and `GPS_RAW_INT.fix_type` is at least 3. The wait is bounded at 90 s and prints each check that is still failing. A rejected arm is retried a few times. Copter arming checks stay at their defaults.
2. Climb to 2.5 m. Hold while `|z_ned + 2.5| < 0.4` m for 2 s.
3. Fly a square of 4 m sides in `LOCAL_NED` at 0.5 m/s, altitude held. Each corner within 0.8 m horizontal.
4. Land. Pass when `landed_state` is `on_ground`, the motors are disarmed with a normal disarm, and no failsafe flag was set after takeoff.

`scripts/ardupilot_sitl_square.py --fly` is that sequence. With no arguments the script prints the recipe and exits 0. The headless SITL launch is `sim_vehicle.py -v ArduCopter -w --speedup=1 --no-mavproxy --no-rebuild`, with serial0 `udpclient:127.0.0.1:14540` so the vehicle sends to the port Ohho listens on. The runner has no `DISPLAY`, so `run_in_terminal_window.sh` writes `/tmp/ArduCopter.log`.

The square goes through `Robot.connect` and the aerial `SafetyGate`. Under GitHub Actions the fly step sets `OHHO_CI_SIM_ARMING=1` and `OHHO_CI_SITL=1`, and the workflow leaves `OHHO_ARM_HARDWARE` unset. That opt-in is the ohho-os sim-only arming exception for loopback `mavlink+udp://`. The gate still enforces its allowlist, the 10 m / 3 m cylinder, the 3 m takeoff cap, speed caps, the battery and link takeoff block, a setpoint in the last 300 ms before `arm_flight`, the e-stop latch, and the watchdogs. This repo does not raise those limits. Takeoff is 2.5 m and the 4 m square (about 5.7 m from the origin at the far corner) fits inside them. `arm_flight` is `MAV_CMD_COMPONENT_ARM_DISARM` with param2 0.

## Modes

External setpoints are [ArduPilot GUIDED](https://ardupilot.org/copter/docs/ac2_guidedmode.html) and [PX4 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard.html). The Copter mode list is the [flight modes page](https://ardupilot.org/copter/docs/flight-modes.html). The `AP_DDS` README on this tag switches mode with `mode: 4`, which is GUIDED.

| | ArduPilot Copter 4.7.2 | PX4 v1.17.0 |
|---|---|---|
| External position and velocity | GUIDED | Offboard |
| Ohho call | `offboard_position` / `offboard_velocity` in `LOCAL_NED` | same Ohho call |
| What the backend sends | MAVSDK `set_position_ned`, then `offboard.start()` | MAVSDK offboard start enters Offboard |
| Loiter / hold | LOITER. Ohho `hold()` is MAVSDK `hold` | Hold (`AUTO.LOITER`). Same Ohho call |
| Land | LAND. Ohho `land()` is MAVSDK `land` | Land. Same Ohho call |
| Return | [RTL](https://ardupilot.org/copter/docs/rtl-mode.html). Ohho `rtl()` is MAVSDK `return_to_launch` | Return. Same Ohho call |

The aerial gate is armed before the autopilot. A ground-hold offboard stream at 10 Hz runs before and during `arm_flight`, because the gate refuses that arm without a setpoint in the last 300 ms. That stream is MAVSDK offboard, which is Copter GUIDED. The autopilot arm counts only after the HEARTBEAT armed bit is visible; Copter's heartbeat is 1 Hz. Takeoff then stops the position stream and calls MAVSDK `takeoff` (`MAV_CMD_NAV_TAKEOFF`). MAVSDK enters GUIDED when the vehicle is not already in its Offboard mode, and the altitude is metres above home. A position target left running at z=0 would leave GUIDED's takeoff and the motors would stay in ground idle. The carrot stream walks each side at 0.5 m/s, 10 Hz, in `LOCAL_NED` after the climb.

Offboard loss on PX4 waits [`COM_OF_LOSS_T`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_OF_LOSS_T) (default 1.0 s) and then does [`COM_OBL_RC_ACT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_OBL_RC_ACT). Default 0 is Position mode. The value list also contains Terminate (6) and Disarm (7). Those two stay unused. Ohho keeps streaming at 10 Hz for the whole square so the timer does not expire.

On Copter, RC loss while flying is [`FS_THR_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-thr-enable-throttle-failsafe-enable). The tag default is `FS_THR_ENABLED_ALWAYS_RTL` (value 1, always RTL). [`FS_OPTIONS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-options-failsafe-options-bitmask) bit 2 is "continue if in Guided on RC failsafe". The tag default enumerator is `GCS_CONTINUE_IF_PILOT_CONTROL` (bit 4, GCS failsafe). CI does not write `FS_OPTIONS`.

## Arming checks

Copter 4.7 replaced `ARMING_CHECK` with [`ARMING_SKIPCHK`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#arming-skipchk-arm-checks-to-skip-bitmask). The source comment calls index 8 the old `CHECK` value and records a 4.7 conversion into `SKIPCHK`. Default 0 skips nothing. `-1` skips all non-mandatory checks. CI does not write `ARMING_SKIPCHK`.

| Parameter | Role on Copter-4.7.2 |
|---|---|
| [`ARMING_SKIPCHK`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#arming-skipchk-arm-checks-to-skip-bitmask) | Bitmask of checks to skip. 0 runs the checks. Bits include barometer, compass, GPS lock, INS, parameters, RC, board voltage, battery, logging, safety switch, GPS configuration, system. |
| [`ARMING_NEED_LOC`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#arming-need-loc-require-vehicle-location) | 1 requires an absolute position before arming, so RTL has a home. |
| [`ARMING_OPTIONS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#arming-options-arming-options) | Display and ICE-motor options. It does not skip the pre-arm checks. |
| [`ARMING_RUDDER`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#arming-rudder-arming-with-rudder-enable-disable) | Stick arming. SITL arming in this job is the MAVLink arm command. |

The Copter procedure is [Arming the motors](https://ardupilot.org/copter/docs/arming_the_motors.html). PX4 pre-arm and arm checks are [Prearm, Arm, Disarm Configuration](https://docs.px4.io/v1.17/en/advanced_config/prearm_arm_disarm) and the [safety page](https://docs.px4.io/v1.17/en/config/safety). Ohho `arm()` arms the safety gate. This CI job calls it under the sim-only opt-in, then streams a setpoint and calls `arm_flight`. It does not set `OHHO_ARM_HARDWARE` and it does not type the hardware phrase.

## Fence

Copter fence guide: [Fences](https://ardupilot.org/copter/docs/common-geofencing-landing-page.html) and [Cylindrical Fence](https://ardupilot.org/copter/docs/common-ac2_simple_geofence.html). The cylindrical page says the radius must be larger than `FENCE_MARGIN` and greater than 30 m. The tag's `@Range` for `FENCE_RADIUS` is 30 to 10000, so the parameter accepts 30. A Copter radius used for a real flight should be above 30 m, which satisfies the page and sits inside that range. The tag default is 300 m. The 4 m square is inside that cylinder. CI does not write these parameters.

`FENCE_ENABLE` default in the tag is 0. The type bitmask can be configured while the fence is disabled. Copter's default `FENCE_TYPE` is max altitude, home circle, and polygon (bits 0, 1, and 2).

| Parameter | Copter-4.7.2 | PX4 v1.17.0 |
|---|---|---|
| Enable | [`FENCE_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-enable-fence-enable-disable) default 0. 1 enables the configured fences. | Simple distance limits are off when the max distances are 0. |
| Which fence | [`FENCE_TYPE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-type-fence-type) bitmask. Bit 0 max altitude, bit 1 circle on home, bit 2 inclusion or exclusion, bit 3 min altitude. | One horizontal and one vertical distance from home. |
| Horizontal size | [`FENCE_RADIUS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-radius-circular-fence-radius) metres, range 30 to 10000, default 300. Must exceed `FENCE_MARGIN`. The Copter page requires a value greater than 30 m. | [`GF_MAX_HOR_DIST`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#GF_MAX_HOR_DIST) metres, default 0 (disabled), max 10000. |
| Vertical size | [`FENCE_ALT_MAX`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-alt-max-fence-maximum-altitude) metres, range 10 to 1000, default 100. Frame is [`FENCE_ALT_MAX_TP`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-alt-max-tp-altitude-max-frame-type). | [`GF_MAX_VER_DIST`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#GF_MAX_VER_DIST) metres, default 0 (disabled). |
| Margin | [`FENCE_MARGIN`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-margin-fence-margin) metres, range 1 to 10, default 2. [`FENCE_MARGIN_XY`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-margin-xy-fence-horizontal-margin) is the horizontal margin, range 0 to 50, default 0, and 0 means use `FENCE_MARGIN`. `FENCE_MARGIN_XY` is present on this tag. | The action runs for a breach of the distance limits. The plan's stopping-distance margin is how far inside `GF_MAX_HOR_DIST` the Ohho software fence should sit. |
| Breach action | [`FENCE_ACTION`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-action-fence-action) default 1, RTL or Land. | [`GF_ACTION`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#GF_ACTION) default 2, Hold. 4 is Terminate. Leave 4 unused. |

[`FENCE_ALT_MIN`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fence-alt-min-fence-minimum-altitude) is the floor. It arms once the vehicle has reached that altitude. CI does not enable it.

PX4 geofence actions are described on the [safety page](https://docs.px4.io/v1.17/en/config/safety). `GF_ACTION` Hold matches the plan's first-flight action. `GF_MAX_HOR_DIST` and `GF_MAX_VER_DIST` stay at 0 until a person sets them for a real flight. This workflow does not disable a fence to make the square pass, and it does not turn a fence on with a radius inside the documented Copter minimum.

Ohho's own cylinder (default sim radius 10 m, max altitude 3 m) is the aerial `SafetyGate` profile on this path. The square is 4 m on a side around the takeoff point and 2.5 m up, inside both the default 300 m Copter cylinder and that 10 m Ohho cylinder, and under the 3 m takeoff cap.

## RTL

| | ArduPilot Copter 4.7.2 | PX4 v1.17.0 |
|---|---|---|
| Mode | RTL. Ohho `rtl()` requests it and does not fly the path itself. | Return. Same Ohho call. |
| Return altitude | [`RTL_ALT_M`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-alt-m-rtl-altitude) metres, range 0.30 to 3000. Minimum height above home before returning. The 4.7 source name is `RTL_ALT_M` (the subgroup `RTL_` plus `ALT_M`). | [`RTL_RETURN_ALT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_RETURN_ALT) default 60 m. |
| Final altitude | [`RTL_ALT_FINAL_M`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-alt-final-m-rtl-final-altitude) metres, range 0 to 10. 0 lands. | [`RTL_DESCEND_ALT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_DESCEND_ALT) default 30 m, then [`RTL_LAND_DELAY`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_LAND_DELAY). |
| Climb | [`RTL_CLIMB_MIN_M`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-climb-min-m-rtl-minimum-climb) metres, range 0 to 30. | The return altitude and [`RTL_CONE_ANG`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_CONE_ANG) limit the climb. |
| Speed | [`RTL_SPEED_MS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-speed-ms-rtl-speed) m/s, range 0 to 20. 0 uses the waypoint speed. | Return uses the navigator RTL profile. |
| Loiter before descent | [`RTL_LOIT_TIME`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-loit-time-rtl-loiter-time) milliseconds. | [`RTL_LAND_DELAY`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_LAND_DELAY) seconds. |
| Cone | [`RTL_CONE_SLOPE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-cone-slope-rtl-cone-slope) | [`RTL_CONE_ANG`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_CONE_ANG) |
| Altitude frame | [`RTL_ALT_TYPE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-alt-type-rtl-mode-altitude-type) 0 home-relative, 1 terrain. | Home altitude, with the cone above the destination. |
| Options | [`RTL_OPTIONS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#rtl-options-rtl-mode-options) bit 2 ignores pilot yaw. | [`RTL_TYPE`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RTL_TYPE) selects the return path. |

Home has to be set and checked before an outdoor flight. The square does not command RTL. A fence breach with the Copter default `FENCE_ACTION` 1 requests RTL or Land. PX4 RC loss default [`NAV_RCL_ACT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#NAV_RCL_ACT) is 2, Return, after [`COM_RC_LOSS_T`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_RC_LOSS_T) (default 0.5 s).

## Failsafes

Ohho leaves these at the firmware defaults. The square does not clear them to make the demo pass.

| Event | ArduPilot Copter 4.7.2 | PX4 v1.17.0 |
|---|---|---|
| RC / manual loss | [`FS_THR_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-thr-enable-throttle-failsafe-enable) default always RTL. Threshold [`FS_THR_VALUE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-thr-value-throttle-failsafe-value) PWM. Guide: [Radio Failsafe](https://ardupilot.org/copter/docs/radio-failsafe.html). | [`COM_RC_LOSS_T`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_RC_LOSS_T) then [`NAV_RCL_ACT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#NAV_RCL_ACT) default Return. |
| GCS / datalink loss | [`FS_GCS_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-gcs-enable-ground-station-failsafe-enable) default disabled. Timeout [`FS_GCS_TIMEOUT`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-gcs-timeout-gcs-failsafe-timeout) default 5 s, range 2 to 120. Guide: [GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html). | [`COM_DL_LOSS_T`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_DL_LOSS_T) then [`NAV_DLL_ACT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#NAV_DLL_ACT) default 0, disabled. |
| Extra options | [`FS_OPTIONS`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-options-failsafe-options-bitmask) | Commander actions above. |
| EKF | [`FS_EKF_ACTION`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-ekf-action-ekf-failsafe-action), [`FS_EKF_THRESH`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-ekf-thresh-ekf-failsafe-variance-threshold), [`FS_EKF_FILT`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-ekf-filt-ekf-failsafe-filter-cutoff) | Estimator checks on the [pre-arm page](https://docs.px4.io/v1.17/en/advanced_config/prearm_arm_disarm). |
| Dead reckoning | [`FS_DR_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-dr-enable-deadreckon-failsafe-action) default RTL, [`FS_DR_TIMEOUT`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-dr-timeout-deadreckon-failsafe-timeout) default 30 s. | Loss of position follows the mode and the RC or datalink action. |
| Vibration | [`FS_VIBE_ENABLE`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-vibe-enable-vibration-failsafe-enable) | — |
| Crash | [`FS_CRASH_CHECK`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#fs-crash-check-crash-check-enable) default 1. A detected crash disarms. | Failure-detector actions in Commander. |
| Battery | [`BATT_FS_LOW_ACT`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#batt-fs-low-act-low-battery-failsafe-action), [`BATT_FS_CRT_ACT`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#batt-fs-crt-act-critical-battery-failsafe-action) | [`COM_LOW_BAT_ACT`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_LOW_BAT_ACT) default 0, Warning. |
| Fence | `FENCE_ACTION` above | `GF_ACTION` above |
| Offboard / GUIDED loss | GUIDED setpoints stop being renewed. RC loss still follows `FS_THR_ENABLE` unless `FS_OPTIONS` bit 2 is set. | `COM_OF_LOSS_T` then `COM_OBL_RC_ACT` |

[`DISARM_DELAY`](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.7.0.html#disarm-delay-disarm-delay) is how long Copter waits, while landed with the motors in ground idle, before it disarms itself. The default is 10 s. A takeoff that never spools the motors hits that timer. The square waits until `landed_state` is `on_ground` and positive-up height is at most 0.25 m, then sends one normal disarm if the motors are still armed.

## E-stop

Three layers. They stay separate.

| Layer | ArduPilot | PX4 | Ohho |
|---|---|---|---|
| Ohho `emergency_stop` | Land when `landed_state` is in air, takeoff, or landing. Hold, then land, when landed state is unknown. Disarm only when `landed_state` is `on_ground` and positive-up height is at most 0.25 m. | Same decision. | Disarm is `MAV_CMD_COMPONENT_ARM_DISARM` param1 0, param2 0. The backend refuses param2 21196 and refuses flight termination. `release_stop` clears the Ohho latch only. It does not arm and it does not resume GUIDED or Offboard. |
| Autopilot failsafe | `FENCE_*` and `FS_*` above. They run when the Ohho process is dead. | `GF_*`, `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, `COM_RC_LOSS_T`, `NAV_RCL_ACT`, `COM_LOW_BAT_ACT`. | Ohho does not clear these to finish a demo. |
| Pilot kill | An RC option value 31, Motor Emergency Stop, on `RCn_OPTION`. [Auxiliary functions](https://ardupilot.org/copter/docs/common-auxiliary-functions.html). Motor outputs stop. | Kill switch mapped by [`RC_MAP_KILL_SW`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#RC_MAP_KILL_SW). [`COM_KILL_DISARM`](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference.html#COM_KILL_DISARM) default 5 s disarms after the switch is held. [Kill switch](https://docs.px4.io/v1.17/en/config/safety). | The pilot's hand. Ohho does not expose this to MCP. |

## Later optional issue: AP_DDS

Not built here. The workflow does not pass `--enable-DDS` and does not start `micro_ros_agent`.

On Copter-4.7.2, [`libraries/AP_DDS/README.md`](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/libraries/AP_DDS/README.md) describes an in-tree DDS client. `DDS_ENABLE` defaults to 1 when DDS is compiled in. The SITL example is `sim_vehicle.py -w -v ArduPlane --console -DG --enable-DDS`. UDP uses `micro_ros_agent udp4 -p 2019`. Services listed there:

| Service | Type |
|---|---|
| `/ap/arm_motors` | `ardupilot_msgs/srv/ArmMotors` |
| `/ap/mode_switch` | `ardupilot_msgs/srv/ModeSwitch` |
| `/ap/prearm_check` | `std_srvs/srv/Trigger` |
| `/ap/experimental/takeoff` | `ardupilot_msgs/srv/Takeoff` |

Takeoff is under `experimental`. The mode example uses mode 4 (GUIDED). The interface list is [ROS 2 interfaces](https://ardupilot.org/dev/docs/ros2-interfaces.html). This is not PX4's `OffboardControlMode` plus `TrajectorySetpoint` contract.

The [ardupilot_ros README](https://github.com/ArduPilot/ardupilot_ros/blob/master/README.md) still requires ROS Humble and Gazebo Garden. That is behind Gazebo Harmonic and behind this Copter pin.

Use the MAVLink square in this workflow first. An optional follow-up can revisit `AP_DDS` when a second native DDS backend is worth more than one MAVLink backend that already talks to both autopilots.
