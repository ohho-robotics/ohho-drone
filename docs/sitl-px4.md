# PX4 SITL square

Headless PX4 **v1.17.0** and Gazebo Harmonic `gz_x500`, flown through Ohho
`px4+dds://127.0.0.1:8888` and the aerial SafetyGate. The mission is arm,
climb to 2.5 m, a 4 m square, and land.

This repository does not claim that a flight has happened. The square was not
run on the machine that added this recipe (19 October 2026). The workflow
records the runtime on its first GitHub Actions run.

## Pins

| Piece | Pin |
|---|---|
| Dev image | `px4io/px4-dev:v1.17.0` |
| Image index | `sha256:288dc1d7f5290855e86dbaf8b8f55cd17f46f27e9df2affb2056df6363bc6fb0` |
| linux/amd64 | `sha256:82e3ddcf4bf3fa4dbc0be6489c6de4158d0efbc8530182c691184302027335ce` |
| PX4 source | git tag `v1.17.0` |
| Gazebo | Harmonic (`gz-harmonic`), model `gz_x500`, world `default` |
| Speed | `PX4_SIM_SPEED_FACTOR=1` |
| Agent | Micro XRCE-DDS Agent `v2.4.3`, `udp4 -p 8888` |
| Messages | `px4_msgs` branch `release/1.17` |
| ROS | Jazzy, installed in the container |
| Ohho | `git+https://github.com/ohho-robotics/ohho-os.git@ohh-290-px4-dds` (`5acb80e`) |
| GCS heartbeat | pymavlink `2.4.50`, bind `127.0.0.1:14550`, send to PX4 port `18570` |

`px4io/px4-sitl-gazebo:v1.17.0` and `px4io/px4-sitl-gazebo-ros2:v1.17.0` were
not on Docker Hub on 19 October 2026. `px4io/px4-sitl-gazebo-ros2:latest` is
`v1.18.0-beta1-660-g61907962c9`. ohho-os refuses a v1.18 message definition,
so the workflow does not pull `latest`. The dev image above was published with
the v1.17.0 release. Its Dockerfile runs `Tools/setup/ubuntu.sh --no-sim-tools`
on Ubuntu 24.04, so it does not contain Gazebo, OpenCV, or the SITL binary.
`scripts/run_sitl_px4.sh` installs Gazebo Harmonic, then the v1.17.0
`ubuntu.sh` packages the gz plugins need (`libopencv-dev` for
`gz_plugins/optical_flow`, `libgstreamer-plugins-base1.0-dev`, `cppzmq-dev`
on Ubuntu 24.04, plus Eigen, unwind, pkg-config, and protobuf), and builds
the `v1.17.0` tag inside that image. It also installs the Ubuntu 24.04
package `bc=1.07.1-3ubuntu4`. The stock `rcS` uses `bc` to scale
`COM_DL_LOSS_T`, `COM_RC_LOSS_T`, `COM_OF_LOSS_T`, and `COM_OBC_LOSS_T` with
`PX4_SIM_SPEED_FACTOR`. This repo does not set those parameters.

`gz_bridge/CMakeLists.txt` in that tag adds the `gz_x500` ninja target only
when `Tools/simulation/gz/worlds/*.sdf` exists during configure. That path is
the `PX4-gazebo-models` submodule (`worlds/default.sdf` and `models/x500`).
A depth-1 clone leaves it empty, and PX4 fetches it only for an
exclude-from-all `gz` target that `px4_sitl_default` does not build, so the
firmware links and `ninja` still has no `gz_x500` target. The script checks
that submodule out to the gitlink recorded by tag `v1.17.0` and refuses to
configure when `worlds/default.sdf` is missing.

The plan's budget for a prebuilt image is 20 minutes cold and 8 minutes warm.
The workflow's hard timeout is 40 minutes so the first compile can finish and
be compared with that budget. The image cache key is the index digest above.

## Simulated GCS

PX4 v1.17 `RcAndDataLinkChecks` blocks arming while `gcs_connection_lost` is
set and `NAV_DLL_ACT` is greater than 0. Commander starts with that flag
set. It clears the flag only after a MAVLink link reports a `MAV_TYPE_GCS`
heartbeat (`heartbeat_type_gcs`). The receiver drops that report 2.5 s after
the last heartbeat. The console line is `Preflight Fail: No connection to
the GCS`.

SITL instance 0 starts the GCS link as `mavlink start -x -u 18570`
(`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`). `-o` is omitted, so the
remote port stays the mavlink default 14550, and `init_udp` sends to
`127.0.0.1`. `scripts/sitl_gcs_heartbeat.py` binds `127.0.0.1:14550` and
sends only that heartbeat to `127.0.0.1:18570` once a second for the whole
run. pymavlink is pinned in `requirements-sitl.txt` (`pymavlink==2.4.50`).

The heartbeat does not arm, does not send any other MAVLink message, and
does not change `NAV_DLL_ACT`, `COM_DL_LOSS_T`, or any other PX4 parameter.
This repository has no vehicle parameter file. The stock data-link check
stays in force. `PX4_INSTANCE` other than 0 is refused; this harness runs
instance 0.

## Pass / fail

The job fails unless all of these are true:

- Within 120 s, `/fmu/out/vehicle_status_v1` is visible, the battery
  fraction is at least 0.3, and `pre_flight_checks_pass` is true. PX4 v1.17
  publishes `vehicle_status`, `vehicle_local_position`, and `battery_status`
  with `MESSAGE_VERSION` 1. ohho-os `ohh-290-px4-dds` at `5acb80e` subscribes
  to those versioned names, so this recipe does not run a topic relay.
- If that wait times out, `sitl-result.json` lists the `Preflight Fail:`
  lines from the last 100 lines of the last 256 KiB of the PX4 console.
  Checks that have stopped printing are not reported.
- After takeoff, `|z_ned + 2.5| < 0.4` m for 2 s.
- The square has 4 m sides in local NED, flown at 0.5 m/s with altitude held.
  Each corner is accepted only when the horizontal error is within 0.8 m and
  the same 0.4 m altitude band still holds.
- The run ends with `landed_state == on_ground` and the motors disarmed.
- No failsafe flag (`offboard_loss`, `rc_loss`, `geofence`, `battery`,
  `datalink`) is set after the takeoff hold.

A timeout, a missed corner, a missing disarm, or a failsafe flag fails the
run. The script does not change PX4 failsafe parameters.

Disarm is a normal disarm, and only after the vehicle is on the ground with
height at or below 0.25 m. The script never sends force-disarm and never sends
flight termination.

## Safety

Commands go through `Robot` and the aerial `SafetyGate`. The URI has to be a
loopback `px4+dds://` host (`127.0.0.0/8`, `localhost`, or `::1`).

The script and the workflow do not set `OHHO_ARM_HARDWARE`. If that variable
is present at all, they exit. They do not pass `--arm`. DDS stays inside the
container, so the recipe does not use host networking or `--privileged`.

The "Fly the square" step sets `OHHO_CI_SIM_ARMING=1` and `OHHO_CI_SITL=1`
and passes both into the container with `CI` and `GITHUB_ACTIONS`. That is
the sim-only CI arming opt-in. It applies only when `CI` and
`GITHUB_ACTIONS` are true, `OHHO_ARM_HARDWARE` is absent, the gate is not a
hardware gate, and this loopback `px4+dds` target has `sim_target` true.
Under GitHub Actions, `scripts/sitl_square.py` refuses to connect when
either variable is not exactly `1`.

The gate stays fully enforced: geofence, altitude cap, battery, link, the
setpoint stream, the allowlist, speed caps, and the e-stop. The script does
not clear `CI`, allocate a fake TTY, set `OHHO_ARM_HARDWARE`, or call
`send_flight` on the inner transport. `OHHO_OS_REF` stays `ohh-290-px4-dds`.
After install, the container log prints the resolved ohho-os commit.

`arm()` on a `sim_target` does not ask for a human TTY, and
`requires_hardware_arming` is false for that target. `arm_flight` and
`takeoff` still go through `_human_tty_reason` and `_aerial_privileged`.
Without the opt-in, CI is reason `ci` and a headless process is `no_tty`.
`sitl-result.json` names that refusal. The square can pass only when the
installed ohho-os includes the exception and every remaining gate check
passes. This page does not claim that a flight has already happened.

## Local recipe

Linux with Docker. Do not point this at a machine that has a flight controller
plugged in. The image is about 2 GB; this recipe pulls it.

```bash
mkdir -p artifacts
docker run -d --name ohho-px4 --entrypoint /bin/bash \
  -e HEADLESS=1 \
  -e PX4_SIM_MODEL=gz_x500 \
  -e PX4_GZ_WORLD=default \
  -e PX4_SIM_SPEED_FACTOR=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD:/opt/ohho-drone:ro" \
  -v "$PWD/artifacts:/artifacts" \
  px4io/px4-dev:v1.17.0 \
  -lc 'sleep infinity'

docker exec ohho-px4 bash /opt/ohho-drone/scripts/run_sitl_px4.sh
docker rm -f ohho-px4
```

That `docker exec` has no TTY, and the recipe does not set `OHHO_ARM_HARDWARE`
or the CI opt-in variables. Outside GitHub Actions, `arm_flight` and
`takeoff` are refused without a TTY. The local command is the same script
the CI job runs, without that opt-in. It is not a completed flight.

Confirm the image digest is the index or amd64 digest in the table before
trusting a retag of `v1.17.0`. `scripts/run_sitl_px4.sh` checks out PX4
`v1.17.0`, installs the simulation packages above, checks out the Gazebo
models submodule, builds `px4_sitl_default`, starts `MicroXRCEAgent udp4 -p
8888`, then runs `bin/px4 -d` with `HEADLESS=1`, `PX4_SIM_MODEL=gz_x500`,
`PX4_GZ_WORLD=default`, `PX4_SIM_SPEED_FACTOR=1`, and `GZ_IP=127.0.0.1`.
`-d` is daemon mode, so the `pxh` shell does not run. The `gz_x500` cmake
target launches the same binary without `-d`; that shell reads stdin, and a
redirect from `/dev/null` makes it print `pxh> ` until the log is huge.
The Ohho process is `scripts/sitl_square.py`. The simulated GCS heartbeat
is started before PX4 and left running until the container script exits.

Mission rules without a simulator:

```bash
python -m unittest discover -s tests -v
```

## Artifacts

Pass or fail, the workflow uploads `artifacts/`. The container writes that
directory as root. After `docker rm`, the workflow runs `sudo chown -R` so
the runner can replace `px4-console-tail.txt` and the upload can read the
files.

- `px4-console.log` and `px4-console-tail.txt` (last 200 lines of the last
  256 KiB, so a newline-free prompt cannot copy the whole console)
- `xrce-agent.log`
- `gcs-heartbeat.log`
- `sitl-result.json` (criteria, phases, corner errors, failsafes, duration,
  and `preflight_failures` when the status wait times out)
- any `*.ulg` logs the PX4 build wrote

`duration_s` in `sitl-result.json` is only the mission clock after
`sitl_square.py` starts. The GitHub Actions job time includes the image pull,
the Gazebo install, and the PX4 compile. Compare that job time with the 20
minute cold budget.
