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
| Ohho | `git+https://github.com/ohho-robotics/ohho-os.git@ohh-290-px4-dds` |

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
the `v1.17.0` tag inside that image.

The plan's budget for a prebuilt image is 20 minutes cold and 8 minutes warm.
The workflow's hard timeout is 40 minutes so the first compile can finish and
be compared with that budget. The image cache key is the index digest above.

## Pass / fail

The job fails unless all of these are true:

- `/fmu/out/vehicle_status` shows up within 120 s.
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
is already `1`, they exit. They do not pass `--arm`. DDS stays inside the
container, so the recipe does not use host networking or `--privileged`.

GitHub Actions forwards `CI` and `GITHUB_ACTIONS` into the container. The gate
sees that this is CI.

On the public `ohh-290-px4-dds` branch checked on 19 October 2026
(`053b149`), `px4-sitl` uses the aerial profile, and
`requires_hardware_arming` is false when the transport's `sim_target` is true.
`arm()` on that sim target does not ask for the hardware phrase. `arm_flight`
and `takeoff` still call the human-TTY check, which returns `ci` when `CI` or
`GITHUB_ACTIONS` is set, before it looks at the sim flag. A headless runner
also has no TTY. The script stops when the gate does that. It does not clear
`CI`, allocate a fake TTY, set `OHHO_ARM_HARDWARE`, or call `send_flight` on
the inner transport.

Switch the Ohho install from `ohh-290-px4-dds` to `main` once that branch has
merged. The square can pass only after the gate allows `arm_flight` and
`takeoff` for this loopback target from a headless CI job.

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

That `docker exec` has no TTY, and the recipe does not set `OHHO_ARM_HARDWARE`.
On the current gate, `arm_flight` and `takeoff` are refused without a TTY. The
commands are the ones a later gate can run unchanged. They are not a completed
flight.

Confirm the image digest is the index or amd64 digest in the table before
trusting a retag of `v1.17.0`. `scripts/run_sitl_px4.sh` checks out PX4
`v1.17.0`, installs the simulation packages above, builds `px4_sitl_default`,
starts `MicroXRCEAgent udp4 -p 8888`, then runs `HEADLESS=1
PX4_SIM_MODEL=gz_x500 PX4_SIM_SPEED_FACTOR=1 make px4_sitl gz_x500`. The Ohho
process is `scripts/sitl_square.py`.

Mission rules without a simulator:

```bash
python -m unittest discover -s tests -v
```

## Artifacts

Pass or fail, the workflow uploads `artifacts/`:

- `px4-console.log` and `px4-console-tail.txt` (last 200 lines)
- `xrce-agent.log`
- `sitl-result.json` (criteria, phases, corner errors, failsafes, duration)
- any `*.ulg` logs the PX4 build wrote

`duration_s` in `sitl-result.json` is only the mission clock after
`sitl_square.py` starts. The GitHub Actions job time includes the image pull,
the Gazebo install, and the PX4 compile. Compare that job time with the 20
minute cold budget.
