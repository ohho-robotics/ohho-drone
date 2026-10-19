#!/usr/bin/env bash
# Start PX4 v1.17.0 + Gazebo Harmonic gz_x500 (headless) and fly the Ohho square.
#
# Runs inside px4io/px4-dev:v1.17.0. DDS stays in this container: the agent,
# PX4, and the Ohho process share localhost. Do not pass OHHO_ARM_HARDWARE.
set -euo pipefail

PX4_GIT_REF="${PX4_GIT_REF:-v1.17.0}"
PX4_MSGS_REF="${PX4_MSGS_REF:-release/1.17}"
XRCE_AGENT_REF="${XRCE_AGENT_REF:-v2.4.3}"
OHHO_OS_REF="${OHHO_OS_REF:-ohh-290-px4-dds}"
PX4_REPO="${PX4_REPO:-https://github.com/PX4/PX4-Autopilot.git}"
PX4_MSGS_REPO="${PX4_MSGS_REPO:-https://github.com/PX4/px4_msgs.git}"
XRCE_AGENT_REPO="${XRCE_AGENT_REPO:-https://github.com/eProsima/Micro-XRCE-DDS-Agent.git}"
OHHO_OS_URL="${OHHO_OS_URL:-git+https://github.com/ohho-robotics/ohho-os.git@${OHHO_OS_REF}}"
REPO_DIR="${REPO_DIR:-/opt/ohho-drone}"
ARTIFACT_DIR="${ARTIFACT_DIR:-/artifacts}"
PX4_SRC="${PX4_SRC:-/opt/PX4-Autopilot}"
PX4_WS="${PX4_WS:-/opt/px4_ws}"
AGENT_SRC="${AGENT_SRC:-/opt/Micro-XRCE-DDS-Agent}"

if [ "${OHHO_ARM_HARDWARE:-}" = "1" ]; then
  echo "refusing to run: OHHO_ARM_HARDWARE is set" >&2
  exit 1
fi
if [ "$(id -u)" -ne 0 ]; then
  echo "run this script as root inside the PX4 dev container" >&2
  exit 1
fi

export HEADLESS=1
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
export PX4_SIM_SPEED_FACTOR=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export PYTHONUNBUFFERED=1
export DEBIAN_FRONTEND=noninteractive

mkdir -p "$ARTIFACT_DIR"

apt_get() { apt-get "$@"; }

log() { echo "ohho-sitl: $*"; }

clone_ref() {
  local dest="$1" url="$2" ref="$3" kind="$4"
  if [ ! -d "$dest/.git" ]; then
    rm -rf "$dest"
    git clone --depth 1 --branch "$ref" "$url" "$dest"
  fi
  if [ "$kind" = "tag" ]; then
    local described head tag_commit
    described="$(git -C "$dest" describe --tags --exact-match HEAD 2>/dev/null || true)"
    head="$(git -C "$dest" rev-parse HEAD)"
    tag_commit="$(git -C "$dest" rev-parse "${ref}^{}" 2>/dev/null || true)"
    if [ "$described" != "$ref" ] && { [ -z "$tag_commit" ] || [ "$head" != "$tag_commit" ]; }; then
      echo "checkout $dest is '${described:-unknown}', expected tag $ref" >&2
      exit 1
    fi
  else
    local branch
    branch="$(git -C "$dest" rev-parse --abbrev-ref HEAD)"
    if [ "$branch" != "$ref" ]; then
      echo "checkout $dest is branch '$branch', expected $ref" >&2
      exit 1
    fi
  fi
  log "$(git -C "$dest" log -1 --oneline)"
}

install_gazebo() {
  if command -v gz >/dev/null 2>&1 && gz sim --versions >/dev/null 2>&1; then
    log "Gazebo already installed: $(gz sim --versions | head -n 1)"
    return
  fi
  log "installing Gazebo Harmonic"
  apt_get update -y --quiet
  apt_get install -y --no-install-recommends curl ca-certificates gnupg
  mkdir -p /usr/share/keyrings /etc/apt/sources.list.d
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  . /etc/os-release
  if [ -z "${UBUNTU_CODENAME:-}" ]; then
    echo "UBUNTU_CODENAME is unset; Gazebo's apt repo needs Ubuntu" >&2
    exit 1
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
  apt_get update -y --quiet
  apt_get install -y --no-install-recommends gz-harmonic libgl1-mesa-dri
}

install_px4_sim_deps() {
  # px4io/px4-dev:v1.17.0 is Ubuntu 24.04 with Tools/setup/ubuntu.sh --no-sim-tools.
  # gz-harmonic is installed above. The v1.17.0 sim set also has the packages
  # the gz plugins configure against: optical_flow requires OpenCV, the
  # gstreamer plugin needs the GStreamer dev files, and Ubuntu 24.04's
  # ubuntu.sh also installs cppzmq-dev with Gazebo Harmonic (libzmq3-dev
  # does not ship zmq.hpp). Eigen, unwind, pkg-config, and protobuf are the
  # rest of that ubuntu.sh list the bridge and plugins link against.
  log "installing PX4 v1.17.0 Gazebo plugin packages"
  . /etc/os-release
  local packages=(
    libeigen3-dev
    libgstreamer-plugins-base1.0-dev
    libopencv-dev
    libunwind-dev
    pkg-config
    protobuf-compiler
  )
  if [ "${VERSION_ID:-}" != "24.04" ]; then
    echo "bc is pinned to Ubuntu 24.04 package 1.07.1-3ubuntu4; got ${VERSION_ID:-unset}" >&2
    exit 1
  fi
  # rcS scales COM_DL_LOSS_T, COM_RC_LOSS_T, COM_OF_LOSS_T, and COM_OBC_LOSS_T
  # with bc. This repo does not set those parameters; the stock startup script
  # does, once bc exists. Noble publishes this exact binary.
  packages+=(cppzmq-dev "bc=1.07.1-3ubuntu4")
  apt_get update -y --quiet
  apt_get install -y --no-install-recommends "${packages[@]}"
  command -v bc >/dev/null
}

install_ros() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    log "ROS 2 Jazzy already installed"
    return
  fi
  log "installing ROS 2 Jazzy"
  apt_get update -y --quiet
  apt_get install -y --no-install-recommends curl ca-certificates gnupg
  mkdir -p /usr/share/keyrings /etc/apt/sources.list.d
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  . /etc/os-release
  if [ -z "${UBUNTU_CODENAME:-}" ]; then
    echo "UBUNTU_CODENAME is unset; the ROS apt repo needs Ubuntu" >&2
    exit 1
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    > /etc/apt/sources.list.d/ros2.list
  apt_get update -y --quiet
  apt_get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    cmake
}

install_agent() {
  if command -v MicroXRCEAgent >/dev/null 2>&1; then
    log "MicroXRCEAgent already on PATH"
    return
  fi
  log "building Micro XRCE-DDS Agent ${XRCE_AGENT_REF}"
  clone_ref "$AGENT_SRC" "$XRCE_AGENT_REPO" "$XRCE_AGENT_REF" tag
  cmake -S "$AGENT_SRC" -B "$AGENT_SRC/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$AGENT_SRC/build" -j"$(nproc)"
  cmake --install "$AGENT_SRC/build"
  ldconfig || true
  command -v MicroXRCEAgent >/dev/null
}

build_px4_msgs() {
  log "building px4_msgs ${PX4_MSGS_REF}"
  mkdir -p "$PX4_WS/src"
  clone_ref "$PX4_WS/src/px4_msgs" "$PX4_MSGS_REPO" "$PX4_MSGS_REF" branch
  (
    cd "$PX4_WS"
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select px4_msgs
  )
}

checkout_gz_models() {
  # v1.17.0 src/modules/simulation/gz_bridge/CMakeLists.txt globs
  # Tools/simulation/gz/worlds/*.sdf while configuring, then adds ninja
  # target gz_<model> for each airframe in the default world. That tree is
  # the PX4-gazebo-models submodule. clone --depth 1 does not populate it,
  # and px4_add_git_submodule only fetches it for the exclude-from-all `gz`
  # target, which px4_sitl_default does not build. An empty worlds directory
  # means cmake emits no gz_x500 target (the failure on run 39294116216).
  log "checking out PX4 Gazebo models"
  git -C "$PX4_SRC" submodule update --init -- Tools/simulation/gz
  local recorded head
  recorded="$(git -C "$PX4_SRC" rev-parse "HEAD:Tools/simulation/gz")"
  head="$(git -C "$PX4_SRC/Tools/simulation/gz" rev-parse HEAD)"
  if [ "$recorded" != "$head" ]; then
    echo "Gazebo models checkout is ${head}, expected ${recorded}" >&2
    exit 1
  fi
  if [ ! -f "$PX4_SRC/Tools/simulation/gz/worlds/default.sdf" ]; then
    echo "Gazebo world worlds/default.sdf is missing; gz_x500 cannot be generated" >&2
    exit 1
  fi
  local build_dir="$PX4_SRC/build/px4_sitl_default"
  if [ -f "$build_dir/build.ninja" ] && ! grep -q 'build gz_x500:' "$build_dir/build.ninja"; then
    log "dropping PX4 build configured before Gazebo worlds were present"
    rm -rf "$build_dir"
  fi
}

build_px4() {
  log "building PX4 ${PX4_GIT_REF} SITL"
  clone_ref "$PX4_SRC" "$PX4_REPO" "$PX4_GIT_REF" tag
  checkout_gz_models
  make -C "$PX4_SRC" px4_sitl_default
}

install_ohho() {
  log "installing ohho-os @ ${OHHO_OS_REF}"
  python3 -m pip install --break-system-packages "$OHHO_OS_URL"
}

AGENT_PID=""
PX4_PID=""
RELAY_PID=""

# GNU tail -n copies an entire line. The pxh prompt is "pxh> " with no newline,
# so an unbounded tail -n reproduced the whole console (run 39300774890, 1.1 GB).
# Read only the last 256 KiB, then keep the last 200 lines of that slice.
write_console_tail() {
  local src="$1" dest="$2"
  if [ -f "$src" ]; then
    tail -c 262144 "$src" | tail -n 200 > "$dest"
  fi
}

copy_logs() {
  local status="$1"
  trap - EXIT
  set +e
  if [ -n "$PX4_PID" ]; then
    kill -- "-$PX4_PID" 2>/dev/null || kill "$PX4_PID" 2>/dev/null
    wait "$PX4_PID" 2>/dev/null
  fi
  if [ -n "$AGENT_PID" ]; then
    kill "$AGENT_PID" 2>/dev/null
    wait "$AGENT_PID" 2>/dev/null
  fi
  if [ -n "$RELAY_PID" ]; then
    kill "$RELAY_PID" 2>/dev/null
    wait "$RELAY_PID" 2>/dev/null
  fi
  mkdir -p "$ARTIFACT_DIR"
  write_console_tail "$ARTIFACT_DIR/px4-console.log" "$ARTIFACT_DIR/px4-console-tail.txt"
  if [ -d "$PX4_SRC" ]; then
    find "$PX4_SRC" -name '*.ulg' -print 2>/dev/null \
      | while IFS= read -r ulog; do
          cp "$ulog" "$ARTIFACT_DIR/" || true
        done
  fi
  log "artifacts in $ARTIFACT_DIR (exit $status)"
  ls -l "$ARTIFACT_DIR" || true
  exit "$status"
}
trap 'copy_logs $?' EXIT

install_gazebo
install_px4_sim_deps
install_ros
install_agent
build_px4_msgs
build_px4
install_ohho

log "starting MicroXRCEAgent udp4 -p 8888"
MicroXRCEAgent udp4 -p 8888 > "$ARTIFACT_DIR/xrce-agent.log" 2>&1 &
AGENT_PID=$!
sleep 1
if ! kill -0 "$AGENT_PID" 2>/dev/null; then
  echo "MicroXRCEAgent exited" >&2
  cat "$ARTIFACT_DIR/xrce-agent.log" >&2 || true
  exit 1
fi

log "starting headless gz_x500 at speed factor 1"
# The gz_x500 cmake target runs bin/px4 with no arguments, so pxh reads stdin.
# docker exec has no TTY and this redirect is /dev/null, so the prompt spins
# and fills the console log. -d is daemon mode: rcS still runs, pxh does not.
# setsid forks when this shell is already a session leader, and $! would be
# that parent. The child records its own pid, which stays the session leader
# across the exec into px4.
rm -f "$ARTIFACT_DIR/px4.pid"
setsid bash -c '
  echo $$ > "$4/px4.pid"
  cd "$1/build/px4_sitl_default"
  exec env HEADLESS=1 \
    PX4_SIM_MODEL=gz_x500 \
    PX4_GZ_WORLD="$2" \
    PX4_SIM_SPEED_FACTOR=1 \
    LIBGL_ALWAYS_SOFTWARE="$3" \
    GZ_IP=127.0.0.1 \
    "$1/build/px4_sitl_default/bin/px4" -d
' bash "$PX4_SRC" "$PX4_GZ_WORLD" "$LIBGL_ALWAYS_SOFTWARE" "$ARTIFACT_DIR" \
  > "$ARTIFACT_DIR/px4-console.log" 2>&1 < /dev/null &
PX4_PID=""
for _ in $(seq 1 50); do
  if [ -s "$ARTIFACT_DIR/px4.pid" ]; then
    PX4_PID="$(tr -d '[:space:]' < "$ARTIFACT_DIR/px4.pid")"
    break
  fi
  sleep 0.1
done
if [ -z "$PX4_PID" ]; then
  echo "PX4 did not record a session pid" >&2
  exit 1
fi

ready=0
for _ in $(seq 1 180); do
  if ! kill -0 "$PX4_PID" 2>/dev/null; then
    echo "PX4 exited during startup" >&2
    write_console_tail "$ARTIFACT_DIR/px4-console.log" "$ARTIFACT_DIR/px4-console-tail.txt"
    cat "$ARTIFACT_DIR/px4-console-tail.txt" >&2 || true
    exit 1
  fi
  if grep -E -q "Startup script returned successfully" "$ARTIFACT_DIR/px4-console.log"; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  log "PX4 console did not report ready within 180s; the mission still waits for vehicle_status"
fi

log "flying the square through Ohho"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PX4_WS/install/setup.bash"
set -u
log "relaying versioned PX4 topics onto the names Ohho subscribes to"
python3 "$REPO_DIR/scripts/relay_px4_topics.py" > "$ARTIFACT_DIR/topic-relay.log" 2>&1 &
RELAY_PID=$!
sleep 1
if ! kill -0 "$RELAY_PID" 2>/dev/null; then
  echo "topic relay exited" >&2
  cat "$ARTIFACT_DIR/topic-relay.log" >&2 || true
  exit 1
fi
python3 "$REPO_DIR/scripts/sitl_square.py" \
  --uri px4+dds://127.0.0.1:8888 \
  --result "$ARTIFACT_DIR/sitl-result.json"
