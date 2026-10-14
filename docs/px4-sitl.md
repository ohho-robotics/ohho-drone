# PX4 SITL Simulation & Hello-World Guide

> **WARNING: EXPERIMENTAL PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY**  
> This guide and associated scripts are strictly for software simulation (SITL).  
> Do not use for real aircraft flight or physical hardware control.

---

## 1. Introduction

PX4 SITL (Software-In-The-Loop) runs the PX4 flight control stack directly as a software process on your host computer or inside a Docker container. In SITL mode, the autopilot communicates over standard UDP sockets using the MAVLink protocol.

This repository provides an experimental adapter and hello-world script that connect OhhO OS to a PX4 SITL instance.

---

## 2. Architecture & Network Topology

In standard PX4 SITL, communication ports are allocated as follows:

```text
┌──────────────────────────────────────────────┐
│                  PX4 SITL                    │
│   (Docker container or local native binary)  │
└──────┬────────────────────────────────┬──────┘
       │ UDP :14540                     │ UDP :14550
       ▼                                ▼
┌──────────────────────────┐     ┌──────────────────────────┐
│ OhhO Drone Transport     │     │ QGroundControl (GCS)     │
│ (MavlinkDroneTransport)  │     │ (Optional GUI monitor)   │
└──────────────────────────┘     └──────────────────────────┘
```

- **`14540/udp`**: Default offboard API port. The OhhO Drone MAVLink transport connects here to exchange heartbeats, telemetry, and velocity setpoints.
- **`14550/udp`**: Default Ground Control Station (GCS) port. You can concurrently run QGroundControl to visually observe the simulated drone's artificial horizon, flight mode, and map location.

---

## 3. Running PX4 SITL

You can run PX4 SITL using either **Docker** (recommended for zero local toolchain setup) or a **local source build** in Ubuntu/WSL2.

### Option A: Official Docker Container (Recommended)

PX4 provides official pre-built containers with all simulation dependencies bundled.

```bash
# Pull and run headless PX4 SITL quadrotor container
docker run --rm -it \
  --net=host \
  -e PX4_SIM_SPEED_FACTOR=1 \
  px4io/px4-sitl:v1.14.0
```

*Note: On Linux or WSL2, `--net=host` binds the container's UDP ports directly to localhost.*

### Option B: Local Native Build (Ubuntu 22.04 / 24.04 or WSL2)

To build and run PX4 SITL directly from source:

1. **Clone PX4 Autopilot:**
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   ```

2. **Install Ubuntu dependencies:**
   ```bash
   bash ./Tools/setup/ubuntu.sh
   # Reboot or log out and back in after toolchain installation
   ```

3. **Launch Headless SITL:**
   ```bash
   # Run minimal headless simulation (no 3D GUI window required)
   HEADLESS=1 make px4_sitl none_iris
   ```

4. **(Optional) Launch with Gazebo Simulator:**
   ```bash
   make px4_sitl gz_x500
   ```

---

## 4. Running the OhhO Drone Hello-World Script

Once PX4 SITL is running (or in headless CI environments using the built-in mock mode), execute `scripts/px4_sitl_hello.py`.

### A. Headless Mock Mode (CI & Fast Local Verification)

If Docker or PX4 is not installed, the script includes an in-process pure-Python mock SITL server that simulates heartbeat generation, attitude/odometry streaming, battery health, and arming/takeoff/landing lifecycles:

```bash
python scripts/px4_sitl_hello.py --mock
```

Sample output:
```text
========================================================================
  OhhO Drone -- PX4 SITL MAVLink Hello-World Prototype
  Status: EXPERIMENTAL STRETCH PROTOTYPE -- NOT FLIGHT-READY
  Software simulation only -- No real aircraft or hardware
========================================================================

[1/5] Starting in-process Mock PX4 SITL server on 127.0.0.1...
      Mock SITL server listening on UDP 127.0.0.1:61763
[2/5] Connecting to MAVLink endpoint (timeout: 4.0s)...
[+] Connected! Protocol: mavlink, State: connected
    Label: ohho-drone-mavlink (experimental/not-flight-ready)

[3/5] Initial Telemetry Snapshot:
      Armed: False | Mode: disarmed
      Battery: 98.0% (12.40V)
      GNSS: 47.397742, 8.545594 | Alt: 0.00m
      Pose (x, y, z): 0.00m, 0.00m, 0.00m

[4/5] Executing simulated flight sequence:
      -> Sending ARM command...
      [+] Autopilot armed.
      -> Sending TAKEOFF command (target: 2.5m)...
      -> Sending forward velocity setpoint (linear_x=0.5 m/s)...
      -> Sending LAND command...

[5/5] Disconnecting...
[+] Clean shutdown complete. Received 246 telemetry frames.
```

### B. Live PX4 SITL Mode

When running against a live PX4 container or local build listening on port 14540:

```bash
python scripts/px4_sitl_hello.py --host 127.0.0.1 --port 14540
```

---

## 5. Automated Regression Smoke Test

For continuous integration and automated test suites, run:

```bash
python scripts/run_smoke_test.py
```

This executes an end-to-end handshake, validates telemetry parsing (odometry, 6-DOF pose, GNSS, battery), exercises simulated arming, takeoff climb, velocity setpoints, and emergency stop disarm in under 2 seconds.
