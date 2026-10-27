# OhhO Drone Architecture & Documentation

> **WARNING: EXPERIMENTAL STRETCH PROTOTYPE — NOT FLIGHT-READY — SIMULATION ONLY**  
> Nothing in this repository has been tested on real aircraft hardware.  
> The workflows in `.github/workflows/` do not run hardware commands (`python.yml`, `credibility.yml`, `sitl-ardupilot.yml`). This tree does not show what other automation might run.  
> Do not use this software for physical flight or real flight controllers.

---

## Overview

This directory documents the aerial robotics reference architecture for **OhhO OS**.
The current implementation provides a prototype MAVLink transport adapter and minimal PX4 SITL (Software-In-The-Loop) hello-world integration scripts.

### Current Status

| Component | Status | Description |
|---|---|---|
| **MAVLink Packet & Message Codec** | Prototype | Pure-Python MAVLink v1/v2 frame serialization, checksums, and standard message dataclasses. |
| **OhhO OS Transport Adapter** | Prototype | `MavlinkDroneTransport` implementing the OhhO OS `Transport` port with 3D velocity and telemetry translation. |
| **Mock PX4 SITL Server** | Prototype | In-process UDP simulation server for zero-dependency CI regression testing. |
| **PX4 SITL Hello-World Script** | Prototype | `scripts/px4_sitl_hello.py` supporting live UDP SITL or headless mock mode. |
| **Real Airframe Hardware** | Vision | Unstarted. Gated on physical airframe selection, safety pilot, and geofence verification. |

---

## Documentation Index

- [PX4 SITL Simulation Guide](px4-sitl.md) — How to run PX4 SITL via Docker or local builds, network ports, and hello-world execution.
- [MAVLink Adapter Specification](mavlink-adapter.md) — Technical specification of the OhhO OS MAVLink transport adapter, message mappings, coordinate frames, and safety boundaries.

---

## Directory Layout

```text
ohho-drone/
├── .github/workflows/
│   ├── credibility.yml    # Validates honesty guarantees and absence of false claims
│   └── python.yml         # CI: ruff linting, unittest discovery, regression smoke test
├── docs/
│   ├── README.md          # Architecture overview (this document)
│   ├── px4-sitl.md        # PX4 SITL setup and hello-world guide
│   └── mavlink-adapter.md # MAVLink adapter technical specification
├── ohho_drone/
│   ├── __init__.py        # Package exports and experimental banner
│   ├── adapter.py         # MavlinkDroneTransport (implements OhhO OS Transport port)
│   ├── schema.py          # DroneVelocity, DronePose, FlightMode, DroneTelemetry
│   ├── mavlink/           # Pure-Python MAVLink v1/v2 codec and typed messages
│   └── sitl/              # Mock PX4 SITL server for headless CI
├── scripts/
│   ├── check-credibility.sh # Credibility validation script
│   ├── px4_sitl_hello.py  # Minimal PX4 SITL hello-world script
│   └── run_smoke_test.py  # Automated regression smoke test
├── tests/                 # Full unit and integration test suite
└── pyproject.toml         # Python package metadata and ruff configuration
```
