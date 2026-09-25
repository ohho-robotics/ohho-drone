# OhhO Drone 🚁

The aerial reference architecture for OhhO OS. Integrates MAVLink and PX4 with the OhhO cloud for autonomous 3D spatial mapping and fleet swarm control.

This repository is part of the **[OhhO Robotics Platform](https://github.com/ohho-robotics)**. It acts as the meta-workspace for onboarding this specific form factor into the OhhO ecosystem.

## 🚀 Quick Start (Simulation)
You can test the AI models and control stack for this robot in the OhhO Digital Twin without physical hardware.

```bash
git clone https://github.com/ohho-robotics/OhhO-Drone.git
cd OhhO-Drone
vcs import src < ohho.repos
docker compose up -d
```

## 🧩 OhhO Integration
This hardware profile natively supports:
- **OhhO Fleet**: Live telemetry and multi-agent coordination.
- **OhhO Connect**: ROSBridge / WebRTC low-latency streaming.
- **OhhO Mind**: VLA-based spatial intelligence.
