# OhhO Drone

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

**Concept — not started.** The working robot is [OmniBot](https://github.com/ohho-robotics/OmniBot).

This repository is a placeholder for a drone form factor in the [OhhO Robotics](https://github.com/ohho-robotics) org. It does not contain a robot description or a hardware flight controller. An experimental software-simulation hello-world is documented in [docs/README.md](docs/README.md). Nothing here has flown.

Research and plan: [docs/research/drone-stacks.md](docs/research/drone-stacks.md), [docs/plan/implementation-plan.md](docs/plan/implementation-plan.md).

## Clone

```bash
git clone https://github.com/ohho-robotics/ohho-drone.git
```

There is no install step. Nothing in this tree builds or launches a robot.

## Emergency stop

This tree is a software simulation sketch. The experimental transport refuses any connection that is not a local loopback address. An explicit simulation flag may also allow a local container hostname, and it still refuses every other address.

Emergency stop does not kill motors in the air. An airborne vehicle is commanded to land. If it is unknown whether the vehicle is on the ground, the transport commands a hold and then a landing. Motors are disarmed only after the vehicle is confirmed on the ground.

## License

[Apache License 2.0](LICENSE).

## Roadmap

Not in this repository:

- A checked-in airframe description and a simulator launch that runs from public packages
- A flight-controller hello-world that has been run on a real vehicle
- Hardware bring-up for a specific airframe

## Documentation

For experimental stretch work (software simulation only, not flight-ready), see [docs/README.md](docs/README.md) and [AGENTS.md](AGENTS.md).
