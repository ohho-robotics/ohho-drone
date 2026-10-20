# AGENTS.md — Coding Agent Guidelines for ohho-drone

## Scope & Purpose

**Experimental stretch prototype — software simulation only. Nothing has flown.**

This repository contains:
- Prototype MAVLink adapter (`ohho_drone/adapter.py`) implementing the OhhO OS `Transport` port.
- Pure-Python MAVLink v1/v2 frame serialization, checksums, and message codec (`ohho_drone/mavlink/`).
- In-process Mock PX4 SITL server (`ohho_drone/sitl/mock_server.py`) for zero-dependency CI testing.
- Minimal PX4 SITL hello-world script (`scripts/px4_sitl_hello.py`) and regression smoke test (`scripts/run_smoke_test.py`).
- SITL simulation setup and adapter architecture documentation (`docs/`).

## Core Rules for AI Coding Agents

1. **Honest Vision & Stretch Prototype:**
   - Do NOT claim real aircraft flight, hardware testing, or flight certification.
   - Mark all code, documentation, and adapters as experimental / not flight-ready.
   - Do NOT make purchases or propose hardware orders.
   - Do NOT publish packages to PyPI or external registries.
2. **Never Merge & No Autonomous Merges:**
   - Always push work to a dedicated feature branch (e.g. `ohh-67-px4-sitl-mavlink`).
   - Open draft PRs only if `gh` CLI is authenticated; otherwise push the branch and leave instructions for human maintainers.
   - Never merge into `main`.
3. **Credibility & Quality Checks:**
   - All code must pass `ruff check .`.
   - All unit tests must pass `python -m unittest discover -s tests`.
   - All regression smoke tests must pass `python scripts/run_smoke_test.py`.
   - Credibility checks must pass `bash scripts/check-credibility.sh`.

## How to Run Checks

```bash
# 1. Linting
ruff check .

# 2. Unit tests
python -m unittest discover -s tests

# 3. Automated regression smoke test
python scripts/run_smoke_test.py

# 4. Hello-world mock verification
python scripts/px4_sitl_hello.py --mock --timeout 3.0

# 5. Credibility check
bash scripts/check-credibility.sh
```
