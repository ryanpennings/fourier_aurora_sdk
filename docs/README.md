# Fourier Aurora SDK — Documentation

Detailed documentation for the **Fourier Aurora SDK** (Advanced Unified Robot Operation and Resource Architecture), version **1.3.0**.

These docs expand on the top-level [README](../README.md) and the concise API listing in [`python/docs/`](../python/docs/API_document_EN.md). They are derived from the SDK source, configuration, and example scripts in this repository.

## Contents

| Document | What it covers |
|---|---|
| [Getting Started](getting-started.md) | Prerequisites, installation, configuration (`config.yaml`), Docker, and your first script |
| [Architecture](architecture.md) | How the client, Aurora server, and simulator fit together; FSM states; control groups; communication layers (DDS / LCM) |
| [API Reference](api-reference.md) | Full `AuroraClient` and `MoveCommandManager` reference with parameter tables, state ID tables, and usage snippets |
| [Robot Reference](robots.md) | Per-robot control groups, joint counts and ordering, default poses, and recommended PD gains |
| [Examples Guide](examples.md) | Walkthrough of every example script, including the RL walking policy demos |
| [Simulation](simulation.md) | The MuJoCo simulation stack: viewer, LCM messages, terrain loading, and utility scripts |

## Quick orientation

- The SDK is used from Python via the **`fourier_aurora_client`** package (`pip install fourier_aurora_client==0.1.8`). This repository contains configuration, examples, docs, and simulation tooling — the client library itself is installed from PyPI, and the Aurora **server** runs on the robot (or in Docker for simulation).
- Communication between client and server is over **DDS** (Fast DDS); the client and server must share the same **domain ID** (default `123` in `config/config.yaml` and all examples).
- Supported robots in v1.3.0: **GR-1P**, **GR-2**, **GR-3** and **Fourier-N1**.
- For system-level documentation (installing Aurora itself, robot bring-up), see the [Fourier Document Center](https://support.fftai.com).

## Repository layout

```
fourier_aurora_sdk/
├── config/
│   └── config.yaml          # Aurora server configuration (robot type, sim vs real, domain ID)
├── python/
│   ├── docs/                # Original concise API documentation (EN + CN)
│   └── example/             # Example scripts, one folder per robot model
│       ├── fouriern1/       # Fourier-N1 examples (+ policy_jit_walk.pt RL policy)
│       ├── gr1p/            # GR-1P examples
│       ├── gr2/             # GR-2 examples (+ policy_jit.pt RL policy)
│       └── gr3/             # GR-3 examples (+ wbc_yaw.pt RL policy)
├── sim/
│   ├── start_simulate_root.py  # Interactive launcher for the MuJoCo simulator
│   └── mujoco/                 # Modified MuJoCo viewer + LCM message definitions
├── docs/                    # ← You are here: detailed documentation
├── docker_run.bash          # Convenience script to start the Aurora server container
├── CHANGELOG                # Version history
├── VERSION                  # Current version (1.3.0)
└── LICENSE                  # Apache 2.0
```
