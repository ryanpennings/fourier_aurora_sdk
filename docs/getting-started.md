# Getting Started

This guide takes you from a clean machine to sending your first commands to a Fourier robot (real or simulated).

## 1. Prerequisites

### Supported robots (v1.3.0)

- GR-1P
- GR-2
- GR-3 (hardware revisions v211, v222, v224)
- Fourier-N1

### Actuator firmware (real robots only)

| Component | Minimum version |
|---|---|
| Communication firmware | 0.3.12.31 |
| Driver firmware | 0.2.10.30 |

Actuator firmware can be upgraded with **FSA Assistant** ([Linux download](https://fsa-1302548221.cos.ap-shanghai.myqcloud.com/tool/FSA_Assistant/FSA_Assistant_V0.0.1.24_155_31_x64_Linux_2025-07-08.tar.gz)).

### Software

- Linux (the Docker script and simulator paths assume a Linux host with X11)
- Docker (for running the Aurora server / simulator locally)
- Python 3 with `pip`

The example scripts additionally use these Python packages (only needed for the demos that import them): `numpy`, `torch` (for the RL walking demos), `pygame` (joystick input), and `ischedule` (fixed-rate control loops).

## 2. Install the Python client

```bash
pip install fourier_aurora_client==0.1.8
```

This installs the `fourier_aurora_client` package providing `AuroraClient` and `MoveCommandManager`. The client talks to the Aurora server over DDS; nothing in this repository needs to be built.

For installing the Aurora server/system itself, refer to the [Fourier Document Center](https://support.fftai.com).

## 3. Configure the server (`config/config.yaml`)

```yaml
Config:
  RobotName: gr2        # gr1 / gr2 / gr3 / fouriern1
  HardwareType: null    # null for gr2 and fouriern1; v211, v222 or v224 for gr3
  RunType: 0            # 0 run simulator / 1 control real robot
  UseJoyStick: true     # use as default
  DomainID: 123         # use as default
```

| Key | Meaning |
|---|---|
| `RobotName` | Which robot model the server controls: `gr1`, `gr2`, `gr3`, or `fouriern1`. |
| `HardwareType` | Hardware revision. Only meaningful for GR-3 (`v211`, `v222`, `v224`); leave `null` for GR-2 and Fourier-N1. |
| `RunType` | `0` = drive the MuJoCo simulator, `1` = drive the real robot. |
| `UseJoyStick` | Whether the server accepts joystick velocity input. Leave `true` (the default). |
| `DomainID` | The DDS domain ID. Every `AuroraClient.get_instance(domain_id=...)` call must use the same value. Default is `123`. |

## 4. Start the Aurora server (Docker)

`docker_run.bash` starts the server container with the repository mounted at `/workspace`, host networking (required for DDS discovery), and X11 forwarding (required for the MuJoCo viewer window):

```bash
bash docker_run.bash
```

The script runs, in essence:

```bash
xhost +local:root
docker run -it \
 -e DISPLAY=$DISPLAY \
 -v "$(pwd)":/workspace -w /workspace \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged --net=host --ipc=host --cap-add=SYS_NICE \
 --name fourier_aurora_server \
 fourier_aurora_sdk_gr2:v1.2.0 bash
```

> Note: the image tag in the script (`fourier_aurora_sdk_gr2:v1.2.0`) is robot-specific. Use the image matching your robot model and SDK release — see the CHANGELOG and the Fourier Document Center for the current image names per robot.

## 5. (Optional) Start the simulator

If `RunType: 0`, launch the MuJoCo simulator from inside the container:

```bash
python3 sim/start_simulate_root.py
```

The launcher lists robot models found under `/usr/local/resources` and terrains under `/usr/local/terrains`, prompts you to pick each by number, and then starts the interactive MuJoCo viewer. See [Simulation](simulation.md) for details of how the simulator exchanges state and commands with the server over LCM.

## 6. Your first script

With the server (and simulator, if applicable) running, this minimal script stands the robot up and walks forward (adapted from `python/example/gr3/demo_quick_start.py`):

```python
import time
from fourier_aurora_client import AuroraClient

# domain_id must match DomainID in config.yaml; robot_name matches your robot
client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
time.sleep(1)  # allow DDS discovery to complete

input("Press Enter to switch to PdStand...")
client.set_fsm_state(2)          # 2 = PD stand
time.sleep(1.0)

input("Press Enter to enter walk task...")
client.set_fsm_state(3)          # 3 = RL locomotion
time.sleep(1.0)
client.set_velocity_source(2)    # 2 = accept velocity commands over DDS

input("Press Enter to walk...")
client.set_velocity(0.2, 0.0, 0.0, 3.0)  # 0.2 m/s forward for up to 3 s
time.sleep(1.0)

client.close()
```

Key points to remember:

- **Sleep ~1 s after `get_instance`** so DDS discovery finishes before you issue commands.
- **FSM state ordering matters.** Enter PD stand (`2`) and let the robot stabilize before switching to RL locomotion (`3`) or user command (`10`) states. See [Architecture — FSM states](architecture.md#whole-body-fsm-states).
- **Velocity source must be `2` (Navigation/DDS)** for `set_velocity()` calls from the client to take effect; source `0` means the joystick is in control.
- **Always call `client.close()`** when finished.

## 7. Where to go next

- [Examples Guide](examples.md) — every demo script explained, per robot.
- [API Reference](api-reference.md) — all client getters/setters and the move-command API.
- [Robot Reference](robots.md) — control group names and joint counts you will need for joint-level control.
