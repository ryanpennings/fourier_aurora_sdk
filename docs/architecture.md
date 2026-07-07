# Architecture

This page explains how the pieces of the Aurora system fit together and the core concepts you need to control a robot: communication layers, finite state machines (FSMs), control groups, and command paths.

## System overview

```
┌────────────────────┐        DDS (Fast DDS)         ┌────────────────────┐
│  Your Python code  │  domain_id must match server  │   Aurora server    │
│  fourier_aurora_   │ ◄───────────────────────────► │  (Docker container │
│  client            │   state topics / cmd topics   │   or on-robot)     │
└────────────────────┘                               └─────────┬──────────┘
                                                               │
                                              RunType: 1       │      RunType: 0
                                          ┌────────────────────┴───────────────┐
                                          ▼                                    ▼
                                 ┌─────────────────┐                 ┌───────────────────┐
                                 │   Real robot    │                 │  MuJoCo simulator │
                                 │  (FSA actuators)│                 │  (sim/mujoco)     │
                                 └─────────────────┘                 └───────────────────┘
                                                                       ▲ LCM channels:
                                                                       │ msg_simulator_to_controller
                                                                       │ msg_controller_to_simulator
```

Three processes are involved:

1. **Your client program** uses the `fourier_aurora_client` package. It publishes commands and reads robot state over **DDS** (Fast DDS). `AuroraClient.get_instance()` accepts an optional `participant_qos` (`fastdds.DomainParticipantQos`) and can add ROS-compatible type mangling (`is_ros_compatible=True`) plus a `namespace` prefix, so it can coexist with ROS 2 systems.
2. **The Aurora server** runs the whole-body controller and FSMs. It is configured by `config/config.yaml` (robot model, hardware revision, sim vs. real, DDS domain ID) and is distributed as a per-robot Docker image.
3. **The plant** is either the real robot or the **MuJoCo simulator** in `sim/`. The simulator communicates with the server over **LCM** (not DDS) — see [Simulation](simulation.md).

The client and server must agree on the DDS **domain ID** (`domain_id=123` by default everywhere in this repo).

## Whole-body FSM states

The Aurora server runs a whole-body finite state machine. You read it with `get_fsm_state()` / `get_fsm_name()` and change it with `set_fsm_state(state)`.

| ID | State | Meaning |
|---|---|---|
| 0 | Default | Idle/initial state. |
| 1 | Joint stand | Stand using joint-space control. |
| 2 | PD stand | Stand using PD control. Stand-pose adjustment (`set_stand_pose`) only works here. |
| 3 | RL locomotion | Walking driven by the built-in reinforcement-learning locomotion policy. Velocity commands (`set_velocity`) apply here. |
| 9 | Security protection | Safety state. |
| 10 | User command | Whole-body joint-level control: your client streams joint targets via `set_group_cmd` (and configures gains via `set_motor_cfg_pd`). |
| 11 | Upper-body user command | Like state 10, but only the upper body is under user control. |

Typical transitions used by the examples:

- **Velocity walking:** `2 (PD stand)` → `3 (RL locomotion)` + `set_velocity_source(2)` → `set_velocity(...)`.
- **Custom whole-body policy (e.g. the `demo_walk` RL demos):** `2 (PD stand)` → `10 (user command)` → `set_motor_cfg_pd(...)` → stream `set_group_cmd`/joint positions at 50–100 Hz.
- **Arm move commands while walking:** `3 (RL locomotion)` + `set_upper_fsm_state(4)` → `set_move_command(...)`.

Let the robot stabilize in PD stand before switching onward; the examples pause for user confirmation ("After the robot is standing firmly on the ground...") between transitions.

## Upper-body FSM states

Independently of the whole-body FSM, the upper body has its own state machine (`get_upper_fsm_state` / `set_upper_fsm_state`):

| ID | State | Meaning |
|---|---|---|
| 0 | Default | Upper body follows the default behavior. |
| 1 | Act | Arm swing during walking. |
| 2 | Remote | Upper body driven by teleoperation/remote input. |
| 4 | Move command | Upper body accepts planned point-to-point `MoveCommand`s (see below). |

## Velocity sources

The locomotion controller takes velocity commands from exactly one source at a time (`get_velocity_source` / `set_velocity_source`):

| ID | Source | Meaning |
|---|---|---|
| 0 | Joystick | Physical joystick paired with the robot (default). |
| 2 | Navigation (DDS) | Velocity commands sent by a client via `set_velocity()`. |

To drive the robot from code you must first call `set_velocity_source(2)`.

## Control groups

The minimum command unit in Aurora is a **control group** — you always command a whole group at once with a list of per-joint values. Group names are strings such as `left_leg`, `right_leg`, `waist`, `head`, `left_manipulator`, `right_manipulator`, and (GR-1P) `left_hand` / `right_hand`. The set of groups and joints per group differs per robot — see [Robot Reference](robots.md).

Groups are used consistently across the API:

- `get_group_state(group, key)` — joint `position` / `velocity` / `effort` for the group.
- `get_cartesian_state(group, key)` — end-effector `pose` (position + quaternion), `twist`, or `wrench`.
- `set_group_cmd(position_cmd, velocity_cmd, torque_cmd)` — dictionaries keyed by group name.
- `set_motor_cfg_pd(kp_config, kd_config)` / `get_group_motor_cfg(group, key)` — per-group gain lists.
- `joint_move_command` / `cartesian_move_command` — planned moves target one group.

> Safety: when streaming `set_group_cmd`, always interpolate from the *measured* current position to the target instead of jumping — every example does linear interpolation over ~2 seconds. Sending a distant target directly commands a step input at your PD gains.

## Command paths: streaming vs. planned moves

There are two distinct ways to move joints:

1. **Streamed joint commands (`set_group_cmd`)** — you run the control loop. Requires FSM state 10 (or 11 for the upper body). The server applies your position/velocity/torque targets with the PD gains configured by `set_motor_cfg_pd`. Used for custom controllers and RL policies at 50–100 Hz.
2. **Planned move commands (`MoveCommandManager` + `set_move_command`)** — the server plans and executes the trajectory. Used from RL locomotion state with `set_upper_fsm_state(4)`. You build a `MotionControlCmd.MoveCommand` with one or more per-group targets:
   - `MOVE_ABS_JOINT` (type 0): joint-space target via `joint_move_command`.
   - `MOVE_JOINT` (type 1): Cartesian target `[x, y, z, qx, qy, qz, qw]`, joint-space interpolation, via `cartesian_move_command`.
   - `MOVE_LINE` (type 2): Cartesian target with straight-line end-effector motion, via `cartesian_move_command`.

   Speed is set with `expect_vel` in thousandths of maximum speed (e.g. `300` = 30%). Track completion with `get_group_motion_state()` or block with `wait_groups_motion_complete([...])`.

## Robot state data

Beyond joint states, the client exposes:

- **Base/IMU data** — `get_base_data(key)` with keys for orientation (`quat_xyzw`, `quat_wxyz`, `rpy`), angular velocity and acceleration in world or base frame (`omega_W`/`omega_B`, `acc_W`/`acc_B`), and base velocity/position (`vel_W`, `vel_B`, `pos_W`).
- **Contact data** — `get_contact_data(key)`: `contact_fz` (force/torque) and `contact_prob` (per-foot contact probability; the status demo treats `> 0.5` as "in contact").
- **Stand pose** — `get_stand_pose()` returns `[delta_z, delta_pitch, delta_yaw, stable_level]` relative to the default standing pose.
