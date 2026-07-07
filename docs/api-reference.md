# API Reference

Detailed reference for the `fourier_aurora_client` Python package (v0.1.8). It covers the two public classes:

- [`AuroraClient`](#auroraclient) — connection, state queries, and commands.
- [`MoveCommandManager`](#movecommandmanager) — builder for planned point-to-point move commands.

Conventions used throughout:

- Angles are in **radians**, distances in **meters**, velocities in **m/s** / **rad/s**.
- Quaternions are `[x, y, z, w]` unless a key says otherwise.
- "Group" means a control group name (e.g. `left_manipulator`); see [Robot Reference](robots.md) for each robot's groups and joint ordering.

---

## AuroraClient

### Lifecycle

#### `get_instance`

```python
AuroraClient.get_instance(
    domain_id: int,
    participant_qos=None,
    robot_name: Optional[str] = None,
    namespace: Optional[str] = None,
    is_ros_compatible: bool = False,
) -> AuroraClient
```

Creates (or returns) the client instance and joins the DDS domain.

| Parameter | Type | Description |
|---|---|---|
| `domain_id` | `int` | DDS domain ID. **Must match `DomainID` in the server's `config.yaml`** (default `123`). |
| `participant_qos` | `fastdds.DomainParticipantQos`, optional | Custom QoS for the DDS DomainParticipant. |
| `robot_name` | `str`, optional | Robot model name: `"gr2"`, `"gr3"`, `"fouriern1"`, or `"gr1t2"` for GR-1P. |
| `namespace` | `str`, optional | Namespace prefix applied after the ROS-compatible prefix. |
| `is_ros_compatible` | `bool`, optional | Add ROS-typed name mangling so topics interoperate with ROS 2. |

Returns the `AuroraClient` singleton.

Notes:
- Sleep ~1 second after construction before issuing commands, so DDS discovery can complete (all examples do this).
- The GR-1P examples also pass `serial_number=None`; supplying a serial number scopes the client to a specific robot unit.

```python
client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
time.sleep(1)
```

#### `close`

```python
AuroraClient.close()
```

Shuts down the client and leaves the DDS domain. Always call this on exit (including exception paths — the walking demos call it from a `finally:` block).

---

### State getters

#### `get_fsm_state` / `get_fsm_name`

```python
AuroraClient.get_fsm_state() -> int
AuroraClient.get_fsm_name() -> str
```

Current **whole-body** FSM state, as an ID or human-readable name.

| ID | State |
|---|---|
| 0 | Default |
| 1 | Joint stand |
| 2 | PD stand |
| 3 | RL locomotion |
| 9 | Security protection |
| 10 | User command |
| 11 | Upper-body user command |

#### `get_upper_fsm_state` / `get_upper_fsm_name`

```python
AuroraClient.get_upper_fsm_state() -> int
AuroraClient.get_upper_fsm_name() -> str
```

Current **upper-body** FSM state.

| ID | State |
|---|---|
| 0 | Default |
| 1 | Act (arm swing) |
| 2 | Remote |
| 4 | Move command |

#### `get_velocity_source` / `get_velocity_source_name`

```python
AuroraClient.get_velocity_source() -> int
AuroraClient.get_velocity_source_name() -> str
```

Which input currently feeds velocity commands to the locomotion controller.

| ID | Source |
|---|---|
| 0 | Joystick |
| 2 | Navigation (DDS — i.e. this client) |

#### `get_stand_pose`

```python
AuroraClient.get_stand_pose() -> list[float]
```

Returns `[delta_z, delta_pitch, delta_yaw, stable_level]` — the current offsets from the default standing pose (m / rad) plus a stability indicator.

#### `get_group_state`

```python
AuroraClient.get_group_state(group_name: str, key: str = 'position') -> list[float]
```

Joint-space state of one control group.

| `key` | Returns |
|---|---|
| `'position'` | Joint positions (rad) |
| `'velocity'` | Joint velocities (rad/s) |
| `'effort'` | Joint efforts/torques |

The returned list has one entry per joint in the group, in the group's fixed joint order.

```python
q = client.get_group_state("left_manipulator", key="position")   # e.g. 7 floats on GR-3
```

#### `get_cartesian_state`

```python
AuroraClient.get_cartesian_state(group_name: str, key: str = 'pose') -> list[float]
```

Cartesian state of a group's end effector.

| `key` | Returns |
|---|---|
| `'pose'` | End pose `[x, y, z, qx, qy, qz, qw]` |
| `'twist'` | End twist |
| `'wrench'` | End wrench |

#### `get_group_motor_cfg`

```python
AuroraClient.get_group_motor_cfg(group_name: str, key: str = 'pd_kp') -> list[float]
```

Reads back the motor gain configuration for a group (one value per joint).

| `key` | Meaning |
|---|---|
| `'pd_kp'` | Proportional gain, PD control mode |
| `'pd_kd'` | Derivative gain, PD control mode |
| `'pos_kp'` | Proportional gain, position control mode |
| `'pos_ki'` | Integral gain, position control mode |
| `'pos_kd'` | Derivative gain, position control mode |

Useful for verifying that a `set_motor_cfg_pd` call was applied (see `demo_joint_command.py`).

#### `get_base_data`

```python
AuroraClient.get_base_data(key: str) -> list[float]
```

Base/IMU state. Frames: `_W` = world, `_B` = base (body).

| `key` | Returns |
|---|---|
| `'quat_xyzw'` | Base orientation quaternion `[x, y, z, w]` |
| `'quat_wxyz'` | Base orientation quaternion `[w, x, y, z]` |
| `'rpy'` | Roll, pitch, yaw (rad) |
| `'omega_W'` / `'omega_B'` | Angular velocity in world / base frame (rad/s) |
| `'acc_W'` / `'acc_B'` | Linear acceleration in world / base frame |
| `'vel_W'` / `'vel_B'` | Base linear velocity in world / base frame (m/s) |
| `'pos_W'` | Base position in world frame (m) |

#### `get_contact_data`

```python
AuroraClient.get_contact_data(key: str) -> list[float]
```

| `key` | Returns |
|---|---|
| `'contact_fz'` | Contact force/torque |
| `'contact_prob'` | Per-foot contact probability (treat `> 0.5` as in contact) |

#### `get_group_motion_state`

```python
AuroraClient.get_group_motion_state(group_name: str) -> int
```

Motion state of a group executing a planned move command (see [`MoveCommandManager`](#movecommandmanager)).

#### `wait_groups_motion_complete`

```python
AuroraClient.wait_groups_motion_complete(
    group_names: list[str],
    print_interval: float = 0.3,
) -> bool
```

Blocks until every listed group has finished its current move command, printing progress every `print_interval` seconds. Returns `True` when all groups completed. Typical usage after `set_move_command`:

```python
client.set_move_command(move_command)
time.sleep(0.2)  # let the command land before polling
client.wait_groups_motion_complete(["left_manipulator", "right_manipulator"])
```

---

### Command setters

#### `set_fsm_state`

```python
AuroraClient.set_fsm_state(state: int)
```

Requests a whole-body FSM transition (state IDs as in [`get_fsm_state`](#get_fsm_state--get_fsm_name)). Give the robot time to complete each transition (`time.sleep(0.5–1.0)` in the examples), and only leave PD stand once the robot is standing firmly.

#### `set_upper_fsm_state`

```python
AuroraClient.set_upper_fsm_state(state: int)
```

Requests an upper-body FSM transition (state IDs as in [`get_upper_fsm_state`](#get_upper_fsm_state--get_upper_fsm_name)). Set state `4` (Move command) before sending `MoveCommand`s, or `1` (Act) to enable arm swing while walking.

#### `set_velocity_source`

```python
AuroraClient.set_velocity_source(source: int)
```

Selects the velocity input: `0` = joystick, `2` = Navigation (DDS). **Client-side `set_velocity` calls only take effect with source `2`.** Consider restoring source `0` when your program exits (the GR-1P usage demo does).

#### `set_velocity`

```python
AuroraClient.set_velocity(vx: float, vy: float, yaw: float, duration: float = 1.0)
```

Commands a base velocity while in RL locomotion state (3).

| Parameter | Meaning |
|---|---|
| `vx` | Forward (+) / backward (−) velocity, m/s |
| `vy` | Left (+) / right (−) velocity, m/s |
| `yaw` | Counter-clockwise (+) angular velocity, rad/s |
| `duration` | How long the command stays valid, seconds (default 1.0) |

The command expires after `duration`; send `set_velocity(0, 0, 0, ...)` to stop deliberately. Actual velocity is limited by the robot's capabilities.

#### `set_stand_pose`

```python
AuroraClient.set_stand_pose(delta_z: float, delta_pitch: float, delta_yaw: float)
```

Adjusts the standing pose **relative to the default pose** — it is an absolute offset, not incremental:

| Parameter | Meaning |
|---|---|
| `delta_z` | Base height change, m (positive = raise, negative = crouch) |
| `delta_pitch` | Pitch change, rad (positive = lean forward) |
| `delta_yaw` | Yaw change, rad (positive = turn left) |

Only available in **PD stand (state 2)**. `set_stand_pose(0, 0, 0)` returns to the default pose. Example: crouch 10 cm with `set_stand_pose(-0.10, 0.0, 0.0)`.

#### `set_group_cmd`

```python
AuroraClient.set_group_cmd(
    position_cmd: Dict[str, list[float]],
    velocity_cmd: Optional[Dict[str, list[float]]] = None,
    torque_cmd: Optional[Dict[str, list[float]]] = None,
)
```

Streams joint-level targets. Each dictionary maps a **group name** to a list with one value per joint in that group. Requires FSM state 10 (whole body) or 11 (upper body).

```python
client.set_group_cmd(position_cmd={
    "left_manipulator": [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
    "waist": [0.5, 0.0, 0.0],
})
```

> **Safety:** never jump to a distant target in a single command. Read the current position with `get_group_state` and linearly interpolate to the target over a couple of seconds (the examples use 200 steps at 100 Hz).

#### `set_motor_cfg_pd`

```python
AuroraClient.set_motor_cfg_pd(
    kp_config: Dict[str, list[float]],
    kd_config: Dict[str, list[float]],
)
```

Sets PD gains per group (keys = group names, values = one gain per joint). Configure gains **after** entering user-command state and **before** streaming `set_group_cmd`. Read back with `get_group_motor_cfg` to verify. Per-robot recommended values are listed in [Robot Reference](robots.md).

#### `set_move_command`

```python
AuroraClient.set_move_command(move_command: MotionControlCmd.MoveCommand)
```

Submits a planned move command built with [`MoveCommandManager`](#movecommandmanager). The upper-body FSM must be in Move command state (`set_upper_fsm_state(4)`).

---

## MoveCommandManager

Builder for `MotionControlCmd.MoveCommand` messages. You add one or more per-group moves, fetch the assembled message, and pass it to `AuroraClient.set_move_command()`.

### Constructor

```python
MoveCommandManager(
    config_path: Optional[Union[str, Path]] = None,
    robot_name: Optional[str] = None,
) -> MoveCommandManager
```

| Parameter | Meaning |
|---|---|
| `config_path` | Robot config directory (optional). |
| `robot_name` | Robot type, e.g. `"gr2"`, `"gr3"`. Defaults to `"gr3"`. |

### Move types

| Value | Name | Space | Interpolation |
|---|---|---|---|
| 0 | `MOVE_ABS_JOINT` | Joint | Joint space (use `joint_move_command`) |
| 1 | `MOVE_JOINT` | Cartesian target | Joint space (use `cartesian_move_command`) |
| 2 | `MOVE_LINE` | Cartesian target | Straight-line end-effector path (use `cartesian_move_command`) |

### `joint_move_command`

```python
joint_move_command(
    move_type: Union[int, MoveType],   # only 0 (MOVE_ABS_JOINT)
    group_name: str,
    group_pos: list[float],            # target joint positions, one per joint
    group_vel: Optional[int] = None,   # target joint velocities
    expect_vel: int = 1000,            # speed, thousandths of max (1000 = 100%)
)
```

Adds a joint-space move for one control group to the pending command.

### `cartesian_move_command`

```python
cartesian_move_command(
    move_type: Union[int, MoveType],   # 1 (MOVE_JOINT) or 2 (MOVE_LINE)
    group_name: str,
    group_pos: list[float],            # target pose [x, y, z, qx, qy, qz, qw]
    group_vel: Optional[int] = None,
    expect_vel: int = 1000,
)
```

Adds a Cartesian move for one control group. `MOVE_JOINT` interpolates in joint space toward the pose; `MOVE_LINE` moves the end effector along a straight line.

### `get_move_command`

```python
get_move_command() -> MotionControlCmd.MoveCommand
```

Returns the assembled `MoveCommand` containing every move added since the last fetch.

### Complete example

```python
from fourier_aurora_client import AuroraClient, MoveCommandManager
import time

client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
mcm = MoveCommandManager(robot_name="gr3")

client.set_fsm_state(3)            # RL locomotion
time.sleep(0.5)
client.set_upper_fsm_state(4)      # upper body: Move command mode

# Joint-space move: both elbows to -1.2 rad at 30% speed
mcm.joint_move_command(move_type=0, group_name="left_manipulator",
                       group_pos=[0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0], expect_vel=300)
mcm.joint_move_command(move_type=0, group_name="right_manipulator",
                       group_pos=[0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0], expect_vel=300)

client.set_move_command(mcm.get_move_command())
time.sleep(0.2)
client.wait_groups_motion_complete(["left_manipulator", "right_manipulator"])

# Cartesian straight-line move of the left hand
mcm.cartesian_move_command(move_type=2, group_name="left_manipulator",
                           group_pos=[0.3, 0.25, 0.3, 0.0, -0.7071, 0.0, 0.7071],
                           expect_vel=300)
client.set_move_command(mcm.get_move_command())
time.sleep(0.2)
client.wait_groups_motion_complete(["left_manipulator"])

client.close()
```

---

## Notes and caveats

- **`set_joint_positions`** — the `demo_walk.py` scripts call `client.set_joint_positions({"whole_body": [...]})` to command all joints at once via a `whole_body` pseudo-group. This method is used by the shipped examples but is not part of the documented API above; prefer `set_group_cmd` for group-level control unless you are following the walking-policy demo pattern.
- **Singleton client** — `get_instance` follows a get-instance pattern; create it once per process.
- **Timing** — the examples consistently insert small sleeps after mode switches (0.5–1.0 s) and after submitting move commands (0.2 s) before polling for completion. Match this pattern.
