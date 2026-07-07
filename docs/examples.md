# Examples Guide

Walkthrough of every script in `python/example/`. There is one folder per robot; the scripts follow the same patterns across robots, differing mainly in `robot_name`, group sizes, and gains (see [Robot Reference](robots.md)).

All examples assume:

- The Aurora server is running (Docker or on-robot) with `DomainID: 123`.
- If simulating, the MuJoCo simulator is running (see [Simulation](simulation.md)).
- `fourier_aurora_client==0.1.8` is installed.

Most scripts pause with `input("Press Enter ...")` before each stage so you can verify the robot is stable before proceeding. **On a real robot, treat every Enter press as a go/no-go checkpoint.**

## Script matrix

| Script | gr3 | gr2 | fouriern1 | gr1p | Purpose |
|---|:-:|:-:|:-:|:-:|---|
| `demo_quick_start.py` | ✓ | ✓ | ✓ | — | Minimal stand → walk flow |
| `demo_robot_status.py` | ✓ | ✓ | ✓ | — | Read-only state monitoring loop |
| `demo_motion_command.py` | ✓ | ✓ | ✓ | — | Stand-pose adjustment + velocity commands |
| `demo_joint_command.py` | ✓ | ✓ | ✓ | — | Streamed joint control with PD gain setup |
| `demo_move_command.py` | ✓ | ✓ | — | — | Planned joint/Cartesian moves via `MoveCommandManager` |
| `demo_walk.py` | ✓ | ✓ | ✓ | — | Custom RL walking policy at 50 Hz (joystick required) |
| `demo_client_usage.py` | — | — | — | ✓ | GR-1P tour: stand, crouch, walk, arm swing |
| `demo_get_state.py` | — | — | — | ✓ | GR-1P state polling loop |
| `demo_move_joints.py` | — | — | — | ✓ | GR-1P interpolated joint moves incl. hands |

---

## `demo_quick_start.py` — minimal walking

The smallest useful program:

1. `set_fsm_state(2)` — PD stand.
2. `set_fsm_state(3)` — RL locomotion, then `set_velocity_source(2)` to take velocity control away from the joystick.
3. `set_velocity(0.2, 0.0, 0.0, 3.0)` — walk forward at 0.2 m/s for up to 3 s.
4. `close()`.

## `demo_robot_status.py` — state monitoring

A read-only 2 Hz loop that never changes robot state — safe to run any time. It prints:

- base orientation `get_base_data('rpy')` and world velocity `get_base_data('vel_W')`;
- foot contact from `get_contact_data('contact_prob')` (probability > 0.5 → "Yes");
- left arm joint positions (`get_group_state`) and end-effector pose (`get_cartesian_state`).

## `demo_motion_command.py` — stand pose and velocity

1. In **PD stand (2)**: `set_stand_pose(-0.10, 0, 0)` crouches 10 cm, then `set_stand_pose(0, 0, 0)` returns to default. Reads back with `get_stand_pose()`.
2. Switch to **RL locomotion (3)** and `set_velocity_source(2)`.
3. Sequence of `set_velocity` calls: forward 0.2 m/s, left 0.2 m/s, rotate 0.4 rad/s (each for 5 s), then stop with `set_velocity(0, 0, 0, 1.0)`.

## `demo_joint_command.py` — streamed joint control

Demonstrates the **user command** path for the upper body:

1. `set_fsm_state(10)` — user command state.
2. `set_motor_cfg_pd(kp_config, kd_config)` — per-robot gains (see [Robot Reference](robots.md)), then verifies with `get_group_motor_cfg('left_manipulator', 'pd_kp'/'pd_kd')`.
3. Reads current positions of the arms and waist, then **linearly interpolates** to a target pose over 200 steps at 100 Hz using `set_group_cmd(position_cmd=...)`.
4. Interpolates back to zero the same way and prints the final state.

The interpolation pattern is the important takeaway — direct joint commands should always ramp from the measured position:

```python
for step in range(total_steps + 1):
    cmd = {g: [c + (t - c) * step / total_steps
               for c, t in zip(current[g], target[g])] for g in target}
    client.set_group_cmd(position_cmd=cmd)
    time.sleep(0.01)
```

## `demo_move_command.py` — planned moves (gr3, gr2)

Demonstrates server-side planned motion while the robot balances in RL locomotion:

1. `set_fsm_state(3)` then `set_upper_fsm_state(4)` (Move command mode).
2. **`MOVE_ABS_JOINT` (type 0)** via `joint_move_command`: both elbows to a bent pose at `expect_vel=300` (30% speed). Blocks on `wait_groups_motion_complete(["waist", "head", "left_manipulator", "right_manipulator"])`.
3. **`MOVE_JOINT` (type 1)** via `cartesian_move_command`: both hands to a Cartesian pose `[x, y, z, qx, qy, qz, qw]` with joint-space interpolation.
4. **`MOVE_LINE` (type 2)**: raises both hands 10 cm along a straight-line end-effector path.

After each stage it prints joint positions and `get_cartesian_state(..., 'pose')` so you can compare commanded vs. achieved poses. Each `set_move_command` is followed by `time.sleep(0.2)` before waiting for completion.

## `demo_walk.py` — custom RL walking policy (joystick required)

The most advanced demos: they bypass the built-in locomotion controller and run a TorchScript walking policy on the client at 50 Hz, streaming whole-body joint targets.

Common structure (all robots):

1. **Joystick** — `pygame` reads left stick (vx, vy) and right stick (yaw) in a background thread at 50 Hz. The script exits if no joystick is present.
2. **FSM setup** — PD stand (2) → wait for the robot to be firmly standing → user command (10).
3. **Gains** — `set_motor_cfg_pd` for all groups.
4. **Policy** — loads the TorchScript file from the script's directory (`wbc_yaw.pt` for gr3, `policy_jit.pt` for gr2, `policy_jit_walk.pt` for fouriern1) with `torch.jit.load(..., map_location='cpu')`.
5. **Control loop** (`ischedule` at `CTRL_DT = 0.02`, i.e. 50 Hz):
   - Read IMU (`quat_xyzw`, `omega_B`) and all group positions/velocities.
   - Build the observation vector (projected gravity via quaternion rotation, scaled joint states, velocity commands, previous action; the gr3 version stacks a 40-frame history of 8 observation terms).
   - Run the policy, clamp actions, convert to joint targets (`action * scale + default_pose`).
   - Command all joints at once: `client.set_joint_positions({"whole_body": [...]})`.
6. **Shutdown** — on Ctrl-C, stops the joystick thread, returns to PD stand (2), and closes pygame and the client.

Robot-specific details:

- **gr3** (`wbc_yaw.pt`, configured for **gr3v224**): policy controls only the 12 leg joints; arms/waist/head are held at the default pose. Adds a velocity-command low-pass filter, a walk/stand flag with rising/falling edge holds, height-dependent velocity clipping, a yaw-tracking correction, and a torque-limit clamp on the commanded leg positions (`est_torque = kp * Δq`, clipped to per-joint limits, then converted back to positions).
- **gr2** (`policy_jit.pt`): policy controls 13 joints (legs + waist) with explicit per-joint action min/max clipping.
- **fouriern1** (`policy_jit_walk.pt`): policy controls 13 joints (legs + waist); velocity ranges vx ±0.2, vy ±0.5, yaw ±1.0 scaled by a 0.9 safety ratio.

> These demos require `torch`, `pygame`, `ischedule`, and `numpy`, and a joystick connected to the machine running the client.

## GR-1P examples

GR-1P uses `robot_name="gr1t2"` and passes `serial_number=None` to `get_instance`.

### `demo_client_usage.py`

A guided tour: PD stand (2) → crouch/stand via `set_stand_pose(-0.2/0.0, 0, 0)` → RL locomotion (3) + `set_velocity_source(2)` → walk forward 0.2 m/s → enable arm swing with `set_upper_fsm_state(1)` → stop, restore `set_velocity_source(0)` (joystick), and close.

### `demo_get_state.py`

1 Hz polling loop printing FSM states, velocity source, stand pose, base velocities (`vel_B`, `omega_B`), left leg joint state, and left arm Cartesian state.

### `demo_move_joints.py`

Interpolated joint control including the dexterous hands:

- Moves the left arm to `[-0.2, 0, 0, -1.2, 0, 0, 0]` over 2 s (200 × 10 ms steps).
- Closes the left hand from the open pose `[0.2, 0.2, 0.2, 0.2, 1.2, 0.0]` to `[1.7, 1.7, 1.7, 1.7, 0.0, 0.0]` over 1 s.
- Returns **all eight groups** (legs, waist, head, arms, hands) to their initial poses simultaneously with a shared `move_joints` helper at 100 Hz.

The script's docstring states the guiding rule: the minimum control unit is the control group, and for safety any direct joint command should go through an interpolator.
