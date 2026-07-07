# Robot Reference

Per-robot control groups, joint counts, and the gain/pose values used by the example scripts. All values below are taken directly from the examples in `python/example/`; treat the gains as sane starting points for each robot, not hard limits.

> Joint ordering within a group is fixed. The leg groups on all robots follow the pattern used by the walking demos: hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll (6 joints). Use `sim/mujoco/get_names_and_id.py` against your robot's URDF to print exact joint names and IDs.

## Summary

| Robot | `robot_name` | Total joints | Control groups |
|---|---|---|---|
| GR-3 | `gr3` | 31 | `left_leg` (6), `right_leg` (6), `waist` (3), `head` (2), `left_manipulator` (7), `right_manipulator` (7) |
| GR-2 | `gr2` | 29 | `left_leg` (6), `right_leg` (6), `waist` (1), `head` (2), `left_manipulator` (7), `right_manipulator` (7) |
| Fourier-N1 | `fouriern1` | 23 | `left_leg` (6), `right_leg` (6), `waist` (1), `left_manipulator` (5), `right_manipulator` (5) |
| GR-1P | `gr1t2` | — | `left_leg` (6), `right_leg` (6), `waist` (3), `head` (3), `left_manipulator` (7), `right_manipulator` (7), `left_hand` (6), `right_hand` (6) |

Notes:
- GR-1P is addressed as `robot_name="gr1t2"` in the client (and the server config uses `RobotName: gr1`). Its examples also pass a `serial_number` argument to `get_instance`.
- GR-3 has hardware revisions `v211`, `v222`, `v224` selected via `HardwareType` in `config/config.yaml`. The shipped GR-3 walking demo (`wbc_yaw.pt`) is configured for **gr3v224**.
- GR-1P is the only robot in the examples with dexterous `left_hand`/`right_hand` groups (6 DOF each).

---

## GR-3 (`gr3`)

31 joints across 6 groups. Whole-body order used by `demo_walk.py`:

```
[ left_leg (6) | right_leg (6) | waist (3) | head (2) | left arm (7) | right arm (7) ]
```

### Default joint position (walking demo, gr3v224)

```python
DEFAULT_JOINT_POSITION = [
    -0.2618, 0.0, 0.0, 0.5236, -0.2618, 0.0,   # left leg
    -0.2618, 0.0, 0.0, 0.5236, -0.2618, 0.0,   # right leg
     0.0, 0.0, 0.03,                            # waist
     0.0, 0.0,                                  # head
     0.1,  0.15, -0.25, -0.5, 0.0, 0.0, 0.0,    # left arm
     0.1, -0.15,  0.25, -0.5, 0.0, 0.0, 0.0,    # right arm
]
```

### PD gains — whole-body walking (`demo_walk.py`)

| Group | Kp | Kd |
|---|---|---|
| `left_leg` / `right_leg` | `[200, 200, 120, 200, 115.4, 15]` | `[20, 20, 12, 20, 11.54, 1.5]` |
| `waist` | `[300, 450, 300]` | `[20, 30, 20]` |
| `head` | `[100, 100]` | `[8, 8]` |
| `left_manipulator` / `right_manipulator` | `[200, 200, 50, 200, 30, 30, 30]` | `[10, 10, 2, 10, 0.5, 0.5, 0.5]` |

### PD gains — upper-body joint control (`demo_joint_command.py`)

| Group | Kp | Kd |
|---|---|---|
| `left_manipulator` / `right_manipulator` | `[400, 200, 200, 200, 50, 50, 50]` | `[20, 10, 10, 10, 2.5, 2.5, 2.5]` |
| `waist` | `[200, 300, 200]` | `[10, 15, 10]` |
| `head` | `[100, 100]` | `[10, 10]` |

### Walking demo policy parameters (gr3v224, `wbc_yaw.pt`)

| Parameter | Value |
|---|---|
| Policy-controlled joints | 12 (both legs) |
| Control rate | 50 Hz (`CTRL_DT = 0.02`) |
| Action scale | 0.5 |
| Observation history | 40 frames |
| Height/attitude init / bounds | init `[0.88, 0, 0]`, lower `[0.40, -0.3, -0.4]`, upper `[0.92, 0.5, 0.4]` |
| Max velocity command | `[1.0 m/s, 0.3 m/s, 0.6 rad/s]` |
| Leg torque limits | `[300, 140, 140, 300, 30, 10]` N·m per leg |

---

## GR-2 (`gr2`)

29 joints. Single-joint waist (`waist: [value]`). The walking policy (`policy_jit.pt`) controls **13 joints**: both legs (6+6) plus the waist joint.

### Walking demo defaults (`demo_walk.py`)

```python
default_joint_position = [
    -0.1309, 0.0, 0.0, 0.2618, -0.1309, 0.0,   # left leg
    -0.1309, 0.0, 0.0, 0.2618, -0.1309, 0.0,   # right leg
     0.0,                                       # waist
]
```

Action limits (policy joints, rad):

```python
action_min = [-2.6180, -0.5934, -0.6981, -0.0873, -0.7854, -0.38397,   # left leg
              -2.6180, -1.5708, -1.5708, -0.0873, -0.7854, -0.38397,   # right leg
              -2.6180]                                                  # waist
action_max = [ 2.6180,  1.5708,  1.5708,  2.3562,  0.7854,  0.38397,
               2.6180,  0.5934,  0.6981,  2.3562,  0.7854,  0.38397,
               2.6180]
```

### PD gains — upper-body joint control (`demo_joint_command.py`)

| Group | Kp | Kd |
|---|---|---|
| `left_manipulator` / `right_manipulator` | `[300, 300, 100, 100, 50, 50, 50]` | `[10, 10, 5, 5, 5, 5, 5]` |
| `waist` | `[200]` | `[10]` |
| `head` | `[100, 100]` | `[10, 10]` |

---

## Fourier-N1 (`fouriern1`)

23 joints: two 6-DOF legs, a 1-DOF waist, and two 5-DOF arms (no head group in the examples). The walking policy (`policy_jit_walk.pt`) controls 13 joints (legs + waist).

### Walking demo defaults (`demo_walk.py`)

```python
default_joint_position = [
    -0.2468, 0.0, 0.0, 0.5181, 0.0, -0.2408,   # left leg
    -0.2468, 0.0, 0.0, 0.5181, 0.0, -0.2408,   # right leg
     0.0,                                       # waist
]
```

Velocity command ranges (scaled by a 0.9 safety ratio): vx ∈ [−0.2, 0.2] m/s, vy ∈ [−0.5, 0.5] m/s, yaw ∈ [−1.0, 1.0] rad/s.

### PD gains — upper-body joint control (`demo_joint_command.py`)

| Group | Kp | Kd |
|---|---|---|
| `left_manipulator` / `right_manipulator` | `[50.0] × 5` | `[5.0] × 5` |
| `waist` | `[40.0]` | `[4.0]` |

---

## GR-1P (`gr1t2`)

The GR-1P examples use `AuroraClient.get_instance(domain_id=123, robot_name="gr1t2", serial_number=None)`. Groups commanded in `demo_move_joints.py`:

| Group | Joints | Zero/target example |
|---|---|---|
| `left_leg` / `right_leg` | 6 | `[0]*6` |
| `waist` | 3 | `[0]*3` |
| `head` | 3 | `[0]*3` |
| `left_manipulator` / `right_manipulator` | 7 | `[-0.2, 0, 0, -1.2, 0, 0, 0]` (elbow bend example) |
| `left_hand` / `right_hand` | 6 | open `[0.2, 0.2, 0.2, 0.2, 1.2, 0.0]`, closed `[1.7, 1.7, 1.7, 1.7, 0.0, 0.0]` |

The hand groups are position-controlled like any other group via `set_group_cmd`; the demo interpolates between the open and closed poses at 100 Hz.
