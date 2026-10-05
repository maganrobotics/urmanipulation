# Quick Start

The first Community exercise intentionally uses a small simulated joint-link chain rather than a physical UR3e.

## Goal

In one example you will see the basic manipulation loop:

```text
target pose
    ↓
inverse kinematics (IK)
    ↓
joint configuration
    ↓
forward kinematics (FK)
    ↓
visualized robot state
```

## Run

From the repository root:

```bash
python community/quick_start.py
```

Expected behavior:

1. the terminal prints an IK solution;
2. a Panda3D visualization window opens;
3. the target coordinate frame and the solved robot configuration are shown;
4. closing the visualization exits the example.

## What to read in the code

Focus on four operations:

- `JLChain(...)` — creates a simple robot kinematic chain;
- `set_tcp(...)` — defines the tool center point;
- `ik(...)` — solves a target pose into joint values;
- `fk(...)` — applies those joint values back to the robot model.

Once this works, continue with [the full learning path](../docs/LEARNING_PATH.md).

## Scope

This is a **simulation-only learning example**. It does not send commands to a physical robot.
