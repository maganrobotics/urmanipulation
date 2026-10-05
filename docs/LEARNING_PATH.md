# Community Learning Path

This page provides a stable entry point into the existing examples without renaming or rewriting the upstream code in Community v0.1.

## Recommended order

### 1. Coordinate frames and kinematic chains

Start with:

```bash
python 0000_book/wrssystem_jlchain.py
```

What to observe:

- how a joint-link chain is constructed;
- revolute/prismatic joint configuration;
- TCP definition;
- inverse kinematics;
- forward kinematics and visualization.

### 2. Motion planning with RRT-Connect

Then study:

```bash
python 0000_book/motion_rrtconnect.py
```

This example uses a simple two-DOF XY robot so the planning result is easy to understand before moving to a full manipulator.

Look for:

- start and goal configurations;
- obstacle representation;
- collision checking;
- extension distance and planning time;
- the final path.

### 3. Grasping

Relevant learning examples include:

```text
0000_book/grasping_defined.py
0000_book/grasping_antipodal_planning.py
0000_book/grasping_stepbystep.py
0000_examples/rtq85_define_grasps.py
```

Some older examples depend on local data files or relative working-directory assumptions. Treat them as learning/reference examples until they are marked verified.

### 4. UR3e dual-arm simulation

Study:

```bash
python 0000_examples/ur3e_dual_dynamics.py
```

This example builds the UR3e dual-arm model and connects it with the Bullet dynamics layer.

### 5. Handover and coordinated manipulation

Study:

```text
0000_examples/test_handover.py
```

This is a useful code-reading entry point for dual-arm / object handover concepts. In Community v0.1 it is not yet designated as a clean one-command tutorial because it relies on assets and assumptions inherited from the research codebase.

### 6. Integrated applications

After the fundamentals, inspect:

```text
app/
app/demo/
```

These contain larger application-oriented workflows. They are intentionally placed later in the learning route because they combine multiple subsystems.

## Example maturity labels

Community releases will progressively classify examples as:

- **Verified** — tested with a documented environment.
- **Learning** — useful educational code, but may need environment-specific setup.
- **Research** — research/application code with additional assumptions, hardware, data, or dependencies.
- **Legacy** — retained for reference and upstream compatibility.

Community v0.1 does not claim that every historical example is verified.

## Why we do not reorganize all source folders in v0.1

The existing code has import relationships and research history tied to the current layout. Renaming folders aggressively would make the repository look cleaner while increasing the risk of breaking working code.

The v0.1 strategy is therefore:

1. preserve the algorithm/source tree;
2. add a clear learning layer;
3. verify examples incrementally;
4. refactor only when tests or reproducible demonstrations protect behavior.
