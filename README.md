# OmindOS Manipulation Community

**Community edition for learning robot manipulation, motion planning, grasping, and dual-arm coordination.**

This repository is the community learning edition of the manipulation stack used as a foundation for **OmindOS Manipulation**. It is intended for education, research, prototyping, and community experimentation.

> **Status:** Community v0.1 — the first community release focuses on making the existing UR3e simulation and planning examples easier to understand and explore. The underlying algorithms are intentionally kept close to the original project at this stage.

## What you can learn

A suggested learning path is:

1. **Robot basics and kinematics** — coordinate frames, joint chains, forward/inverse kinematics.
2. **Robot simulation** — build and visualize robot models with Panda3D.
3. **Grasping** — define grippers, grasp poses, and grasp candidates.
4. **Collision checking** — reason about robot, object, and environment collisions.
5. **Motion planning** — generate collision-aware motions, including RRT-Connect examples.
6. **Dual-arm manipulation** — explore handover and coordinated manipulation.
7. **Integrated manipulation** — study a complete dual-arm manipulation / digital-twin example.

## Repository map

The current codebase contains the original research and teaching modules. Important starting points include:

```text
basis/              Mathematical and geometry utilities
grasping/           Grasp-related algorithms and utilities
manipulation/       Manipulation and placement functionality
motion/             Motion-related functionality
motionplanning/     Motion-planning algorithms
robot_sim/          Robot simulation models
modeling/           Geometric and collision models
visualization/      Visualization utilities
pandaplotutils/     Panda3D-related visualization utilities
0000_examples/      Runnable examples
0000_book/          Learning-oriented examples
app/                Integrated application demos
```

Some directory names differ from descriptions in older documentation. This map follows the current repository structure.

## Quick start

### 1. Clone the community repository

```bash
git clone https://github.com/maganrobotics-boop/urmanipulation.git
cd urmanipulation
```

For the v0.1 development snapshot:

```bash
git checkout community-v0.1
```

### 2. Create a Python environment

A dedicated virtual environment is recommended. Install the project dependencies from:

```bash
pip install -r requirements.txt
```

The existing project uses Panda3D for visualization and includes scientific-computing, geometry, vision, and planning dependencies.

> The repository is an evolving research/education codebase rather than a polished binary distribution. Dependency compatibility can vary by Python version and operating system. Community v0.1 will progressively improve reproducibility.

### 3. Run the Community Quick Start

From the repository root:

```bash
python community/quick_start.py
```

This simulation-only example introduces the basic target pose → IK → joint configuration → FK → visualization loop. See [`community/README.md`](community/README.md) for the walkthrough.

### 4. Continue with the existing simulation examples

Explore `0000_examples/` and `0000_book/` before connecting any physical robot.

Representative examples in the existing project include:

- Bullet / dynamics examples
- grasp definition examples
- dual-arm handover examples
- UR3e dual-arm simulation
- robot-math and grasp-planning examples
- RRT-Connect motion-planning examples

For an integrated example, see the demos under `app/`, including the existing roller-hole docking / dual-arm manipulation workflow where available in the current tree.

## Community Demos — from basics to dual-arm manipulation

The Community edition is organized around progressive demos so learners can move from a minimal simulation toward complete manipulation tasks.

| Demo | Topic | What you learn | Status |
|---|---|---|---|
| **Demo 01 — Quick Start** | Kinematics | target pose → IK → joint configuration → FK → visualization | Available |
| **Demo 02 — UR3e Single Arm** | Manipulator basics | UR3e model, joint motion, TCP and end-effector pose | Planned for v0.1 |
| **Demo 03 — Grasping** | Robot grasping | gripper model, grasp pose and grasp candidates | Planned for v0.1 |
| **Demo 04 — Motion Planning** | RRT-Connect | collision-aware path planning from start to goal | Planned for v0.1 |
| **Demo 05 — Dual-arm Coordination** | Dual-arm manipulation | two-arm configuration, handover and coordinated motion | Planned for v0.1 |
| **Demo 06 — Mobile Dual-arm Manipulation** | Mobile manipulation | connect a ROS2/C++ mobile base with Python dual-arm manipulation through stable ROS2 interfaces | Planned |
| **Demo 07 — Integrated Task** | Complete manipulation workflow | combine navigation, grasping, planning, collision checking and dual-arm manipulation | Planned |

Start here:

```bash
python community/quick_start.py
```

Then follow [`community/README.md`](community/README.md) and [`docs/LEARNING_PATH.md`](docs/LEARNING_PATH.md).

The original examples remain available under `0000_examples/`, `0000_book/`, and `app/`. Community demos provide a cleaner learning layer on top of them rather than replacing the upstream research code.

## Community learning route

| Stage | Topic | Goal |
|---|---|---|
| 01 | Robot model | Understand links, joints, frames, and robot representation |
| 02 | Kinematics | Move from joint configurations to end-effector poses and back |
| 03 | Grasping | Generate and inspect grasp candidates |
| 04 | Collision | Check whether a planned state is physically feasible |
| 05 | Planning | Generate collision-aware robot motion |
| 06 | Dual arm | Coordinate two manipulators and study handover |
| 07 | Integrated task | Combine perception/modeling, planning, and manipulation |

The long-term Community goal is to make each stage independently runnable and suitable for coursework, self-study, and research prototyping.

## Real robots

**Start in simulation.**

The Community repository should not be treated as a plug-and-play safety controller for a physical industrial manipulator. Before running generated motions on a real robot, independently validate joint limits, collision constraints, workspace safety, emergency-stop behavior, payload/tool configuration, and the manufacturer's operating requirements.

Hardware adapters and deployment-specific interfaces can evolve separately from the community learning examples.

## Mobile Dual-arm Architecture

OmindOS does **not** require every robotics module to use the same programming language. The integration contract is ROS2.

```text
                  OmindOS
                     |
        +------------+------------+
        |                         |
Navigation Community       Manipulation Community
    ROS2 / C++                    Python
        |                         |
 localization / mapping      grasping / planning
 navigation / base control   single & dual-arm control
        |                         |
        +------------+------------+
                     |
             ROS2 Interface Layer
                     |
          Mobile Dual-arm Robot
```

The design principle is **interface unification, not language unification**:

- Navigation can remain ROS2/C++ for localization, mapping, navigation, and base control.
- Manipulation can remain Python for kinematics, grasping, motion planning, dual-arm coordination, and higher-level experimentation.
- A Python ROS2 adapter (`rclpy`) can expose manipulation capabilities as ROS2 topics, services, and actions.
- Mobile manipulation composes both sides through stable ROS2 contracts rather than merging both codebases into one language.

A future OmindOS integration layer should standardize robot pose/state, navigation goals and results, manipulation goals and results, task status, collision/safety state, and recovery behavior. This allows Navigation and Manipulation implementations to evolve independently while remaining interoperable.

## OmindOS

OmindOS is being organized as modular robot capabilities. This repository represents the **Manipulation Community** track:

```text
OmindOS Navigation Community
        +
OmindOS Manipulation Community
        ↓
Mobile Manipulation / Embodied Robot Applications
```

The Community edition is intended to remain useful on its own for learning and experimentation. Product-specific integrations, hardware deployment, validation, and support may be maintained separately.

## Project origin and attribution

This repository is derived from the **Universal Robots UR3e Planning & Control System** and originates from work based on / inspired by the **WRS** robotics software developed by **Weiwei Wan and collaborators at Osaka University**.

The original repository license and attribution are intentionally preserved. Please retain the applicable copyright and license notices when redistributing derived work.

## License

This repository is distributed under the **MIT License**. See [LICENSE](LICENSE).

The existing LICENSE identifies:

> Copyright (c) 2020 Weiwei Wan

Community documentation and later contributions do not remove or replace the original copyright notice.

## Contributing

Community contributions are welcome, particularly:

- reproducible installation instructions;
- verified examples;
- bug fixes;
- clearer learning materials;
- robot-model adapters;
- simulation examples;
- manipulation and motion-planning improvements.

For substantial changes, prefer a focused issue or pull request with a runnable example and a short explanation of the expected behavior.

---

**OmindOS Manipulation Community v0.1**

Learn in simulation. Validate before hardware. Build reusable robot manipulation capabilities.
