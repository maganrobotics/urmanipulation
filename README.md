# OmindOS Manipulation Community v0.1

Simulation examples for learning UR3e kinematics, grasping, collision checking,
RRT-Connect, fixed dual arms and mobile manipulation task composition.

**Status: v0.1 source ready for release.** The seven Community examples (the original
Quick Start plus six new demos), five contract tests and three ROS2 Action cases
passed [GitHub CI](https://github.com/maganrobotics-boop/urmanipulation/actions/runs/37375913047).
Local ROS2 communication also passed three same-host shared-memory checks.
Navigation in the offline mobile examples is explicitly simulated. Physical
robots, contact dynamics and production deployment are outside this release.
See [verification results](docs/COMMUNITY_VALIDATION.md) for the exact tested scope.

## Start here

```bash
git clone --branch community-v0.1 https://github.com/maganrobotics-boop/urmanipulation.git
cd urmanipulation
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m community.verify
```

The verified reference environment is Ubuntu 24.04, CPython 3.12, NumPy 1.26.4
and Panda3D 1.10.15. `requirements-community.txt` is for these demos;
`requirements.txt` preserves the historical application environment.
Windows and macOS have not been verified. See [environment notes](docs/ENVIRONMENT.md).

To open a visualization window on a desktop:

```bash
python -m community.ur3e_single_arm
```

Add `--headless` to compute and check results without a display. Each demo supports
`--seed 7` and `--output community-results/result.json`. A failure exits nonzero.
Run the module commands from the repository root; the original
`python community/quick_start.py --headless` entry point also remains available.

## Community Demos

| Demo | Command after `python -m community.` | What it actually checks | Offline status |
|---|---|---|---|
| 01 — Quick Start | `quick_start --headless` | Joint chain, target → IK → FK residual | Verified |
| 02 — UR3e Single Arm | `ur3e_single_arm --headless` | UR3e meshes, six joints, TCP, IK and self collision | Verified |
| 03 — Grasping | `grasping --headless` | Robotiq85 grasp candidates on a generated box, mesh collision filtering | Verified |
| 04 — RRT-Connect | `motion_planning --headless` | Original planner, obstacle detection, endpoints, joint limits and densely sampled edges | Verified |
| 05 — Fixed Dual Arm | `dual_arm --headless` | Twelve-joint coordinated motion and original dual-arm collision model | Verified |
| 06 — Mobile Dual Arm | `mobile_dual_arm --headless` | Simulated base route, rigid transforms, collision checks, navigation result gate | Verified; mock navigation |
| 07 — Integrated Task | `integrated_task --headless` | Simulated navigation, geometric grasps, two sequential arm plans, attached objects, release | Verified; mock navigation |

The mobile example relocates the existing dual-arm rig as a kinematic assembly;
it does not add a wheel model, localization, or a mobile-base dynamics engine.
The integrated task begins with boxes at known reachable grasp poses. It does not
claim perception, approach planning, contact-force simulation, or a dual-arm handover.

Follow the [demo walkthrough](community/README.md) and [learning path](docs/LEARNING_PATH.md).

## ROS2 interface: Navigation C++, Manipulation Python

Navigation remains ROS2/C++; manipulation remains Python. They exchange goals,
results and state through standard ROS2 interfaces. No language rewrite is required.

| Capability | ROS2 interface | Community endpoint |
|---|---|---|
| Navigation goal and result | `nav2_msgs/action/NavigateToPose` | `/omindos/community/navigate_to_pose` |
| Ordered twelve-joint playback | `control_msgs/action/FollowJointTrajectory` | `/omindos/community/follow_joint_trajectory` |
| Base pose and stopped-state evidence | `nav_msgs/msg/Odometry` | `/omindos/community/odom` |

The repository includes a Python action client and an **isolated mock action server**.
It exports an offline checked trajectory, sends a navigation action, requires a fresh
stopped-base observation, then sends the arm trajectory. This is a transport exercise:
the mock server acknowledges accelerated playback and does not drive a controller.
Gripper attachment/release remains in the offline geometric demo.

See [ROS2 setup, commands and limits](docs/ROS2_INTERFACE.md). The GitHub verification
workflow separately checks numerical simulation and real ROS2 message transport.
Both jobs passed: Ubuntu 24.04/Python 3.12 simulation and Ubuntu 22.04/ROS2 Humble
standard localhost DDS action communication. The servers are mocks; cross-machine
networking and the actual C++ Navigation stack remain outside the verified scope.

## Repository map

```text
community/          Community entry points, adapters, validation and ROS2 mock bridge
basis/              Original mathematical and geometry utilities
robot_sim/          Original robot, joint-chain and gripper models
modeling/           Geometric and collision models
grasping/           Grasp annotation and planning
motion/             Current motion/planning modules used by Community demos
motionplanning/     Historical motion-planning modules
visualization/      Panda3D visualization
0000_examples/      Historical examples
0000_book/          Historical teaching examples
app/                Integrated research/hardware applications
docs/               Installation, learning, interface and validation notes
```

Historical examples are retained and are **not all verified**. Community does not
rename the upstream source tree or import hardware drivers. The original research
IK, geometry and RRT algorithms remain in place; small Python/NumPy compatibility
edits are itemized in [the release notes](docs/RELEASE_v0.1.md).

## Real robots

Start in simulation. This release is not a physical-robot safety controller.
Hardware integration needs its own driver mapping, joint limits, tool/payload
calibration, collision validation and manufacturer-required safety checks.

## Project origin and attribution

This repository is derived from the **Universal Robots UR3e Planning & Control System**
and originates from work based on / inspired by the **WRS** robotics software developed
by **Weiwei Wan and collaborators at Osaka University**.

The original repository license, source author comments and attribution are preserved.
Please retain the applicable notices when redistributing derived work. Original WRS
and UR3e work is not reattributed to OriginMind.

## License and contributing

This repository is distributed under the **MIT License**. See [LICENSE](LICENSE):

> Copyright (c) 2020 Weiwei Wan

Community documentation and later contributions do not remove or replace this notice.
OmindOS Community additions are maintained by OriginMind under the same MIT license.

Contributions should include a reproducible command, environment details and the
expected output. Run `python -m community.verify` and
`python -m unittest community.test_contract` before proposing changes to a demo.
