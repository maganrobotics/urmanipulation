# Community v0.1 verification

Recorded 2026-10-06 (Asia/Shanghai). Starting branch commit:
`30ce37feb53c2ff5e77faeeccd2d3e95b0accc8a`.

Reference host: Ubuntu 24.04, CPython 3.12.14, Linux x86_64, fresh `.venv` with
Community pins. Commands: `python -m community.verify`,
`python -m unittest community.test_contract`, `python -m pip check`.

| Check | Measured result |
|---|---|
| Quick Start | Pass; position error 0.00112234 m, rotation-matrix error 0.00099629 |
| UR3e single arm | Pass; position error 0.00090410 m, rotation-matrix error 0.00244977; 7 active collision links |
| Grasping | Pass; 6 collision-filtered box grasp candidates |
| RRT-Connect | Pass; 283 edge samples, valid endpoints/limits, blocking-obstacle positive control |
| Fixed dual arm | Pass; 5 checked 12-joint configurations |
| Mobile dual arm | Pass; 31 base transforms, consistent TCP transform, `ready` gate |
| Integrated task | Pass; arm paths with 389 and 402 checked samples, 2 released objects, successful task state |
| Contract regression checks | 5 passed: state order, failure/stopped-base gate, cancellation, invalid base goal and invalid trajectory |
| Dependency integrity | `pip check` reports no broken requirements |
| Geometry rendering | Panda3D TinyOffscreen rendered the full dual-arm mesh successfully; image visually inspected |
| Desktop window/input | Not tested: this host has no desktop display |
| ROS2 action transport, same-host IPC | Pass; 3/3 actual Action checks on RoboStack Humble / Python 3.14.7 / Fast DDS 2.6.11, with shared-memory-only transport |
| Independent simulation CI | Pass; Ubuntu 24.04 / Python 3.12, 7/7 demos and 5 contract tests |
| Standard localhost ROS2 DDS/action CI | Pass; Ubuntu 22.04 / ROS2 Humble container, 3/3 Action cases and 5 contract tests |
| Cross-machine ROS2 networking | Not tested |
| Physical hardware / C++ Navigation stack | Not tested |

Machine-readable results: [Linux report](validation/linux-py312.json).
Observed package versions: [environment snapshot](validation/pip-freeze-linux-py312.txt).
ROS2 results: [three Action cases](validation/ros2-humble-shm.json).
ROS2 package snapshot: [RoboStack environment](validation/ros2-humble-packages.json).
The CI workflow preserves per-demo logs and JSON plus ROS2 results as build artifacts.

## Independent GitHub CI

[Run 37375913047](https://github.com/maganrobotics-boop/urmanipulation/actions/runs/37375913047)
completed successfully on 2026-10-05 at 21:29 UTC for source commit
`44c3e9c85861ada88b0892c93888a6ca760330f5`. Both jobs passed:

- `simulation`: installs Community dependencies, runs the five contract tests and
  all seven headless demos, and exports the integrated playback plan.
- `ros2-transport`: consumes that plan in `ros:humble-ros-base` on Ubuntu 22.04,
  repeats the five contract tests and passes all three separate-process Action cases
  with the default verifier's localhost DDS configuration.

Artifacts are `community-simulation` and `community-ros2`. Run, job and artifact
identifiers and digests are recorded in [the CI evidence](validation/github-actions.json).
The release-status documentation update follows this tested source commit and
does not change executable code, dependencies or workflow configuration.

This confirms standard same-host discovery and Action communication with mock
servers. The CI run does not determine whether each DDS payload used shared memory
or UDP, and does not exercise cross-machine networking or the actual C++ Navigation stack.

## ROS2 follow-up

The normal local-only UDP discovery attempt failed with `getifaddrs: Operation not
permitted` before the client could discover the server. An isolated RoboStack Humble
environment was then tested with Fast DDS's explicit shared-memory-only XML profile:
no UDP/TCP transport is configured. Three separate-process tests passed:

1. Navigation succeeds, a new stopped-base observation arrives, and the arm action succeeds.
2. Navigation aborts and the task fails without starting manipulation.
3. Manipulation aborts and the task reports failure.

This verifies real ROS2 serialization, discovery, actions, feedback and results over
same-host IPC against mock capability servers. It does not validate UDP networking,
physical devices, or the actual C++ Navigation stack. The shared-memory JSON report
preserves the observation made before CI completed; its queued-CI note is historical.
The successful independent system-ROS2 CI run is recorded separately above.

## Issues found and handled

- Direct Quick Start execution did not put the repository root on the import path.
  The entry point now resolves its own source root; module entry points are preferred.
- Missing optional QPanda3D blocked every geometry import. Qt embedding imports are lazy.
- Removed NumPy aliases in the vendored `basis/trimesh` layer are replaced with their
  equivalent builtin types; integer RRT node-ID bounds work on Python 3.12.
- The original UR3E constructor creates an empty collision checker. The Community
  single-arm adapter explicitly calls the existing UR3E collision setup, and a
  deliberately blocking box confirms detection is active.
- The historical RRT smoother changed the seed-7 terminal configuration. Community
  disables smoothing and validates every returned edge; original algorithms are retained.
- The historical mobile `move_to` uses a removed pose method. The Community adapter
  relocates existing kinematic chains and calls original FK to update grippers and objects.

## Interpretation

Passing these checks demonstrates the stated simulation exercises on the recorded
host, not hardware readiness or continuous collision safety. The original planner
is probabilistic; other seeds/scenes may fail. A failure remains a nonzero exit,
not an automatically accepted motion. The integrated task is a geometric transfer
with known object poses and abstract release, not a contact-physics pick-and-place benchmark.
