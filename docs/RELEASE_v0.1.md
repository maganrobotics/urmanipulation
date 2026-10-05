# OmindOS Manipulation Community v0.1

## Release scope

This release candidate adds a runnable Community learning layer over the original
WRS/UR3e code. Quick Start and six progressive demos cover UR3e, grasping,
RRT-Connect, fixed dual arms, a kinematic mobile assembly and an integrated geometric task.

Included:

- Simulation dependency pins and recorded Linux environment.
- One-command headless verification, per-demo JSON/logs and nonzero failures.
- Independent FK/IK residual checks, endpoint/joint-limit/edge collision checks.
- Procedural objects; all UR3e/gripper meshes remain in the source repository.
- Python ROS2 action client and isolated mock server for cross-language integration.
- Navigation-result/stopped-base gate, failure propagation and cancellation requests.
- CI jobs for Python simulation and ROS2 Humble action transport.
- Installation, demo, interface and validation documentation.

## Preservation and compatibility

`LICENSE` is unchanged: MIT, Copyright (c) 2020 Weiwei Wan. Original WRS/UR3e
attribution and author comments are preserved. The original IK and RRT algorithms
remain the implementation used by the demos. Compatibility edits are limited to:

1. Builtin equivalents for removed `np.bool`, `np.int`, `np.float` aliases in the
   vendored `basis/trimesh` copy.
2. Integer node-ID upper bounds in `motion/probabilistic/rrt.py` and `rrt_connect.py`.
3. Lazy QPanda3D import for optional Qt embedding.

Community adapters activate the existing single-arm collision setup and handle
mobile base relocation without rewriting the original robot modules. Historical
requirements, assets, app directories and core numerical algorithms are retained.

## Validation and release checklist

- [x] Seven Community entry points run headlessly in the recorded environment.
- [x] Dependency integrity and task-contract regression checks pass.
- [x] Model meshes load and dual-arm geometry renders offscreen.
- [x] README commands, attribution and release boundaries reviewed.
- [ ] ROS2 Humble transport CI run recorded (see current validation report).
- [ ] Optional desktop OS acceptance; physical robots are outside v0.1 scope.

See [exact verification results](COMMUNITY_VALIDATION.md). No final tag or public
GitHub Release is created merely by preparing this candidate. After the branch's
required checks pass, maintainers can review and tag the source snapshot as `v0.1`.

## Known limits

Offline navigation is simulated; the existing fixed rig is moved kinematically.
The ROS2 mock acknowledges accelerated trajectory playback and does not execute
hardware or gripper commands. The integrated example starts with known reachable
grips and excludes perception, approach planning, contact dynamics and handover.
Collision checking samples joint-space paths, not a continuous swept volume.
Only the Community examples are verified, not all historical research applications.
