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
| ROS2 action transport | Awaiting the dedicated Humble CI job; this local host has no ROS2 installation |
| Physical hardware / C++ Navigation stack | Not tested |

Machine-readable results: [Linux report](validation/linux-py312.json).
Observed package versions: [environment snapshot](validation/pip-freeze-linux-py312.txt).
The CI workflow preserves per-demo logs and JSON plus ROS2 results as build artifacts.

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
