# Community v0.1 environment

## Verified reference

Ubuntu 24.04 (Linux x86_64), CPython 3.12.14, using a fresh virtual environment.
Direct dependencies are pinned in `requirements-community.txt`. The full observed
package list is recorded in `docs/validation/pip-freeze-linux-py312.txt`.
That record is an environment snapshot, not a portable cross-platform lockfile.

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m pip check
python -m community.verify
```

Open3D is an existing eager import in the original geometry layer, so its relatively
large wheel and dependencies are still installed even for the small Community demos.
Panda3D and all model meshes are required in headless mode too: it computes actual
model/collision geometry, not a stub calculation. A display is unnecessary for
`--headless`; desktop mode requires a functioning graphical session/OpenGL setup.

No QPanda3D/PyQt5 install is required for standalone simulation. The original optional
Qt embedding entry point imports QPanda3D only when `World.set_parent` is called.
Historical embedded Qt applications need their own environment.

## Troubleshooting

- `No module named community` or `basis`: run `python -m community.<demo>` from the repository root.
- No display: add `--headless`. It does not exercise desktop input/window behavior.
- Missing DAE/STL: clone the full repository; do not copy only the Python demo files.
- NumPy 2 or old unpinned dependencies: recreate the virtual environment from the Community requirements.
- Missing `pkg_resources`: use the pinned setuptools 80.9.0; the vendored historical mesh loader still uses it.
- ROS2 `rclpy` missing: source ROS2 and use its system Python for the standalone transport module.
  A ROS2 Humble binary built for Python 3.10 cannot be imported by the Python 3.12 simulation venv.

## Historical and unverified environments

`requirements.txt` is preserved unchanged. It includes 2021 versions, notebooks,
vision/hardware packages and Windows-only dependencies; it is not the Community installer.
Windows, macOS, historical application demos and hardware drivers have not been
validated by the Community v0.1 suite.

## Separate ROS2 verification environment

Actual Action transport was checked with RoboStack ROS2 Humble, CPython 3.14.7,
rclpy 3.3.21 and Fast DDS 2.6.11 in an isolated environment. It does not share the
NumPy/Panda3D simulation venv. The exported JSON playback is the interface between
the two environments. See [ROS2 commands](ROS2_INTERFACE.md) and
[recorded results](validation/ros2-humble-shm.json).
