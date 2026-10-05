# Community learning path

Follow the runnable [Community walkthrough](../community/README.md) in order:

1. Joint chain and coordinate frames — Quick Start.
2. UR3e six-joint model — FK, IK, TCP and measured pose error.
3. Grasp candidates — jaw pose, opening and collision filtering.
4. RRT-Connect — start/goal, obstacles and independent path validation.
5. Fixed dual arms — synchronized motion and inter-arm collision checks.
6. Mobile dual arms — base/world transforms and navigation-result gating.
7. Integrated task — two geometric grasps, attached-object transfer and release.
8. ROS2 transport — export the checked playback and run the isolated action bridge.

For deeper code reading, the original `0000_book/wrssystem_jlchain.py`,
`0000_book/motion_rrtconnect.py`, `0000_book/grasping_defined.py`,
`0000_examples/ur3e_dual_dynamics.py`, `0000_examples/test_handover.py` and
`app/demo/roller_hole_docking.py` are retained. They are historical teaching/research
references with their own data and environment assumptions, not additional verified
one-command Community demos.

The Community layer preserves original algorithm modules and MIT/WRS attribution.
It does not replace them with a separate teaching-only planner.
