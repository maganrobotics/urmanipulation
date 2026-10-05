# Community demo walkthrough

Install [the Community environment](../docs/ENVIRONMENT.md). Run the commands below
from the repository root. Omit `--headless` to animate the result on a desktop.
Every example is simulation-only and loads meshes relative to their source files.
Generated box objects avoid unpublished datasets and working-directory assumptions.

| Step | Run | Expected result | Read in the code |
|---|---|---|---|
| 01 | `python -m community.quick_start --headless` | IK position residual below 2 mm | `JLChain`, TCP definition, `ik`, `fk` |
| 02 | `python -m community.ur3e_single_arm --headless` | IK residual below 1 mm; 7 collision links | FK-generated reachable target; original numerical IK |
| 03 | `python -m community.grasping --headless` | At least one grasp; seed 7 gives 6 candidates | Jaw pose, rotation sweep, gripper/box mesh checks |
| 04 | `python -m community.motion_planning --headless` | Valid path and a positive blocking-obstacle check | Original `RRTConnect`, explicit endpoint and edge validation |
| 05 | `python -m community.dual_arm --headless` | 12-joint synchronized path passes collision checks | `both_arm`, cross-arm and body collision pairs |
| 06 | `python -m community.mobile_dual_arm --headless` | 31 base poses; task reaches `ready` | `map` base transform and TCP transform consistency |
| 07 | `python -m community.integrated_task --headless` | Two arm plans, two objects released; task succeeds | Grasp filtering, hold, original RRT, object pose updates, release |

## What changes between examples

Single-arm IK starts near a known reachable target. It reports the actual residual
instead of treating any returned joint vector as a successful solution. The original
IK's weighted stopping tolerance is retained, so this is not a submillimetre accuracy guarantee.

Grasping uses collision-filtered geometric candidates. Jaw width has a small clearance;
there is no contact or force-closure solver in this exercise.

RRT-Connect uses the original algorithm with smoothing disabled: a seed-7 run of the
historical smoother changed the requested terminal configuration. Community independently
samples all returned edges at a maximum 0.025 rad per joint and rejects an invalid
path. This sampled test is not continuous collision detection or trajectory time parameterization.

The fixed-base example synchronizes joint motion; it does not constrain both arms
to a shared rigid object. The mobile example transforms the complete existing rig
along a supplied mock route. It does not run Navigation Community or Nav2 locally.

The integrated example picks boxes already at the jaw centres, moves each arm while
retaining attached-object geometry, and releases both boxes. It preserves the first
attachment while the second arm moves. This is a geometric task, not a simulated
industrial cell with a table, gravity, perception or grasp approach trajectory.

## Run all checks

```bash
python -m unittest community.test_contract
python -m community.verify
```

The verifier runs each demo in its own process with a timeout, writes per-demo logs
and JSON results, and produces `community-results/summary.json`. A failed check
cannot reuse a stale successful result. `community-results/ros2-playback.json`
contains the checked twelve-joint playback for the separate ROS2 transport test.

See [ROS2 instructions](../docs/ROS2_INTERFACE.md) and
[the recorded verification](../docs/COMMUNITY_VALIDATION.md).
