# ROS2 interface for Navigation and Manipulation

Navigation stays C++/ROS2; manipulation stays Python. The teaching bridge uses
standard ROS2 actions and messages. It contains a real `rclpy` client and mock action
servers, not a second navigation or manipulation algorithm.

| Interface | Topic/action | Contract |
|---|---|---|
| `nav2_msgs/action/NavigateToPose` | `/omindos/community/navigate_to_pose` | Target `PoseStamped` in `map`; SI metres, planar quaternion; action terminal status determines success |
| `nav_msgs/msg/Odometry` | `/omindos/community/odom` | Base pose in `map`, child `base_link`; client waits for a new stopped observation after navigation success |
| `control_msgs/action/FollowJointTrajectory` | `/omindos/community/follow_joint_trajectory` | 12 named joint positions in radians; strictly increasing time from start |

Joint order: `left_` then `right_`, each followed by `shoulder_pan_joint`,
`shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.
These are Community contract names; real UR drivers need explicit configuration/mapping.

Task order is `idle → navigating → ready → manipulating → succeeded`. Navigation
failure or a moving/stale base blocks manipulation. Rejection, timeout and unsuccessful
arm results terminate the client with a nonzero exit. Active goals receive a bounded
cancellation request when the client is interrupted or times out. This is not a hardware stop guarantee.

## Prepare the checked offline playback

In the simulation environment:

```bash
python -m community.integrated_task --headless --plan-output community-results/ros2-playback.json
```

The JSON schema is `omindos.community.playback.v0.1` with `simulation_only: true`,
a `map` base goal, ordered `joint_names`, and `points` containing `time_s` and
`positions`. The bridge rejects a different schema, nonfinite values, mismatched
joint order or nonincreasing timestamps.

## Run ROS2 transport separately

Use a ROS2 Humble host with `rclpy`, `nav2_msgs`, `control_msgs`, `nav_msgs`,
`trajectory_msgs` and `action_msgs` installed. On Ubuntu 22.04 with ROS2 already set up:

```bash
sudo apt-get install ros-humble-nav2-msgs ros-humble-control-msgs
source /opt/ros/humble/setup.bash
# Use ROS2's system Python, outside the simulation venv.
python3 -m community.ros2_verify --plan community-results/ros2-playback.json
```

This runs three actual action-transport cases in isolated ROS domains: success,
navigation failure (arm must not start), and manipulation failure. Results and logs
are saved under `community-results/ros2/`.

For manual inspection, use the same environment in both terminals:

```bash
export ROS_DOMAIN_ID=81
export ROS_LOCALHOST_ONLY=1
python3 -m community.ros2_bridge mock-server
# In the other terminal:
python3 -m community.ros2_bridge client --plan community-results/ros2-playback.json
```

## Scope and production integration

The mock server acknowledges accelerated playback, emits stationary mock odometry,
and returns action feedback. It does not run Nav2, drive motors, execute gripper
commands, simulate dynamics, or claim to replay the entire grasp task physically.
The offline WRS demo provides geometric validation; this separate process checks
cross-language ROS2-compatible transport. C++ Navigation can implement/use the same
action type without rewriting the Python planner.

A real mobile manipulator additionally needs timestamp/TF validation for `map → odom →
base_link → arm_base`, calibrated mounts, controller joint names and limits,
time parameterization, gripper actions, tool/payload setup, monitored base stopping,
whole-robot collision validation and independent safety functions. Those are outside
this release. Community endpoints intentionally do not default to real controller names.

Reference definitions:
- [Nav2 Humble NavigateToPose](https://api.nav2.org/actions/humble/navigatetopose.html)
- [ros2_control Humble trajectory controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

Check [validation status](COMMUNITY_VALIDATION.md) before relying on transport results.
