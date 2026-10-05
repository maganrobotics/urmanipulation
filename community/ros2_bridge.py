"""ROS2 action transport exercise. All endpoints are under /omindos/community.

This mock bridge validates and acknowledges offline simulation trajectories;
it does not drive robot hardware or simulate contact forces. Run using the
Python interpreter belonging to the sourced ROS2 installation, not necessarily
the separate simulation virtual environment. Only standard-library imports
and ROS2 packages are needed here.
"""
import argparse
import json
import math
from pathlib import Path
import time

from community.interface import BaseGoal, TaskGate, validate_trajectory

PREFIX = '/omindos/community'


def unpack_points(trajectory):
    return [dict(time_s=p.time_from_start.sec + p.time_from_start.nanosec / 1e9,
                 positions=list(p.positions)) for p in trajectory.points]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('role', choices=['mock-server', 'client'])
    p.add_argument('--plan', type=Path)
    p.add_argument('--fail-navigation', action='store_true', help='Mock-server failure injection')
    p.add_argument('--fail-manipulation', action='store_true', help='Mock-server failure injection')
    p.add_argument('--timeout', type=float, default=30)
    args = p.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error('--timeout must be positive and finite')
    if args.role == 'client' and args.plan is None:
        p.error('client requires --plan from integrated_task --plan-output')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        from control_msgs.action import FollowJointTrajectory
        from nav_msgs.msg import Odometry
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError as exc:
        p.exit(2, f'ROS2 dependencies unavailable: {exc}. Source ROS2 and install nav2_msgs/control_msgs.\n')

    rclpy.init()
    node = Node('omindos_community_' + args.role.replace('-', '_'))
    gate = TaskGate()

    if args.role == 'mock-server':
        # One active goal across both capabilities. Callbacks share a lock
        # because navigation and manipulation must never execute concurrently.
        import threading
        lock = threading.Lock()
        state = dict(busy=False, ready=False, x=0., y=0., yaw=0.)
        odom_pub = node.create_publisher(Odometry, PREFIX + '/odom', 10)

        def publish_odom():
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = state['x']
            msg.pose.pose.position.y = state['y']
            msg.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
            msg.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
            # This transport mock is stationary: no velocity-control command.
            odom_pub.publish(msg)

        def accept_nav(request):
            pose = request.pose
            values = (pose.pose.position.x, pose.pose.position.y,
                      pose.pose.orientation.z, pose.pose.orientation.w)
            with lock:
                if state['busy'] or pose.header.frame_id != 'map' or not all(math.isfinite(v) for v in values):
                    return GoalResponse.REJECT
                if abs(math.hypot(values[2], values[3]) - 1) > 1e-5:
                    return GoalResponse.REJECT
                state['busy'], state['ready'] = True, False
            return GoalResponse.ACCEPT

        def accept_arm(request):
            try:
                validate_trajectory(request.trajectory.joint_names, unpack_points(request.trajectory))
            except (ValueError, KeyError, TypeError):
                return GoalResponse.REJECT
            with lock:
                if state['busy'] or not state['ready']:
                    return GoalResponse.REJECT
                state['busy'] = True
            return GoalResponse.ACCEPT

        def execute_nav(handle):
            result = NavigateToPose.Result()
            try:
                time.sleep(.1)
                if handle.is_cancel_requested:
                    handle.canceled()
                elif args.fail_navigation:
                    handle.abort()
                else:
                    pose = handle.request.pose.pose
                    state.update(x=pose.position.x, y=pose.position.y,
                                 yaw=2 * math.atan2(pose.orientation.z, pose.orientation.w), ready=True)
                    publish_odom()
                    handle.succeed()
                return result
            finally:
                with lock:
                    state['busy'] = False

        def execute_arm(handle):
            result = FollowJointTrajectory.Result()
            try:
                for point in handle.request.trajectory.points:
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return result
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.joint_names = handle.request.trajectory.joint_names
                    feedback.desired = point
                    feedback.actual = point
                    handle.publish_feedback(feedback)
                    time.sleep(.001)  # Accelerated transport playback, not timed control.
                if args.fail_manipulation:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    handle.abort()
                else:
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    handle.succeed()
                return result
            finally:
                with lock:
                    state['busy'], state['ready'] = False, False

        group = ReentrantCallbackGroup()
        servers = [ActionServer(node, NavigateToPose, PREFIX + '/navigate_to_pose',
                     execute_nav, goal_callback=accept_nav, callback_group=group,
                     cancel_callback=lambda _: CancelResponse.ACCEPT),
                   ActionServer(node, FollowJointTrajectory, PREFIX + '/follow_joint_trajectory',
                     execute_arm, goal_callback=accept_arm, callback_group=group,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)]
        timer = node.create_timer(.1, publish_odom)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            for server in servers:
                server.destroy()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    # Client: strict schema and bounded waits; failures terminate the task.
    active = None
    odom = dict(received=0., stopped=False, pose=None)

    def on_odom(msg):
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (v.x, v.y, v.z, w.x, w.y, w.z)
        odom.update(received=time.monotonic(),
                    stopped=all(math.isfinite(x) and abs(x) < .001 for x in values), pose=msg.pose.pose)

    subscription = node.create_subscription(Odometry, PREFIX + '/odom', on_odom, 10)

    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            raise TimeoutError('ROS2 action timed out')
        return future.result()

    def send(client, goal):
        nonlocal active
        if not client.wait_for_server(timeout_sec=args.timeout):
            raise TimeoutError('ROS2 action server unavailable')
        active = wait(client.send_goal_async(goal))
        if not active.accepted:
            active = None
            raise RuntimeError('ROS2 action goal rejected')
        result = wait(active.get_result_async())
        active = None
        return result

    try:
        plan = json.loads(args.plan.read_text(encoding='utf-8'))
        if plan.get('schema') != 'omindos.community.playback.v0.1' or plan.get('simulation_only') is not True:
            raise ValueError('Only Community simulation playback plans are accepted')
        validate_trajectory(plan['joint_names'], plan['points'])
        base_goal = BaseGoal(**plan['base_goal'])
        nav = ActionClient(node, NavigateToPose, PREFIX + '/navigate_to_pose')
        arm = ActionClient(node, FollowJointTrajectory, PREFIX + '/follow_joint_trajectory')
        gate.start_navigation()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = base_goal.frame_id
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(base_goal.x), float(base_goal.y)
        goal.pose.pose.orientation.z = math.sin(base_goal.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(base_goal.yaw / 2)
        result = send(nav, goal)
        completed_at = time.monotonic()
        while odom['received'] <= completed_at and time.monotonic() - completed_at < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
        pose = odom['pose']
        at_goal = pose is not None and math.hypot(pose.position.x - base_goal.x, pose.position.y - base_goal.y) < .01
        if at_goal:
            yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
            yaw_error = math.atan2(math.sin(yaw - base_goal.yaw), math.cos(yaw - base_goal.yaw))
            at_goal = math.isfinite(yaw_error) and abs(yaw_error) < .01
        stopped = odom['received'] > completed_at and odom['stopped'] and at_goal
        gate.navigation_result(result.status == GoalStatus.STATUS_SUCCEEDED, stopped)
        gate.start_manipulation()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = plan['joint_names']
        for point in plan['points']:
            msg = JointTrajectoryPoint()
            msg.positions = [float(q) for q in point['positions']]
            ns = round(point['time_s'] * 1e9)
            msg.time_from_start.sec, msg.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(msg)
        result = send(arm, goal)
        ok = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0
        gate.manipulation_result(ok)
        if not ok:
            raise RuntimeError('Manipulation action failed')
        print(json.dumps(dict(status='passed', transport='ROS2', mock=True, task_history=gate.history)))
    except (Exception, KeyboardInterrupt) as exc:
        if active is not None:
            # Request cancellation, with a separate bounded wait for acknowledgment.
            future = active.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        gate.fail()
        print(json.dumps(dict(status='failed', error=str(exc), task_history=gate.history)))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
