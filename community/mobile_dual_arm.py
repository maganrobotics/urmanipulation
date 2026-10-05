"""Demo 06: mock navigation result -> base transform -> dual-arm readiness."""
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
from community.common import parser, seed_rng, require, report, show_robot
from community.interface import BaseGoal, TaskGate, Phase
from community.robots import MobileDual


def navigate(robot, goal, obstacles=()):
    require(not robot.lft_oih_infos and not robot.rgt_oih_infos,
            'v0.1 base relocation requires empty grippers')
    # Kinematic playback of a supplied mock route; not a navigation planner.
    states = [(np.array([goal.x * s, goal.y * s, 0]),
               rm.rotmat_from_axangle([0, 0, 1], goal.yaw * s)) for s in np.linspace(0, 1, 31)]
    before_pos, before_rot = robot.get_gl_tcp('lft_arm')
    for pos, rot in states:
        robot.set_base(pos, rot)
        require(not robot.is_collided(obstacle_list=list(obstacles)), 'Mobile route collision')
    actual_pos, actual_rot = robot.get_gl_tcp('lft_arm')
    pos, rot = states[-1]
    require(np.allclose(actual_pos, pos + rot @ before_pos), 'TCP base transform mismatch')
    require(np.allclose(actual_rot, rot @ before_rot), 'TCP orientation transform mismatch')
    return states


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)
    robot = MobileDual()
    gate = TaskGate()
    gate.start_navigation()
    obstacle = cm.gen_box(extent=[.2, .2, 1.8])
    obstacle.set_pos(np.array([0, -1.3, .9]))
    states = navigate(robot, BaseGoal(.45, .15, .2), [obstacle])
    gate.navigation_result(success=True, stopped=True)
    require(gate.phase == Phase.READY, 'Mobile task did not become ready')
    report(args, '06-mobile-dual-arm', validated_base_samples=len(states),
           navigation='mock', transport='in-process', task_history=gate.history,
           scope='Base transform and collision checks; no Nav2 or base dynamics in this mode')
    if not args.headless:
        show_robot(robot, states, lambda s: robot.set_base(*s), [obstacle], lookat=(.6, 0, 1))


if __name__ == '__main__':
    main()
