"""Demo 05: fixed-base dual-arm coordinated joint motion with collision checks."""
import numpy as np
from robot_sim.robots.ur3e_dual.ur3e_dual import UR3EDual
from community.common import parser, seed_rng, require, report, dense_path, show_robot


def coordinated_states(robot):
    start = np.r_[robot.get_jnt_values('lft_arm'), robot.get_jnt_values('rgt_arm')]
    goal = start + np.array([.08, -.08, .08, 0, .08, 0, -.08, .08, -.08, 0, -.08, 0])
    states = dense_path([start, goal])
    limits = np.vstack([robot.get_jnt_ranges('lft_arm'), robot.get_jnt_ranges('rgt_arm')])
    for q in states:
        require(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), 'Dual-arm joint limit violation')
        robot.fk('both_arm', q)
        require(not robot.is_collided(), 'Dual-arm trajectory collision')
    return states


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)
    robot = UR3EDual()
    states = coordinated_states(robot)
    report(args, '05-fixed-dual-arm', validated_samples=len(states), joints=12,
           scope='Synchronized kinematics; no closed-chain object constraint')
    if not args.headless:
        show_robot(robot, states, lambda q: robot.fk('both_arm', q), lookat=(.4, 0, 1))


if __name__ == '__main__':
    main()
