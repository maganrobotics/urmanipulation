"""Demo 02: a real UR3e model, FK target, IK and independently measured error."""
import numpy as np
from community.common import parser, seed_rng, require, report, dense_path, show_robot
from community.robots import SingleArm


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)
    robot = SingleArm()
    start = np.array([0, -.7, .9, .3, .6, 0.])
    target_q = start + np.array([.25, -.1, .15, 0, .1, -.1])
    robot.arm.fk(target_q)
    pos, rot = robot.arm.get_gl_tcp()
    robot.arm.fk(start)
    solution = robot.arm.ik(pos, rot, seed_jnt_values=start, local_minima='end')
    require(solution is not None, 'UR3e IK failed')
    robot.arm.fk(solution)
    actual_pos, actual_rot = robot.arm.get_gl_tcp()
    pos_error = float(np.linalg.norm(actual_pos - pos))
    rot_error = float(np.linalg.norm(actual_rot - rot))
    require(pos_error < 1e-3 and rot_error < .01, 'IK residual exceeds tolerance')
    require(not robot.is_collided(), 'IK solution is in self collision')
    report(args, '02-ur3e', position_error_m=pos_error, rotation_matrix_error=rot_error,
           joints_rad=solution.tolist(), collision_links=len(robot.arm.cc.all_cdelements))
    if not args.headless:
        show_robot(robot, dense_path([start, solution]), lambda q: robot.fk('arm', q))


if __name__ == '__main__':
    main()
