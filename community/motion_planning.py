"""Demo 04: original RRT-Connect on a UR3e, with dense edge validation."""
import numpy as np
import modeling.collision_model as cm
from community.common import parser, seed_rng, require, report, plan_path, show_robot
from community.robots import SingleArm


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)
    robot = SingleArm()
    start = np.array([0, -.7, .9, .3, .6, 0.])
    goal = start + np.array([.45, -.1, .2, .15, .2, -.15])
    obstacle = cm.gen_box(extent=[.12, .12, .3])
    obstacle.set_pos(np.array([.25, -.25, .15]))
    obstacle.set_rgba([.8, .25, .2, 1])
    robot.fk('arm', start)
    blocker = cm.gen_box(extent=[2, 2, 2])
    require(robot.is_collided([blocker]), 'Collision checker failed to detect a blocking box')
    path = plan_path(robot, 'arm', start, goal, [obstacle])
    report(args, '04-rrt-connect', validated_samples=len(path), edge_step_rad=.025,
           blocking_obstacle_detected=True)
    if not args.headless:
        show_robot(robot, path, lambda q: robot.fk('arm', q), [obstacle])


if __name__ == '__main__':
    main()
