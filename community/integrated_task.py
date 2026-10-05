"""Demo 07: mock navigation and sequential two-arm geometric pick-and-place."""
import numpy as np
import json
from pathlib import Path
import modeling.collision_model as cm
from community.common import parser, seed_rng, require, report, plan_path, show_robot
from community.grasping import candidates
from community.interface import BaseGoal, TaskGate, Phase, joint_names, validate_trajectory
from community.mobile_dual_arm import navigate
from community.robots import MobileDual


def main():
    cli = parser(__doc__)
    cli.add_argument('--plan-output', type=Path, help='Export the validated arm playback for the ROS2 mock bridge')
    args = cli.parse_args()
    seed_rng(args.seed)
    robot = MobileDual()
    gate = TaskGate()
    gate.start_navigation()
    route = navigate(robot, BaseGoal(.35, .1, .15))
    gate.navigation_result(True, True)
    gate.start_manipulation()
    snapshots, objects, counts, errors = [], [], [], []

    def capture():
        snapshots.append((np.r_[robot.get_jnt_values('lft_arm'), robot.get_jnt_values('rgt_arm')],
                          [(o.get_pos().copy(), o.get_rotmat().copy()) for o in objects]))

    # Each box starts at a known reachable grasp pose. Perception, approach
    # planning and contact dynamics are deliberately outside this v0.1 exercise.
    for side in ('lft', 'rgt'):
        component = side + '_arm'
        gripper = getattr(robot, side + '_hnd')
        q0 = robot.get_jnt_values(component).copy()
        pos, rot = robot.get_gl_tcp(component)
        obj = cm.gen_box(extent=[.016, .02, .025])
        obj.set_pos(pos)
        obj.set_rotmat(rot)
        obj.set_rgba([.9, .55, .1, 1] if side == 'lft' else [.15, .55, .9, 1])
        objects.append(obj)
        grasps = candidates(gripper.copy(), obj, pos, rot, width=.024)
        require(len(grasps) > 0, f'No grasp for {side} object')
        # The rotation sweep includes the current jaw pose at angle zero.
        require(any(np.allclose(g[2], rot) for g in grasps), 'Current jaw pose is not an accepted candidate')
        capture()
        robot.hold(component, obj, jaw_width=.024)
        delta = np.array([.08, -.06, .06, 0, .04, 0]) * (1 if side == 'lft' else -1)
        q1 = q0 + delta
        path = plan_path(robot, component, q0, q1, obstacles=())
        for q in path:
            robot.fk(component, q)
            info = getattr(robot, side + '_oih_infos')[0]
            obj.set_pos(info['gl_pos'])
            obj.set_rotmat(info['gl_rotmat'])
            capture()
        target, _ = robot.get_gl_tcp(component)
        error = float(np.linalg.norm(obj.get_pos() - target))
        require(error < 1e-5, 'Object did not follow the grasp transform')

        counts.append(len(path))
        errors.append(error)
        capture()
    # Keep the first object attached while moving the second arm. This makes
    # its geometry part of the robot collision scene rather than leaving the
    # other gripper touching a newly introduced static obstacle.
    for side, obj in zip(('lft', 'rgt'), objects):
        robot.release(side + '_arm', obj, jaw_width=.05)
        require(not getattr(robot, side + '_oih_infos'), 'Object release failed')
    require(not objects[0].is_mcdwith([objects[1]]), 'Placed objects intersect')
    capture()
    gate.manipulation_result(True)
    require(gate.phase == Phase.SUCCEEDED, 'Integrated task incomplete')
    if args.plan_output:
        names = joint_names('left') + joint_names('right')
        points = [dict(time_s=round(i * .2, 6), positions=s[0].tolist()) for i, s in enumerate(snapshots)]
        validate_trajectory(names, points)
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.plan_output.write_text(json.dumps(dict(schema='omindos.community.playback.v0.1',
            simulation_only=True, base_goal=dict(x=.35, y=.1, yaw=.15, frame_id='map'),
            joint_names=names, points=points), indent=2) + '\n', encoding='utf-8')
    report(args, '07-integrated-task', navigation='mock', base_samples=len(route),
           arm_path_samples=counts, placement_error_m=errors, objects_released=2,
           task_history=gate.history,
           scope='Sequential two-arm geometric transfer; no perception, contact dynamics or handover')
    if not args.headless:
        def apply(state):
            robot.fk('both_arm', state[0])
        def objects_at(index):
            models = []
            for obj, (pos, rot) in zip(objects, snapshots[index][1]):
                model = obj.copy()
                model.set_pos(pos)
                model.set_rotmat(rot)
                models.append(model)
            return models
        show_robot(robot, snapshots, apply, lookat=(.7, 0, 1), objects_at=objects_at)


if __name__ == '__main__':
    main()
