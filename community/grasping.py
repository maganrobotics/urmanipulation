"""Demo 03: generate collision-filtered grasp candidates on a procedural box."""
import numpy as np
import modeling.collision_model as cm
from grasping.annotation.utils import define_grasp_with_rotation
from robot_sim.end_effectors.grippers.robotiq85.robotiq85 import Robotiq85
from community.common import parser, seed_rng, require, report, show_robot


def candidates(gripper, obj, pos=None, rot=None, width=.065):
    pos = np.zeros(3) if pos is None else pos
    rot = np.eye(3) if rot is None else rot
    return define_grasp_with_rotation(gripper, obj, pos, rot[:, 2], rot[:, 1], width,
                                      rot[:, 1], rotation_interval=np.pi / 3,
                                      toggle_flip=False, toggle_debug=False)


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)
    obj = cm.gen_box(extent=[.02, .06, .04])
    obj.set_rgba([.8, .5, .15, 1])
    gripper = Robotiq85()
    grasps = candidates(gripper, obj)
    require(len(grasps) > 0, 'No collision-free grasp candidates')

    def apply(grasp):
        width, _, _, pos, rot = grasp
        gripper.fix_to(pos, rot)
        gripper.jaw_to(width)

    for grasp in grasps:
        apply(grasp)
        require(not gripper.is_mesh_collided([obj]), 'Returned grasp intersects the box')
    report(args, '03-grasping', candidates=len(grasps), jaw_width_m=.065,
           scope='Geometric candidates; contact forces and force closure are not simulated')
    if not args.headless:
        show_robot(gripper, [g for g in grasps for _ in range(25)], apply, [obj], lookat=(0, 0, 0))


if __name__ == '__main__':
    main()
