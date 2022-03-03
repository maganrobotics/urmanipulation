import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import os
import basis

"""
Author: Ziqi Xu
date :  2022/01/17

"""
# set up base environment
base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
gm.gen_frame().attach_to(base)

# load hole
holepath = os.path.join(basis.__path__[0], 'objects', 'hole.stl')
hole = cm.CollisionModel(holepath, cdprimit_type='polygons')
hole.set_rgba([1.0, 0.2, 0.7, 1.0])
hole.attach_to(base)

# Instantiate a gripper
gripper_s = rtqhe.RobotiqHE()
# generate grasp infos
grasp_info_list = gpa.plan_grasps(gripper_s, hole,
                                  angle_between_contact_normals=math.radians(160),
                                  openning_direction='loc_x',
                                  # loc_x: opens and closes along the X axis of the object's coordinate system
                                  rotation_interval=math.radians(15),
                                  max_samples=30, min_dist_between_sampled_contact_points=.01,
                                  contact_offset=.01)
# write the grasp info list in pickle file
gpa.write_pickle_file('hole', grasp_info_list, './', 'hole.pickle')
# whether to show the detail of the grasp info list
show_detail_of_grasp_infos = False
if show_detail_of_grasp_infos:
    print("the grasp info list\n", grasp_info_list)
    print("the range of grasp info list", len(grasp_info_list))

show_all_grasp_info_list = False
show_bits_of_grasp_info_list = True

if show_all_grasp_info_list:
    for grasp_info in grasp_info_list:
        jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
        gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
        gripper_s.gen_meshmodel().attach_to(base)

if show_bits_of_grasp_info_list:
    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[0]
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)

    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[120]
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)
    #
    # jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[160]
    # gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    # gripper_s.gen_meshmodel().attach_to(base)

    # jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[240]
    # gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    # gripper_s.gen_meshmodel().attach_to(base)
    #
    # jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[600]
    # gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    # gripper_s.gen_meshmodel().attach_to(base)

base.run()