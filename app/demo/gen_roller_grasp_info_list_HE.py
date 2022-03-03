import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
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

# load roller
rollerpath = os.path.join(basis.__path__[0], 'objects', 'roller.stl')
roller = cm.CollisionModel(rollerpath, cdprimit_type='polygons')
roller.set_rgba([1.0, 0.2, 0.7, 1.0])
roller.attach_to(base)

# Instantiate a gripper
gripper_s = rtqhe.RobotiqHE()
# generate grasp infos
grasp_info_list = gpa.plan_grasps(gripper_s, roller,
                                  angle_between_contact_normals=math.radians(160),
                                  openning_direction='loc_x',  # loc_x: opens and closes along the X axis of the object's coordinate system
                                  rotation_interval=math.radians(15),
                                  max_samples=30, min_dist_between_sampled_contact_points=.001,
                                  contact_offset=.001)
# write the grasp info list in pickle file
gpa.write_pickle_file('roller', grasp_info_list, './', 'roller.pickle')
# show the detail of the grasp info list
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

    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[150]
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)

    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[300]
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)

    jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[450]
    gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    gripper_s.gen_meshmodel().attach_to(base)
    #
    # jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[600]
    # gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
    # gripper_s.gen_meshmodel().attach_to(base)

base.run()