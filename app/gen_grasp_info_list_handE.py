import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.yumi_gripper.yumi_gripper as yg
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe

"""
Author: Ziqi Xu
date :  2022/01/17

"""

base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
gm.gen_frame().attach_to(base)
# object
# object_tube = cm.CollisionModel("objects/tubebig.stl")

# --------- this is written by Ziqi----------------
# object_tube_gm = gm.gen_sphere([0.7, 0.14, 0.8], 0.02)
object_tube = cm.gen_sphere(radius=0.02)
print("the pos is ", object_tube.get_pos() )
object_tube_gm = gm.gen_sphere(radius=0.02)
object_tube.set_rgba([.9, .75, .35, .3])
object_tube.attach_to(base)
object_tube_gm.attach_to(base)
# hnd_s
# gripper_s = yg.YumiGripper()
gripper_s = rtqhe.RobotiqHE()
grasp_info_list = gpa.plan_grasps(gripper_s, object_tube,
                                  angle_between_contact_normals=math.radians(160),
                                  openning_direction='loc_x',  # loc_x
                                  rotation_interval=math.radians(15),
                                  max_samples=15, min_dist_between_sampled_contact_points=.001,
                                  contact_offset=.001)
print("the grasp info list\n", grasp_info_list)
print("the range of grasp info list", len(grasp_info_list))
gpa.write_pickle_file('sphere', grasp_info_list, './', 'sphere.pickle')

# show all the grasp infos
# for grasp_info in grasp_info_list:
#     jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
#     gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
#     gripper_s.gen_meshmodel().attach_to(base)
# show some grasp infos
jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[0]
gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
gripper_s.gen_meshmodel().attach_to(base)

jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[150]
gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
gripper_s.gen_meshmodel().attach_to(base)

jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[300]
gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
gripper_s.gen_meshmodel().attach_to(base)
#
jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[450]
gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
gripper_s.gen_meshmodel().attach_to(base)
#
jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info_list[600]
gripper_s.grip_at_with_jcpose(jaw_center_pos, jaw_center_rotmat, jaw_width)
gripper_s.gen_meshmodel().attach_to(base)
base.run()