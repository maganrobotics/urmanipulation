import math
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtqhe
import os
import basis
import numpy as np
import grasping.annotation.utils as gu
import basis.robot_math as rm

"""
Author: Ziqi Xu
date :  2022/01/17

"""

# set up base environment
base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
gm.gen_frame().attach_to(base)

# load roller
holepath = os.path.join(basis.__path__[0], 'objects', 'hole.stl')
hole = cm.CollisionModel(holepath, cdprimit_type='polygons')
hole.set_rgba([0.7, 0.7, 0.7, 1.0])
hole.attach_to(base)

# Instantiate a gripper
gripper_s = rtqhe.RobotiqHE()

gl_jaw_center_z = rm.rotmat_from_axangle(np.array([0,1,0]), math.pi/2).dot(np.array([-1,0,0]))
print("the gl_jaw_center_z", gl_jaw_center_z)

grasp_info_list = gu.define_grasp(gripper_s, hole,
                                  gl_jaw_center_pos=np.array([0,0,-0.03]),
                                  gl_jaw_center_z=gl_jaw_center_z,
                                  gl_jaw_center_y=np.array([0,1,0]),
                                  jaw_width=.012,
                                  toggle_flip=True,
                                  toggle_debug=True)
print("the grasp info list are", grasp_info_list)
gu.write_pickle_file('hole', grasp_info_list, './', 'define_hole.pickle')
# grasp_info_list = gu.define_grasp_with_rotation(gripper_s, roller,
#                                                 gl_jaw_center_pos=np.array([0, 0, 0]),
#                                                 gl_jaw_center_z=np.array([-1, 0, 0]),
#                                                 gl_jaw_center_y=np.array([0, 1, 0]),
#                                                 jaw_width=.05,
#                                                 gl_rotation_ax=np.array([0, 1, 0]),
#                                                 rotation_range=(math.radians(-180), math.radians(180)),
#                                                 rotation_interval=math.radians(30),
#                                                 toggle_flip=False,
#                                                 toggle_debug=True)
for grasp_info in grasp_info_list:
    aw_width, gl_jaw_center_pos, gl_jaw_center_rotmat, hnd_pos, hnd_rotmat = grasp_info
    gripper_s.fix_to(hnd_pos, hnd_rotmat)
    gripper_s.jaw_to(aw_width)
    gripper_s.gen_meshmodel().attach_to(base)
# gm.gen_frame(pos=hnd_pos, rotmat=hnd_rotmat).attach_to(base)
base.run()
