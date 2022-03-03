
import visualization.panda.world as wd
import modeling.geometric_model as gm
import modeling.collision_model as cm
import grasping.annotation.utils as gutil
import numpy as np
import basis.robot_math as rm
from robot_sim.robots.ur3e_dual.ur3e_dual import UR3EDual
from manipulation.approach_depart_planner import ADPlanner
from manipulation.pick_place_planner import PickPlacePlanner
import math

"""
Author: Ziqi Xu
date :  2022/01/17

"""

if __name__ == '__main__':
    base = wd.World(cam_pos=[2, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    # generate sphere and robot model
    robot_s = UR3EDual(enable_cc=True)
    u3ed_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
    u3ed_meshmodel.attach_to(base)
    # define the planner
    pp_planner = PickPlacePlanner(robot_s)
    manipulator_name = 'rgt_arm'
    hand_name = 'rgt_arm'
    # predefine the goal_homo_list and grasp_info_list
    goal_homomat_list = []
    grasp_info_list = []
    # set up conf_info, start_goal_info, end_goal_info
    # conf_info
    start_conf = robot_s.get_jnt_values(manipulator_name)
    # start_goal_info
    start_goal_pos = np.array([0.7, -0.2, 0.9])
    start_goal_rotmat = np.eye(3)
    start_goal_homomat = rm.homomat_from_posrot(start_goal_pos, start_goal_rotmat)
    goal_homomat_list.append(start_goal_homomat)
    # end_goal_info
    end_goal_pos = np.array([0.7, -0.2, 0.95])
    end_goal_rotmat = rm.rotmat_from_euler(0, -math.pi / 2, 0)
    end_goal_homomat = rm.homomat_from_posrot(end_goal_pos, end_goal_rotmat)
    goal_homomat_list.append(end_goal_homomat)
    # load the grasp_info_list
    grasp_info_list = gutil.load_pickle_file(objcm_name='sphere', file_name='sphere.pickle')
    grasp_info = grasp_info_list[0]
    # define the sphere pos
    objcm = cm.gen_sphere(radius=0.02)
    # objcm = cm.gen_sphere([0.7, 0.14, 0.8], 0.02)
    gm.gen_frame().attach_to(objcm)
    objcm.set_pos(start_goal_pos)
    objcm.set_rotmat(start_goal_rotmat)
    objcm.attach_to(base)
    objcm1 = objcm.copy()
    objcm1.set_pos(end_goal_pos)
    objcm1.set_rotmat(end_goal_rotmat)
    objcm1.attach_to(base)
    # objcm = cm.CollisionModel('tubebig.stl')

    # solve the conf_list, jawwidth_list, objpose_list
    conf_list, jawwidth_list, objpose_list = \
        pp_planner.gen_pick_and_place_motion(hnd_name=hand_name,
                                             objcm=objcm,
                                             grasp_info_list=grasp_info_list,
                                             #goal_homomat_list=goal_homomat_list,
                                             goal_homomat_list=[start_goal_homomat, end_goal_homomat],
                                             start_conf=start_conf,
                                             end_conf=start_conf,
                                             depart_direction_list=[np.array([0, 0, 1])] * len(goal_homomat_list),
                                             approach_direction_list=[np.array([0, 1, 0])] * len(goal_homomat_list),  # 接近和远离方向沿着哪一轴
                                             # depart_distance_list=[None] * len(goal_homomat_list),
                                             # approach_distance_list=[None] * len(goal_homomat_list),
                                             depart_distance_list=[.1] * len(goal_homomat_list),
                                             approach_distance_list=[.1] * len(goal_homomat_list), # 越小越好，本体不容易发生碰撞
                                             approach_jawwidth=None,
                                             depart_jawwidth=None,
                                             ad_granularity=.003,
                                             use_rrt=True,
                                             obstacle_list=[],
                                             use_incremental=False)
    print("the conf list are", conf_list)
    print("the jaw width list are", jawwidth_list)
    print("the obj pos list are ", objpose_list)

    robot_attached_list = []
    object_attached_list = []
    counter = [0]

    def update(robot_s,
               hand_name,
               objcm,
               robot_path,
               jawwidth_path,
               obj_path,
               robot_attached_list,
               object_attached_list,
               counter,
               task):
        if counter[0] >= len(robot_path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            object_attached_list.clear()
        pose = robot_path[counter[0]]
        robot_s.fk(hand_name, pose)
        robot_s.jaw_to(hand_name, jawwidth_path[counter[0]])
        robot_meshmodel = robot_s.gen_meshmodel()
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)
        obj_pose = obj_path[counter[0]]
        objb_copy = objcm.copy()
        objb_copy.set_homomat(obj_pose)
        objb_copy.attach_to(base)
        object_attached_list.append(objb_copy)
        counter[0] += 1
        return task.again


    taskMgr.doMethodLater(0.01, update, "update",
                          extraArgs=[robot_s,
                                     hand_name,
                                     objcm,
                                     conf_list,
                                     jawwidth_list,
                                     objpose_list,
                                     robot_attached_list,
                                     object_attached_list,
                                     counter],
                          appendTask=True)



    base.run()
