import time, sys
sys.path.append("./")
print(sys.path)
import visualization.panda.world as wd
import modeling.geometric_model as gm
import basis
import modeling.collision_model as cm
import grasping.annotation.utils as gutil
import numpy as np
import robot_sim.robots.ur3e_dual.ur3e_dual as ur3e_dual
import os
from manipulation.pick_place_planner import PickPlacePlanner
import manipulation.approach_depart_planner as adp
import motion.probabilistic.rrt_connect as rrtc
import motion.optimization_based.incremental_nik as inik
import math
import basis.robot_math as rm
import motion.trajectory.piecewisepoly as pwp
import pandas as pd

from panda3d.bullet import BulletRigidBodyNode
import modeling.dynamics.bullet.bdmodel as bdm
import modeling.dynamics.bullet.bdbody as bbd



"""
Author: Ziqi Xu
date :  2022/01/17

"""


def get_lnk_bdmodel(robot_s, component_name, lnk_id):
    print(
        f"robot_s.manipulator_dict[component_name].lnks:{len(robot_s.manipulator_dict[component_name].lnks)} | {robot_s.manipulator_dict[component_name].lnks}")
    lnk = robot_s.manipulator_dict[component_name].lnks[lnk_id]
    bd_lnk = bdm.BDModel(lnk["collisionmodel"], mass=0, type="box", dynamic=False)
    bd_lnk.set_homomat(rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
    return bd_lnk


def update_robot_bdmodel(robot_s, bd_lnk_list):
    cnter = 0
    for arm_name in ["lft_arm", "rgt_arm"]:
        for lnk_id in [1, 2, 3, 4, 5, 6]:
            lnk = robot_s.manipulator_dict[arm_name].lnks[lnk_id]
            bd_lnk_list[cnter].set_homomat(rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
            cnter += 1


def get_robot_bdmoel(robot_s):
    bd_lnk_list = []
    for arm_name in ["lft_arm", "rgt_arm"]:
        for lnk_id in [0, 1, 2, 3, 4, 5, 6]:
            bd_lnk_list.append(get_lnk_bdmodel(robot_s, arm_name, lnk_id))
    return bd_lnk_list


if __name__ == '__main__':

    # load model
    # load base environment
    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    # load dual_arm robot
    u3ed = ur3e_dual.UR3EDual()

    # define pick and place planner
    ppplanner = PickPlacePlanner(u3ed)
    # define approach and depart  planner
    adplanner = adp.ADPlanner(u3ed)
    # define rrt_planner
    rrtPlanner = rrtc.RRTConnect(u3ed)
    # define incremental_ik solver
    inik_svlr = inik.IncrementalNIK(u3ed)
    # define the trajectory builder
    traj_gen = pwp.PiecewisePoly(method="linear")

    # pre_define manipulator name and hand name
    rgt_manipluator_name = 'rgt_arm'
    rgt_hand_name = 'rgt_arm'
    lft_manipulator_name = 'lft_arm'
    lft_hand_name = 'lft_arm'

    # define rgt arm home pose
    rgt_arm_home_pose = u3ed.get_gl_tcp(manipulator_name='rgt_arm')
    rgt_arm_home_pos = rgt_arm_home_pose[0]
    rgt_arm_home_rotmat = rgt_arm_home_pose[1]
    rgt_arm_home_conf = u3ed.get_jnt_values(rgt_manipluator_name)
    print("the rgt_arm home pose", rgt_arm_home_pose)
    # define lft arm home pose
    lft_arm_home_pose = u3ed.get_gl_tcp(manipulator_name='lft_arm')
    lft_arm_home_pos = lft_arm_home_pose[0]
    lft_arm_home_rotmat = lft_arm_home_pose[1]
    lft_arm_home_conf = u3ed.get_jnt_values(lft_manipulator_name)
    print("the lft_arm home pose", lft_arm_home_pose)
    print("the lft_arm_home_conf", lft_arm_home_conf)

    # set rotation
    rotmat = rm.rotmat_from_euler(math.radians(180), 0, 0)

    # define the move list

    cube_conf_list = []
    cube_jawwidth_list = []
    cube_pose_list = []
    cube_homomat_list = []

    right_conf_list = []
    left_conf_list = []

    left_start_conf_list = []
    left_start_jawwidth_list = []
    left_start_pose_list = []
    left_homomat_list = []



    lrotmat = rm.rotmat_from_euler(math.radians(90), 0, 0)
    rrotmat = rm.rotmat_from_euler(math.radians(-90), 0, 0)
    trotmat = rm.rotmat_from_euler(math.radians(-180), 0, 0)
    drotmat = rm.rotmat_from_euler(math.radians(180), 0, 0)


    left_lmove_pos = np.array([0.6, .028, 1.0])
    left_lmove_rotmat = lrotmat
    left_tmove_pos = np.array([0.6, 0.0022, 1.024])
    left_tmove_rotmat = drotmat
    left_thold_pos = np.array([0.6, 0.0022, 1.020])
    left_thold_rotmat = drotmat
    left_hold_pos = np.array([0.6, .024, 0.993])
    left_hold_rotmat = lrotmat
    left_wait_pos = np.array([0.6, .051, 1.0])
    left_wait_rotmat = lrotmat
    left_twait_pos = np.array([0.6, 0.0022, 1.046])
    left_twait_rotmat = drotmat



    right_rmove_pos = np.array([0.6, -0.022, 1.0])
    right_rmove_rotmat = rrotmat
    right_tmove_pos = np.array([0.6, 0.0022, 1.024])
    right_tmove_rotmat = trotmat
    right_thold_pos = np.array([0.6, 0.0022, 1.020])
    right_thold_rotmat = trotmat
    right_hold_pos = np.array([0.6, -0.018, 0.993])
    right_hold_rotmat = rrotmat
    right_wait_pos = np.array([0.6, -0.044, 1.0])
    right_wait_rotmat = rrotmat
    right_twait_pos = np.array([0.6, 0.0022, 1.046])
    right_twait_rotmat = trotmat


    left_grasp_info_list = gutil.load_pickle_file(objcm_name='hole', file_name='define_hole.pickle')
    left_grasp_info = left_grasp_info_list[0]
    left_jaw_width, left_jaw_center_pos, left_jaw_center_rotmat, left_hnd_pos, left_hnd_rotmat = left_grasp_info

    cube_grasp_info_list = gutil.load_pickle_file(objcm_name='roller', file_name='define_roller.pickle')
    cube_grasp_info = cube_grasp_info_list[0]
    cube_grasp_info[0] = 0.041
    cube_grasp_info[1][-1] = -0.018
    cube_grasp_info[1][1] = 0.008
    cube_jaw_width, cube_jaw_center_pos, cube_jaw_center_rotmat, cube_hnd_pos, cube_hnd_rotmat = cube_grasp_info

    # define the pick pos and rotmat
    cube_pick_pos = np.array([0.7, -0.304, 0.83])
    cube_pick_rotmat = rotmat


    center_pos = np.array([0.6, 0.0035, 1.0])
    center_rotmat = rrotmat

    cube_location_pos = np.array([0.7, -0.3, 0.81])
    cube_location_rotmat = trotmat



    direction_list = [[0, 0, -1], [0, 0, 1], [-1, 0, 0], [0, -1, 0], [1, 0, 0], [0, 1, 0]]


    def rotational(radian, last_conf, last_jawwidth, last_pose, rotational_type=True, movement_range=20):
        if rotational_type:
            flag = 1
        else:
            flag = -1
        print(f"last_conf: {last_conf}")
        # rotate30 = last_conf.copy()
        # rotate30[-1] += flag*math.pi / 6
        # rotate60 = last_conf.copy()
        # rotate60[-1] += flag*math.pi / 3
        # rotate90 = last_conf.copy()
        # rotate90[-1] += flag*math.pi / 2

        rotate_list = []
        for i in range(1, movement_range + 1):
            add_radian = i * radian / movement_range
            temp_list = last_conf.copy()
            temp_list[-1] += add_radian
            rotate_list.append(temp_list)

        return rotate_list, [last_jawwidth for _ in range(movement_range)], [last_pose for _ in range(movement_range)]


    def sleep(times=3):
        times = int(times * 10)
        global left_conf_list, left_jawwidth_list, left_pose_list, right_conf_list, right_jawwidth_list, right_pose_list
        l1 = left_conf_list[-1]
        l2 = left_jawwidth_list[-1]
        l3 = left_pose_list[-1]
        r1 = right_conf_list[-1]
        r2 = right_jawwidth_list[-1]
        r3 = right_pose_list[-1]
        t1, t2, t3, t4, t5, t6 = [l1 for _ in range(times)], [l2 for _ in range(times)], [l3 for _ in range(times)], \
                                 [r1 for _ in range(times)], [r2 for _ in range(times)], [r3 for _ in range(times)],
        left_conf_list += t1
        left_jawwidth_list += t2
        left_pose_list += t3
        right_conf_list += t4
        right_jawwidth_list += t5
        right_pose_list += t6
        return t1, t2, t3, t4, t5, t6


    def dual_arm_init_generator(component_name, wait_conf_list, last_motion="", obj_pos=cube_location_pos,
                                obj_rotmat=cube_location_rotmat):
        if str(last_motion) == "":
            last_motion = u3ed.get_jnt_values(component_name)
        arm_static_conf_list, \
        arm_static_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=wait_conf_list,
                                                               conf=last_motion,
                                                               jawwidth=0.041)

        arm_static_pose_list = ppplanner.gen_object_motion(component_name=component_name,
                                                           conf_list=arm_static_conf_list,
                                                           obj_pos=obj_pos,
                                                           obj_rotmat=obj_rotmat,
                                                           type='absolute')
        return arm_static_conf_list, arm_static_jawwidth_list, arm_static_pose_list


    def dual_arm_static_generator(component_name, wait_conf):
        global left_conf_list, left_jawwidth_list, left_pose_list, right_conf_list, right_jawwidth_list, right_pose_list
        conf_range = len(wait_conf)
        if component_name == "lft_arm":
            conf_list = left_conf_list
            jawwidth_list = left_jawwidth_list
            pose_list = left_pose_list
        elif component_name == "rgt_arm":
            conf_list = right_conf_list
            jawwidth_list = right_jawwidth_list
            pose_list = right_pose_list
        else:
            print("机械臂输入名称错误")
            return None
        static_conf_list = [conf_list[-1] for _ in range(conf_range)]
        static_jawwidth_list = [jawwidth_list[-1] for _ in range(conf_range)]
        static_pose_list = [pose_list[-1] for _ in range(conf_range)]
        return static_conf_list, static_jawwidth_list, static_pose_list

    def none_pose_list_generator(none_conf_list):
        none_pose_list = [np.array([[0, 0, 0, 0] for _ in range(4)]) for _ in
                                 range(len(none_conf_list))]
        return none_pose_list


    for i in sys.path:
        rubik_model_path = i + "rubik's_cube/rubiks.bam"
        if os.path.exists(rubik_model_path):
            break
    cube_model = base.loader.loadModel(rubik_model_path)
    cube_model.setScale(0.00645)
    min, max = cube_model.getTightBounds()
    size = max - min
    print(f"min: {min}, max: {max}, size: {size}")
    from panda3d.core import Point3

    dimensions = Point3(max - min)
    compList = [dimensions.getX(), dimensions.getY(), dimensions.getZ()]
    print(compList)

    cube = cm.CollisionModel(cube_model, cdprimit_type='box')

    cube.set_pos(cube_location_pos)
    # cube.set_scale([1, 1, 1])
    cube.set_rotmat(cube_location_rotmat)
    # base.attach_internal_update_obj(cube)
    # cube.attach_to(base)
    # cube.show_localframe()








    s = cube_pick_pos[2]  # 正确的cube_pick_pos =
    temp_direction_list = []
    for _ in range(1):
        flag = False
        cube_pick_pos[2] = s
        for i in direction_list:
            # 靠近魔方的动作序列
            cube_approach_conf_list, cube_approach_jawwidth_list = adplanner.gen_approach_motion(rgt_manipluator_name,
                                                                                                 cube_pick_pos,  #cube_pick_rotmat.dot(cube_jaw_center_pos) +
                                                                                                 cube_pick_rotmat.dot(
                                                                                                     cube_jaw_center_rotmat),
                                                                                                 start_conf=u3ed.get_jnt_values(
                                                                                                     rgt_manipluator_name),
                                                                                                 approach_direction=np.array(
                                                                                                     [0, 0, -1]),
                                                                                                 approach_distance=0.05,
                                                                                                 toggle_end_grasp=True,
                                                                                                 end_jawwidth=0.041,
                                                                                                 granularity=0.001
                                                                                                 )
            if cube_approach_conf_list:
                print(f"正确的cube_pick_pos = {cube_pick_pos[2]}, direction = {i}")
                flag = True
                if 0:
                    temp_direction_list.append(i)
                else:
                    break
        if flag:
            print(f"temp_direction_list: {temp_direction_list}")
            break
        s += 0.001


    # 爪子抓取魔方的运动序列
    cube_approach_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                          conf_list=cube_approach_jawwidth_list,
                                                          obj_pos=cube_location_pos,
                                                          obj_rotmat=cube_location_rotmat,
                                                          type='absolute')

    # left_start_conf_list, left_start_jawwidth_list, left_start_pose_list = dual_arm_init_generator(component_name=lft_hand_name, wait_conf_list=cube_approach_conf_list, last_motion="")



    left_conf_list = left_start_conf_list
    left_jawwidth_list = left_start_jawwidth_list
    left_pose_list = left_start_pose_list

    right_conf_list = cube_approach_conf_list
    right_jawwidth_list = cube_approach_jawwidth_list
    right_pose_list = cube_approach_pose_list





    cube_homomat_list.append(rm.homomat_from_posrot(cube_pick_pos, cube_location_rotmat))
    cube_homomat_list.append(rm.homomat_from_posrot(center_pos, center_rotmat))

    # 抓取魔方并移动到多个点, 返回 魔方运动序列, 爪子运动序列, 魔方姿态
    cube_holding_moveto_conf_list, \
    cube_holding_moveto_jawwidth_list, \
    cube_holding_moveto_pose_list = ppplanner.gen_holding_moveto(hand_name=rgt_hand_name,
                                                                 objcm=cube,
                                                                 grasp_info=cube_grasp_info,
                                                                 obj_pose_list=cube_homomat_list,
                                                                 depart_direction_list=np.array([0, 0, 1]) * len(
                                                                     cube_homomat_list),
                                                                 approach_direction_list=np.array([0, 0, -1]) * len(
                                                                     cube_homomat_list),
                                                                 depart_distance_list=[0.05] * len(cube_homomat_list),
                                                                 approach_distance_list=[0.05] * len(cube_homomat_list),
                                                                 ad_granularity=0.001,
                                                                 use_rrt=True,
                                                                 # seed_jnt_values=u3ed.get_jnt_ranges(rgt_manipluator_name)
                                                                 seed_jnt_values=cube_approach_conf_list[-1])

    print(f"cube_holding_moveto_conf_list,{cube_holding_moveto_conf_list} \n"
          f"cube_holding_moveto_jawwidth_list,{cube_holding_moveto_jawwidth_list} \n"
          f"cube_holding_moveto_pose_list,{cube_holding_moveto_pose_list}")



    # 拼接魔方的运动序列
    cube_conf_list += cube_approach_conf_list
    cube_conf_list += cube_holding_moveto_conf_list

    # 拼接爪子抓魔方的运动序列
    cube_jawwidth_list += cube_approach_jawwidth_list
    cube_jawwidth_list += cube_holding_moveto_jawwidth_list

    # 拼接魔方姿态
    cube_pose_list += cube_approach_pose_list
    cube_pose_list += cube_holding_moveto_pose_list

    # 左臂等待右臂的运动序列, 返回左臂姿态
    left_static_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                   conf_list=cube_conf_list,
                                                   obj_pos=cube_pick_pos,
                                                   obj_rotmat=cube_pick_rotmat,
                                                   type='absolute')
    # 右臂运动(抓取+移动)时, 左臂等待(保持不动, 关节值, 爪子宽度), 返回左臂运动序列, 爪子运动序列
    left_static_conf_list, \
    left_static_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=cube_conf_list,
                                                            conf=u3ed.get_jnt_values(lft_manipulator_name),
                                                            jawwidth=0.041)


    left_conf_list += left_static_conf_list
    left_jawwidth_list += left_static_jawwidth_list
    left_pose_list += left_static_list

    right_conf_list += cube_holding_moveto_conf_list
    right_jawwidth_list += cube_holding_moveto_jawwidth_list
    right_pose_list += cube_holding_moveto_pose_list








    s = left_wait_pos[1]  # 正确的left_wait_pos =
    temp_direction_list = []
    for _ in range(10):
        flag = False
        left_wait_pos[1] = s
        for i in direction_list:
            left_st2wait_conf_list, left_st2wait_jawwidth_list = adplanner.gen_approach_motion(
                lft_manipulator_name,
                # left_wait_rotmat.dot(left_jaw_center_pos) + left_wait_pos,
                 left_wait_pos, # left_wait_rotmat.dot(left_jaw_center_pos) +
                left_wait_rotmat,  # .dot(left_jaw_center_rotmat)
                start_conf=u3ed.get_jnt_values(
                    lft_manipulator_name),
                approach_direction=np.array(
                    [0, 0, 1]),
                approach_distance=0.05,
                toggle_end_grasp=True,
                end_jawwidth=0.041,
                granularity=0.001)
            if left_st2wait_conf_list:
                print(f"正确的left_wait_pos = {left_wait_pos}, direction = {i}")
                flag = True
                if 0:
                    temp_direction_list.append(i)
                else:
                    break
        if flag:
            print(f"temp_direction_list: {temp_direction_list}")
            break
        s += 0.001

    left_st2wait_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_st2wait_jawwidth_list,
                                                                obj_pos=left_wait_pos,
                                                                obj_rotmat=left_wait_rotmat,
                                                                type='absolute')
    left_st2wait_none_pose_list = none_pose_list_generator(left_st2wait_conf_list)

    r_st2wait_conf_list, \
    r_st2wait_jawwidth_list, \
    r_st2wait_pose_list = dual_arm_init_generator(component_name=rgt_hand_name,
                                                         wait_conf_list=left_st2wait_conf_list,
                                                         last_motion=right_conf_list[-1],
                                                         obj_pos=center_pos, obj_rotmat=center_rotmat)


    left_wait2st_conf_list = left_st2wait_conf_list[::-1]
    left_wait2st_jawwidth_list = left_st2wait_jawwidth_list[::-1]
    left_wait2st_pose_list = left_st2wait_pose_list[::-1]
    left_wait2st_none_pose_list = left_st2wait_none_pose_list[::-1]


    r_wait2st_conf_list = r_st2wait_conf_list[::-1]
    r_wait2st_jawwidth_list = r_st2wait_jawwidth_list[::-1]
    r_wait2st_pose_list = r_st2wait_pose_list[::-1]

    right_conf_list += r_st2wait_conf_list
    right_jawwidth_list += r_st2wait_jawwidth_list
    right_pose_list += r_st2wait_pose_list

    cube_center_pose = r_st2wait_pose_list[-1]

    left_conf_list += left_st2wait_conf_list
    left_jawwidth_list += left_st2wait_jawwidth_list
    left_pose_list += left_st2wait_none_pose_list
    left_pose_list += [cube_center_pose for _ in range(len(left_st2wait_conf_list))]


    # sleep(10)

    enable_left = False
    enable_right = True




    """
    ================================ 左臂拆解开始 ================================
    """
    if enable_left:

        s = left_lmove_pos[1]  # 正确的left_lmove_pos =
        temp_direction_list = []
        for _ in range(1):
            flag = False
            left_lmove_pos[1] = s
            for i in direction_list:
                left_wait2move_conf_list, left_wait2move_jawwidth_list = adplanner.gen_approach_motion(
                    lft_manipulator_name,
                    left_lmove_pos,  # left_lmove_rotmat.dot(cube_jaw_center_pos) +
                    left_lmove_rotmat,  # .dot(left_jaw_center_rotmat)
                    start_conf=left_conf_list[-1],
                    approach_direction=np.array(
                        [0, -1, 0]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001
                )
                print(left_conf_list[-1])
                print(left_wait2move_conf_list)
                print(f"是否相等{left_conf_list[-1] == left_wait2move_conf_list[0]}")
                left_wait2move_conf_list, left_wait2move_jawwidth_list = left_wait2move_conf_list[30:], left_wait2move_jawwidth_list[30:]

                if left_wait2move_conf_list:
                    print(f"正确的left_lmove_pos = {left_lmove_pos[1]}, direction = {i}")
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001



        left_wait2move_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_wait2move_jawwidth_list,
                                                                obj_pos=left_lmove_pos,
                                                                obj_rotmat=left_lmove_rotmat,
                                                                type='absolute')





        left_wait2move_none_pose_list = none_pose_list_generator(left_wait2move_conf_list)



        r_wait2move_conf_list, r_wait2move_jawwidth_list, r_wait2move_pose_list = dual_arm_static_generator(rgt_hand_name, left_wait2move_conf_list)


        left_conf_list += left_wait2move_conf_list
        left_jawwidth_list += left_wait2move_jawwidth_list
        left_pose_list += left_wait2move_none_pose_list

        right_conf_list += r_wait2move_conf_list
        right_jawwidth_list += r_wait2move_jawwidth_list
        right_pose_list += r_wait2move_pose_list


        left_move2wait_conf_list = left_wait2move_conf_list[::-1]
        left_move2wait_jawwidth_list = left_wait2move_jawwidth_list[::-1]
        left_move2wait_pose_list = left_wait2move_pose_list[::-1]
        left_move2wait_none_pose_list = left_wait2move_none_pose_list[::-1]

        r_move2wait_conf_list = r_wait2move_conf_list[::-1]
        r_move2wait_jawwidth_list = r_wait2move_jawwidth_list[::-1]
        r_move2wait_pose_list = r_wait2move_pose_list[::-1]

        sleep(10)


        left_conf_list += left_move2wait_conf_list
        left_jawwidth_list += left_move2wait_jawwidth_list
        left_pose_list += left_move2wait_none_pose_list

        right_conf_list += r_move2wait_conf_list
        right_jawwidth_list += r_move2wait_jawwidth_list
        right_pose_list += r_move2wait_pose_list



        sleep(10)




        s = left_hold_pos[1]  # 正确的left_lmove_pos =
        temp_direction_list = []
        for _ in range(10):
            flag = False
            left_hold_pos[1] = s
            for i in direction_list:
                left_wait2hold_conf_list, left_wait2hold_jawwidth_list = adplanner.gen_approach_motion(
                    lft_manipulator_name,
                    left_hold_pos,  # left_hold_rotmat.dot(cube_jaw_center_pos) +
                    left_hold_rotmat.dot(cube_jaw_center_rotmat),
                    start_conf=left_conf_list[-1],
                    approach_direction=np.array(
                        [0,-1,0]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.0008
                )
                left_wait2hold_conf_list, left_wait2hold_jawwidth_list = left_wait2hold_conf_list[30:], left_wait2hold_jawwidth_list[30:]
                if left_wait2hold_conf_list:
                    print(f"正确的left_hold_pos = {left_hold_pos}, direction = {i}")
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.001



        left_wait2hold_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_wait2hold_jawwidth_list,
                                                                obj_pos=left_hold_pos,
                                                                obj_rotmat=left_hold_rotmat,
                                                                type='absolute')



        left_wait2hold_none_pose_list = none_pose_list_generator(left_wait2hold_conf_list)

        r_wait2hold_conf_list, r_wait2hold_jawwidth_list, r_wait2hold_pose_list = dual_arm_static_generator(rgt_hand_name,
                                                                                                   left_wait2hold_conf_list)

        left_conf_list += left_wait2hold_conf_list
        left_jawwidth_list += left_wait2hold_jawwidth_list
        left_pose_list += left_wait2hold_none_pose_list

        right_conf_list += r_wait2hold_conf_list
        right_jawwidth_list += r_wait2hold_jawwidth_list
        right_pose_list += r_wait2hold_pose_list



        sleep(5)


        left_hold2wait_conf_list = left_wait2hold_conf_list[::-1]
        left_hold2wait_jawwidth_list = left_wait2hold_jawwidth_list[::-1]
        left_hold2wait_pose_list = left_wait2hold_pose_list[::-1]
        left_hold2wait_none_pose_list = left_wait2hold_none_pose_list[::-1]

        r_hold2wait_conf_list = r_wait2hold_conf_list[::-1]
        r_hold2wait_jawwidth_list = r_wait2hold_jawwidth_list[::-1]
        r_hold2wait_pose_list = r_wait2hold_pose_list[::-1]

        left_conf_list += left_hold2wait_conf_list
        left_jawwidth_list += left_hold2wait_jawwidth_list
        left_pose_list += left_hold2wait_none_pose_list

        right_conf_list += r_hold2wait_conf_list
        right_jawwidth_list += r_hold2wait_jawwidth_list
        right_pose_list += r_hold2wait_pose_list


        sleep(5)

        left_conf_list += left_wait2st_conf_list
        left_jawwidth_list += left_wait2st_jawwidth_list
        left_pose_list += left_wait2st_none_pose_list

        right_conf_list += r_wait2st_conf_list
        right_jawwidth_list += r_wait2st_jawwidth_list
        right_pose_list += r_wait2st_pose_list



        s = left_twait_pos[1]

        for _ in range(1):
            flag = False
            left_twait_pos[1] = s
            for i in direction_list:
                left_st2twait_conf_list, left_st2twait_jawwidth_list = adplanner.gen_approach_motion(lft_manipulator_name,
                                                                                                         # left_wait_rotmat.dot(left_jaw_center_pos) + left_wait_pos,
                                                                                                         left_twait_pos,  # left_twait_rotmat.dot(left_jaw_center_pos) +
                                                                                                         left_twait_rotmat,  # .dot(left_jaw_center_rotmat)
                                                                                                         start_conf=left_conf_list[-1], #u3ed.get_jnt_values(lft_manipulator_name)
                                                                                                         approach_direction=np.array(
                                                                                                             i),
                                                                                                         approach_distance=0.05,
                                                                                                         toggle_end_grasp=True,
                                                                                                         end_jawwidth=0.041,
                                                                                                         granularity=0.001
                                                                                                     )
                # left_st2twait_conf_list, left_st2twait_jawwidth_list = left_st2twait_conf_list[30:], left_st2twait_jawwidth_list[30:]
                if left_st2twait_conf_list:
                    print(f"正确的left_twait_pos = {left_twait_pos}, direction = {i}")
                    flag = True
                    break
            if flag:
                break
            s += 0.0001



        left_st2twait_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_st2twait_conf_list,
                                                                obj_pos=left_twait_pos,
                                                                obj_rotmat=left_twait_rotmat,
                                                                type='absolute')

        left_st2twait_none_pose_list = none_pose_list_generator(left_st2twait_conf_list)

        r_st2twait_conf_list, r_st2twait_jawwidth_list, r_st2twait_pose_list = dual_arm_static_generator(rgt_hand_name, left_st2twait_conf_list)

        left_twait2st_conf_list = left_st2twait_conf_list[::-1]
        left_twait2st_jawwidth_list = left_st2twait_jawwidth_list[::-1]
        left_twait2st_pose_list = left_st2twait_pose_list[::-1]
        left_twait2st_none_pose_list = left_st2twait_none_pose_list[::-1]

        r_twait2st_conf_list = r_st2twait_conf_list[::-1]
        r_twait2st_jawwidth_list = r_st2twait_jawwidth_list[::-1]
        r_twait2st_pose_list = r_st2twait_pose_list[::-1]



        left_conf_list += left_st2twait_conf_list
        left_jawwidth_list += left_st2twait_jawwidth_list
        left_pose_list += left_st2twait_none_pose_list


        right_conf_list += r_st2twait_conf_list
        right_jawwidth_list += r_st2twait_jawwidth_list
        right_pose_list += r_st2twait_pose_list





        left_wait2twait_conf_list = left_wait2st_conf_list + left_st2twait_conf_list
        left_wait2twait_jawwidth_list = left_wait2st_jawwidth_list + left_st2twait_jawwidth_list
        left_wait2twait_pose_list = left_wait2st_pose_list + left_st2twait_pose_list
        left_wait2twait_none_pose_list = left_wait2st_none_pose_list + left_st2twait_none_pose_list

        r_wait2twait_conf_list = r_wait2st_conf_list + r_st2twait_conf_list
        r_wait2twait_jawwidth_list = r_wait2st_jawwidth_list + r_st2twait_jawwidth_list
        r_wait2twait_pose_list = r_wait2st_pose_list + r_st2twait_pose_list

        left_twait2wait_conf_list = left_wait2twait_conf_list[::-1]
        left_twait2wait_jawwidth_list = left_wait2twait_jawwidth_list[::-1]
        left_twait2wait_pose_list = left_wait2twait_pose_list[::-1]
        left_twait2wait_none_pose_list = left_wait2twait_none_pose_list[::-1]

        r_twait2wait_conf_list = r_wait2twait_conf_list[::-1]
        r_twait2wait_jawwidth_list = r_wait2twait_jawwidth_list[::-1]
        r_twait2wait_pose_list = r_wait2twait_pose_list[::-1]

        sleep(3)




        left_twait2tmove_conf_list, left_twait2tmove_jawwidth_list = adplanner.gen_approach_motion(lft_manipulator_name,
                                                                                                 left_tmove_pos,  # left_tmove_rotmat.dot(left_jaw_center_pos) +
                                                                                                 left_tmove_rotmat,  # .dot(left_jaw_center_rotmat)
                                                                                                 start_conf=left_conf_list[-1],
                                                                                                 approach_direction=np.array(
                                                                                                     [0,0,-1]),
                                                                                                 approach_distance=0.05,
                                                                                                 toggle_end_grasp=True,
                                                                                                 end_jawwidth=0.041,
                                                                                                 granularity=0.001
                                                                                                   )

        left_twait2tmove_conf_list, left_twait2tmove_jawwidth_list = left_twait2tmove_conf_list[30:], left_twait2tmove_jawwidth_list[30:]
        left_twait2tmove_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_twait2tmove_conf_list,
                                                                obj_pos=left_tmove_pos,
                                                                obj_rotmat=left_tmove_rotmat,
                                                                type='absolute')

        left_twait2tmove_none_pose_list = none_pose_list_generator(left_twait2tmove_conf_list)


        r_twait2tmove_conf_list, r_twait2tmove_jawwidth_list, r_twait2tmove_pose_list = dual_arm_static_generator(rgt_hand_name, left_twait2tmove_conf_list)



        left_conf_list += left_twait2tmove_conf_list
        left_jawwidth_list += left_twait2tmove_jawwidth_list
        left_pose_list += left_twait2tmove_none_pose_list

        right_conf_list += r_twait2tmove_conf_list
        right_jawwidth_list += r_twait2tmove_jawwidth_list
        right_pose_list += r_twait2tmove_pose_list

        left_tmove2twait_conf_list = left_twait2tmove_conf_list[::-1]
        left_tmove2twait_jawwidth_list = left_twait2tmove_jawwidth_list[::-1]
        left_tmove2twait_pose_list = left_twait2tmove_pose_list[::-1]
        left_tmove2twait_none_pose_list = left_twait2tmove_none_pose_list[::-1]

        r_tmove2twait_conf_list = r_twait2tmove_conf_list[::-1]
        r_tmove2twait_jawwidth_list = r_twait2tmove_jawwidth_list[::-1]
        r_tmove2twait_pose_list = r_twait2tmove_pose_list[::-1]


        sleep(3)


        left_conf_list += left_tmove2twait_conf_list
        left_jawwidth_list += left_tmove2twait_jawwidth_list
        left_pose_list += left_tmove2twait_pose_list

        right_conf_list += r_tmove2twait_conf_list
        right_jawwidth_list += r_tmove2twait_jawwidth_list
        right_pose_list += r_tmove2twait_pose_list

        sleep(3)



        left_twait2thold_conf_list, left_twait2thold_jawwidth_list = adplanner.gen_approach_motion(lft_manipulator_name,
                                                                                                 left_thold_pos,  # left_thold_rotmat.dot(left_jaw_center_pos) +
                                                                                                 left_thold_rotmat,  # .dot(left_jaw_center_rotmat)
                                                                                                 start_conf=left_conf_list[-1],
                                                                                                 approach_direction=np.array(
                                                                                                     [0,0,-1]),
                                                                                                 approach_distance=0.05,
                                                                                                 toggle_end_grasp=True,
                                                                                                 end_jawwidth=0.041,
                                                                                                 granularity=0.001
                                                                                                   )

        left_twait2thold_conf_list, left_twait2thold_jawwidth_list = left_twait2thold_conf_list[30:], left_twait2thold_jawwidth_list[30:]
        left_twait2thold_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                conf_list=left_twait2thold_conf_list,
                                                                obj_pos=left_thold_pos,
                                                                obj_rotmat=left_thold_rotmat,
                                                                type='absolute')

        left_twait2thold_none_pose_list = none_pose_list_generator(left_twait2thold_conf_list)

        r_twait2thold_conf_list, r_twait2thold_jawwidth_list, r_twait2thold_pose_list = dual_arm_static_generator(rgt_hand_name, left_twait2thold_conf_list)



        left_conf_list += left_twait2thold_conf_list
        left_jawwidth_list += left_twait2thold_jawwidth_list
        left_pose_list += left_twait2thold_none_pose_list

        right_conf_list += r_twait2thold_conf_list
        right_jawwidth_list += r_twait2thold_jawwidth_list
        right_pose_list += r_twait2thold_pose_list


        left_thold2twait_conf_list = left_twait2thold_conf_list[::-1]
        left_thold2twait_jawwidth_list = left_twait2thold_jawwidth_list[::-1]
        left_thold2twait_pose_list = left_twait2thold_pose_list[::-1]
        left_thold2twait_none_pose_list = left_twait2thold_none_pose_list[::-1]


        r_thold2twait_conf_list = r_twait2thold_conf_list[::-1]
        r_thold2twait_jawwidth_list = r_twait2thold_jawwidth_list[::-1]
        r_thold2twait_pose_list = r_twait2thold_pose_list[::-1]

        sleep(3)





        left_thold2hold_conf_list = left_thold2twait_conf_list + left_twait2wait_conf_list + left_wait2hold_conf_list
        left_thold2hold_jawwidth_list = left_thold2twait_jawwidth_list + left_twait2wait_jawwidth_list + left_wait2hold_jawwidth_list
        left_thold2hold_pose_list = left_thold2twait_pose_list + left_twait2wait_pose_list + left_wait2hold_pose_list
        left_thold2hold_none_pose_list = left_thold2twait_none_pose_list + left_twait2wait_none_pose_list + left_wait2hold_none_pose_list

        r_thold2hold_conf_list = r_thold2twait_conf_list + r_twait2wait_conf_list + r_wait2hold_conf_list
        r_thold2hold_jawwidth_list = r_thold2twait_jawwidth_list + r_twait2wait_jawwidth_list + r_wait2hold_jawwidth_list
        r_thold2hold_pose_list = r_thold2twait_pose_list + r_twait2wait_pose_list + r_wait2hold_pose_list

        left_hold2thold_conf_list = left_thold2hold_conf_list[::-1]
        left_hold2thold_jawwidth_list = left_thold2hold_jawwidth_list[::-1]
        left_hold2thold_pose_list = left_thold2hold_pose_list[::-1]
        left_hold2thold_none_pose_list = left_thold2hold_none_pose_list[::-1]

        r_hold2thold_conf_list = r_thold2hold_conf_list[::-1]
        r_hold2thold_jawwidth_list = r_thold2hold_jawwidth_list[::-1]
        r_hold2thold_pose_list = r_thold2hold_pose_list[::-1]

        left_conf_list += left_thold2hold_conf_list
        left_jawwidth_list += left_thold2hold_jawwidth_list
        # left_pose_list += left_thold2hold_none_pose_list
        left_pose_list += [cube_center_pose for _ in range(len(left_thold2hold_conf_list))]

        right_conf_list += r_thold2hold_conf_list
        right_jawwidth_list += r_thold2hold_jawwidth_list
        right_pose_list += r_thold2hold_pose_list


    """
    ================================ 左臂拆解结束 ================================
    """

    if enable_right:
        # right_hold2wait_conf_list, right_hold2wait_jawwidth_list = adplanner.gen_approach_motion(rgt_manipluator_name,
        #                                                                                          right_wait_pos,  # right_wait_rotmat.dot(cube_jaw_center_pos) +
        #                                                                                          right_wait_rotmat,  # .dot(cube_jaw_center_rotmat)
        #                                                                                          start_conf=right_conf_list[
        #                                                                                              -1],
        #                                                                                          approach_direction=np.array(
        #                                                                                              [0, 1, 0]),
        #                                                                                          approach_distance=0.05,
        #                                                                                          toggle_end_grasp=True,
        #                                                                                          end_jawwidth=0.041,
        #                                                                                          granularity=0.001
        #                                                                                          )

        s = right_wait_pos[1]
        for _ in range(1):
            flag = False
            right_wait_pos[1] = s
            for i in direction_list:
                right_hold2wait_conf_list, right_hold2wait_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_wait_pos,#right_wait_rotmat.dot(cube_jaw_center_pos) +
                    right_wait_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array(
                        [0, -1, 0]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001)

                if right_hold2wait_conf_list:
                    print(f"正确的right_wait_pos = {right_wait_pos[1]}, direction = {i}")
                    flag = True
                    break
            if flag:
                break
            s += 0.0001

        right_hold2wait_conf_list, right_hold2wait_jawwidth_list = right_hold2wait_conf_list[30:], right_hold2wait_jawwidth_list[30:]
        # 爪子抓取魔方的运动序列
        right_hold2wait_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                                conf_list=right_hold2wait_jawwidth_list,
                                                                obj_pos=right_wait_pos,
                                                                obj_rotmat=right_wait_rotmat,
                                                                type='absolute')
        right_hold2wait_none_pose_list = none_pose_list_generator(right_hold2wait_jawwidth_list)




        l_hold2wait_conf_list, l_hold2wait_jawwidth_list, l_hold2wait_pose_list = dual_arm_static_generator(
            lft_hand_name, right_hold2wait_conf_list)

        left_conf_list += l_hold2wait_conf_list
        left_jawwidth_list += l_hold2wait_jawwidth_list
        left_pose_list += l_hold2wait_pose_list

        right_conf_list += right_hold2wait_conf_list
        right_jawwidth_list += right_hold2wait_jawwidth_list
        right_pose_list += right_hold2wait_none_pose_list




        right_wait2hold_conf_list = right_hold2wait_conf_list[::-1]
        right_wait2hold_jawwidth_list = right_hold2wait_jawwidth_list[::-1]
        right_wait2hold_pose_list = right_hold2wait_pose_list[::-1]
        right_wait2hold_none_pose_list = right_hold2wait_none_pose_list[::-1]


        l_wait2hold_conf_list = l_hold2wait_conf_list[::-1]
        l_wait2hold_jawwidth_list = l_hold2wait_jawwidth_list[::-1]
        l_wait2hold_pose_list = l_hold2wait_pose_list[::-1]

        sleep(6)

        s = right_rmove_pos[1]
        for _ in range(100):
            flag = False
            right_rmove_pos[1] = s
            for i in direction_list:
                right_wait2move_conf_list, right_wait2move_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_rmove_pos,#right_rmove_rotmat.dot(cube_jaw_center_pos) +
                    right_rmove_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array(
                        [0,1,0]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001
                )

                if right_wait2move_conf_list:
                    print(f"正确的right_rmove_pos = {right_rmove_pos[1]}, direction = {i}")
                    flag = True
                    break
            if flag:
                break
            s += 0.0001

        right_wait2move_conf_list, right_wait2move_jawwidth_list = right_wait2move_conf_list[30:], right_wait2move_jawwidth_list[30:]

        # 爪子抓取魔方的运动序列
        right_wait2move_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                                conf_list=right_wait2move_jawwidth_list,
                                                                obj_pos=right_rmove_pos,
                                                                obj_rotmat=right_rmove_rotmat,
                                                                type='absolute')
        right_wait2move_none_pose_list = none_pose_list_generator(right_wait2move_jawwidth_list)




        l_wait2move_conf_list, l_wait2move_jawwidth_list, l_wait2move_pose_list = dual_arm_static_generator(
            lft_hand_name, right_wait2move_conf_list)

        left_conf_list += l_wait2move_conf_list
        left_jawwidth_list += l_wait2move_jawwidth_list
        left_pose_list += l_wait2move_pose_list

        right_conf_list += right_wait2move_conf_list
        right_jawwidth_list += right_wait2move_jawwidth_list
        right_pose_list += right_wait2move_none_pose_list

        right_move2wait_conf_list = right_wait2move_conf_list[::-1]
        right_move2wait_jawwidth_list = right_wait2move_jawwidth_list[::-1]
        right_move2wait_pose_list = right_wait2move_pose_list[::-1]
        right_move2wait_none_pose_list = right_wait2move_none_pose_list[::-1]

        l_move2wait_conf_list = l_wait2move_conf_list[::-1]
        l_move2wait_jawwidth_list = l_wait2move_jawwidth_list[::-1]
        l_move2wait_pose_list = l_wait2move_pose_list[::-1]

        sleep(6)


        left_conf_list += l_move2wait_conf_list
        left_jawwidth_list += l_move2wait_jawwidth_list
        left_pose_list += l_move2wait_pose_list

        right_conf_list += right_move2wait_conf_list
        right_jawwidth_list += right_move2wait_jawwidth_list
        right_pose_list += right_move2wait_none_pose_list






        sleep(6)

        s = rgt_arm_home_pos[1]  # 正确的rgt_arm_home_pos = 0.0011, direction = [0, 0, 0]
        temp_direction_list = []
        for _ in range(10):
            flag = False
            rgt_arm_home_pos[1] = s
            for i in direction_list:
                right_wait2st_conf_list, right_wait2st_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    rgt_arm_home_pos,#rgt_arm_home_rotmat.dot(cube_jaw_center_pos) +
                    rgt_arm_home_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array(
                        i),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001)

                if right_wait2st_conf_list:
                    print(f"正确的rgt_arm_home_pos = {rgt_arm_home_pos[1]}, direction = {i}")
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001

        # right_wait2st_conf_list, right_wait2st_jawwidth_list = right_wait2st_conf_list[15:], right_wait2st_jawwidth_list[15:]

        right_wait2st_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                                conf_list=right_wait2st_jawwidth_list,
                                                                obj_pos=rgt_arm_home_pos,
                                                                obj_rotmat=rgt_arm_home_rotmat,
                                                                type='absolute')
        right_wait2st_none_pose_list = none_pose_list_generator(right_wait2st_jawwidth_list)




        l_wait2st_conf_list, l_wait2st_jawwidth_list, l_wait2st_pose_list = dual_arm_static_generator(
            lft_hand_name, right_wait2st_conf_list)



        left_conf_list += l_wait2st_conf_list
        left_jawwidth_list += l_wait2st_jawwidth_list
        left_pose_list += l_wait2st_pose_list

        right_conf_list += right_wait2st_conf_list
        right_jawwidth_list += right_wait2st_jawwidth_list
        right_pose_list += right_wait2st_none_pose_list



        right_st2wait_conf_list = right_wait2st_conf_list[::-1]
        right_st2wait_jawwidth_list = right_wait2st_jawwidth_list[::-1]
        right_st2wait_pose_list = right_wait2st_pose_list[::-1]
        right_st2wait_none_pose_list = right_wait2st_none_pose_list[::-1]

        l_st2wait_conf_list = l_wait2st_conf_list[::-1]
        l_st2wait_jawwidth_list = l_wait2st_jawwidth_list[::-1]
        l_st2wait_pose_list = l_wait2st_pose_list[::-1]


        s = right_hold_pos[1]  # 正确的right_st2twait_pos = 0.0011, direction = [0, 0, 0]
        temp_direction_list = []
        for _ in range(1):
            flag = False
            right_hold_pos[1] = s
            for i in direction_list:
                right_wait2hold_conf_list, right_wait2hold_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_hold_pos,# right_twait_rotmat.dot(cube_jaw_center_pos) +
                    right_hold_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_st2wait_conf_list[-1],
                    approach_direction=np.array([0,1,0]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.01)

                if right_wait2hold_conf_list:
                    print(
                        f"正确的right_twait_pos = {right_hold_pos[1]}, direction = {i}"
                    )
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001

        # right_st2twait_conf_list, right_st2twait_jawwidth_list = right_st2twait_conf_list[15:], right_st2twait_jawwidth_list[15:]

        right_wait2hold_pose_list = ppplanner.gen_object_motion(
            component_name=rgt_hand_name,
            conf_list=right_wait2hold_jawwidth_list,
            obj_pos=right_hold_pos,
            obj_rotmat=right_hold_rotmat,
            type='absolute')

        right_hold2wait_conf_list, right_hold2wait_jawwidth_list, right_hold2wait_pose_list = right_wait2hold_conf_list[::-1], right_wait2hold_jawwidth_list[::-1], right_wait2hold_pose_list[::-1]


        s = right_twait_pos[1]  # 正确的right_st2twait_pos = 0.0011, direction = [0, 0, 0]
        temp_direction_list = []
        for _ in range(10):
            flag = False
            right_twait_pos[1] = s
            for i in direction_list:
                right_st2twait_conf_list, right_st2twait_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_twait_pos,# right_twait_rotmat.dot(cube_jaw_center_pos) +
                    right_twait_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array([0,0,-1]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.01)

                if right_st2twait_conf_list:
                    print(
                        f"正确的right_twait_pos = {right_twait_pos[1]}, direction = {i}"
                    )
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001

        # right_st2twait_conf_list, right_st2twait_jawwidth_list = right_st2twait_conf_list[15:], right_st2twait_jawwidth_list[15:]
        print(f"right_st2twait_conf_list: {right_st2twait_conf_list}, \nright_st2twait_jawwidth_list: {right_st2twait_jawwidth_list}")
        right_st2twait_pose_list = ppplanner.gen_object_motion(
            component_name=rgt_hand_name,
            conf_list=right_st2twait_jawwidth_list,
            obj_pos=right_twait_pos,
            obj_rotmat=right_twait_rotmat,
            type='absolute')

        right_st2twait_none_pose_list = none_pose_list_generator(
            right_st2twait_jawwidth_list)

        l_st2twait_conf_list, l_st2twait_jawwidth_list, l_st2twait_pose_list = dual_arm_static_generator(
            lft_hand_name, right_st2twait_conf_list)

        left_conf_list += l_st2twait_conf_list
        left_jawwidth_list += l_st2twait_jawwidth_list
        left_pose_list += l_st2twait_pose_list

        right_conf_list += right_st2twait_conf_list
        right_jawwidth_list += right_st2twait_jawwidth_list
        right_pose_list += right_st2twait_none_pose_list




        right_twait2st_conf_list = right_st2twait_conf_list[::-1]
        right_twait2st_jawwidth_list = right_st2twait_jawwidth_list[::-1]
        right_twait2st_pose_list = right_st2twait_pose_list[::-1]
        right_twait2st_none_pose_list = right_st2twait_none_pose_list[::-1]

        l_twait2st_conf_list = l_st2twait_conf_list[::-1]
        l_twait2st_jawwidth_list = l_st2twait_jawwidth_list[::-1]
        l_twait2st_pose_list = l_st2twait_pose_list[::-1]




        right_twait2wait_conf_list = right_twait2st_conf_list + right_st2wait_conf_list
        right_twait2wait_jawwidth_list = right_twait2st_jawwidth_list + right_st2wait_jawwidth_list
        right_twait2wait_pose_list = right_twait2st_pose_list + right_st2wait_pose_list
        right_twait2wait_none_pose_list = right_twait2st_none_pose_list + right_st2wait_none_pose_list

        l_twait2wait_conf_list = l_twait2st_conf_list + l_st2wait_conf_list
        l_twait2wait_jawwidth_list = l_twait2st_jawwidth_list + l_st2wait_jawwidth_list
        l_twait2wait_pose_list = l_twait2st_pose_list + l_st2wait_pose_list





        right_wait2twait_conf_list = right_twait2wait_conf_list[::-1]
        right_wait2twait_jawwidth_list = right_twait2wait_jawwidth_list[::-1]
        right_wait2twait_pose_list = right_twait2wait_pose_list[::-1]
        right_wait2twait_none_pose_list = right_twait2wait_none_pose_list[::-1]

        l_wait2twait_conf_list = l_twait2wait_conf_list[::-1]
        l_wait2twait_jawwidth_list = l_twait2wait_jawwidth_list[::-1]
        l_wait2twait_pose_list = l_twait2wait_pose_list[::-1]




        sleep(6)


        s = right_tmove_pos[1]  # 正确的right_tmove_pos = 0.0011, direction = [0, 0, -1]
        temp_direction_list = []
        for _ in range(10):
            flag = False
            right_tmove_pos[1] = s
            for i in direction_list:
                right_twait2tmove_conf_list, right_twait2tmove_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_tmove_pos,#right_tmove_rotmat.dot(cube_jaw_center_pos) +
                    right_tmove_rotmat,  # .dot(cube_jaw_center_rotmat).dot(cube_jaw_center_rotmat),
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array([0, 0, -1]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001)

                if right_twait2tmove_conf_list:
                    print(
                        f"正确的right_tmove_pos = {right_tmove_pos[1]}, direction = {i}"
                    )
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001
        right_twait2tmove_conf_list, right_twait2tmove_jawwidth_list = right_twait2tmove_conf_list[30:], right_twait2tmove_jawwidth_list[30:]
        right_twait2tmove_pose_list = ppplanner.gen_object_motion(
            component_name=rgt_hand_name,
            conf_list=right_twait2tmove_jawwidth_list,
            obj_pos=right_tmove_pos,
            obj_rotmat=right_tmove_rotmat,
            type='absolute')
        right_twait2tmove_none_pose_list = none_pose_list_generator(
            right_twait2tmove_jawwidth_list)

        l_twait2tmove_conf_list, l_twait2tmove_jawwidth_list, l_twait2tmove_pose_list = dual_arm_static_generator(
            lft_hand_name, right_twait2tmove_conf_list)

        left_conf_list += l_twait2tmove_conf_list
        left_jawwidth_list += l_twait2tmove_jawwidth_list
        left_pose_list += l_twait2tmove_pose_list

        right_conf_list += right_twait2tmove_conf_list
        right_jawwidth_list += right_twait2tmove_jawwidth_list
        right_pose_list += right_twait2tmove_none_pose_list

        right_tmove2twait_conf_list = right_twait2tmove_conf_list[::-1]
        right_tmove2twait_jawwidth_list = right_twait2tmove_jawwidth_list[::-1]
        right_tmove2twait_pose_list = right_twait2tmove_pose_list[::-1]
        right_tmove2twait_none_pose_list = right_twait2tmove_none_pose_list[::-1]

        l_tmove2twait_conf_list = l_twait2tmove_conf_list[::-1]
        l_tmove2twait_jawwidth_list = l_twait2tmove_jawwidth_list[::-1]
        l_tmove2twait_pose_list = l_twait2tmove_pose_list[::-1]

        sleep(6)

        left_conf_list += l_tmove2twait_conf_list
        left_jawwidth_list += l_tmove2twait_jawwidth_list
        left_pose_list += l_tmove2twait_pose_list

        right_conf_list += right_tmove2twait_conf_list
        right_jawwidth_list += right_tmove2twait_jawwidth_list
        right_pose_list += right_tmove2twait_none_pose_list



        sleep(6)

        s = right_thold_pos[1]  # 正确的right_thold_pos = 0.0011, direction = [0, 0, -1]
        temp_direction_list = []
        for _ in range(10):
            flag = False
            right_thold_pos[1] = s
            for i in direction_list:
                right_twait2thold_conf_list, right_twait2thold_jawwidth_list = adplanner.gen_approach_motion(
                    rgt_manipluator_name,
                    right_thold_pos,#right_thold_rotmat.dot(cube_jaw_center_pos) +
                    right_thold_rotmat,  # .dot(cube_jaw_center_rotmat)
                    start_conf=right_conf_list[-1],
                    approach_direction=np.array([0, 0, -1]),
                    approach_distance=0.05,
                    toggle_end_grasp=True,
                    end_jawwidth=0.041,
                    granularity=0.001)

                if right_twait2thold_conf_list:
                    print(
                        f"正确的right_thold_pos = {right_thold_pos[1]}, direction = {i}"
                    )
                    flag = True
                    if 0:
                        temp_direction_list.append(i)
                    else:
                        break
            if flag:
                print(f"temp_direction_list: {temp_direction_list}")
                break
            s += 0.0001
        right_twait2thold_conf_list, right_twait2thold_jawwidth_list = right_twait2thold_conf_list[20:], right_twait2thold_jawwidth_list[20:]
        right_twait2thold_pose_list = ppplanner.gen_object_motion(
            component_name=rgt_hand_name,
            conf_list=right_twait2thold_jawwidth_list,
            obj_pos=right_thold_pos,
            obj_rotmat=right_thold_rotmat,
            type='absolute')
        right_twait2thold_none_pose_list = none_pose_list_generator(
            right_twait2thold_jawwidth_list)

        l_twait2thold_conf_list, l_twait2thold_jawwidth_list, l_twait2thold_pose_list = dual_arm_static_generator(
            lft_hand_name, right_twait2thold_conf_list)

        left_conf_list += l_twait2thold_conf_list
        left_jawwidth_list += l_twait2thold_jawwidth_list
        left_pose_list += l_twait2thold_pose_list

        right_conf_list += right_twait2thold_conf_list
        right_jawwidth_list += right_twait2thold_jawwidth_list
        right_pose_list += right_twait2thold_none_pose_list

        right_thold2twait_conf_list = right_twait2thold_conf_list[::-1]
        right_thold2twait_jawwidth_list = right_twait2thold_jawwidth_list[::-1]
        right_thold2twait_pose_list = right_twait2thold_pose_list[::-1]
        right_thold2twait_none_pose_list = right_twait2thold_none_pose_list[::-1]

        l_thold2twait_conf_list = l_twait2thold_conf_list[::-1]
        l_thold2twait_jawwidth_list = l_twait2thold_jawwidth_list[::-1]
        l_thold2twait_pose_list = l_twait2thold_pose_list[::-1]





        right_thold2hold_conf_list = right_thold2twait_conf_list + right_twait2wait_conf_list + right_wait2hold_conf_list
        right_thold2hold_jawwidth_list = right_thold2twait_jawwidth_list + right_twait2wait_jawwidth_list + right_wait2hold_jawwidth_list
        right_thold2hold_pose_list = right_thold2twait_pose_list + right_twait2wait_pose_list + right_wait2hold_pose_list
        right_thold2hold_none_pose_list = right_thold2twait_none_pose_list + right_twait2wait_none_pose_list + right_wait2hold_none_pose_list

        l_thold2hold_conf_list = l_thold2twait_conf_list + l_twait2wait_conf_list + l_wait2hold_conf_list
        l_thold2hold_jawwidth_list = l_thold2twait_jawwidth_list + l_twait2wait_jawwidth_list + l_wait2hold_jawwidth_list
        l_thold2hold_pose_list = l_thold2twait_pose_list + l_twait2wait_pose_list + l_wait2hold_pose_list


        right_hold2thold_conf_list = right_thold2hold_conf_list[::-1]
        right_hold2thold_jawwidth_list = right_thold2hold_jawwidth_list[::-1]
        right_hold2thold_pose_list = right_thold2hold_pose_list[::-1]
        right_hold2thold_none_pose_list = right_thold2hold_none_pose_list[::-1]

        l_hold2thold_conf_list = l_thold2hold_conf_list[::-1]
        l_hold2thold_jawwidth_list = l_thold2hold_jawwidth_list[::-1]
        l_hold2thold_pose_list = l_thold2hold_pose_list[::-1]





        right_conf_list += right_thold2hold_conf_list
        right_jawwidth_list += right_thold2hold_jawwidth_list
        right_pose_list += right_thold2hold_none_pose_list

        left_conf_list += l_thold2hold_conf_list
        left_jawwidth_list += l_thold2hold_jawwidth_list
        left_pose_list += l_thold2hold_pose_list

        sleep(10)



    import pickle

    pickle_start = False

    pickle_left = False
    pickle_right = True


    if pickle_start:
        start_dict = {}
        start_dict["cube_approach"] = [cube_approach_conf_list, cube_approach_jawwidth_list, cube_approach_pose_list]
        start_dict["cube_depart"] = [cube_approach_conf_list[::-1], cube_approach_jawwidth_list[::-1], cube_approach_pose_list[::-1]]
        start_dict["cube_holding_moveto"] = [cube_holding_moveto_conf_list, cube_holding_moveto_jawwidth_list, cube_holding_moveto_pose_list]
        start_dict["cube_holding_moveback"] = [cube_holding_moveto_conf_list[::-1], cube_holding_moveto_jawwidth_list[::-1], cube_holding_moveto_pose_list[::-1]]


        with open("start_dict.pickle", "wb") as file1:
            pickle.dump(start_dict, file1)
        print("成功写入start")

    if pickle_left and enable_left:
        left_dict = {}
        left_dict["left_st2wait"] = [left_st2wait_conf_list, left_st2wait_jawwidth_list, left_st2wait_pose_list]
        left_dict["left_wait2st"] = [left_wait2st_conf_list, left_wait2st_jawwidth_list, left_wait2st_pose_list]

        left_dict["left_wait2move"] = [left_wait2move_conf_list, left_wait2move_jawwidth_list, left_wait2move_pose_list]
        left_dict["left_move2wait"] = [left_move2wait_conf_list, left_move2wait_jawwidth_list, left_move2wait_pose_list]

        left_dict["left_wait2hold"] = [left_wait2hold_conf_list, left_wait2hold_jawwidth_list, left_wait2hold_pose_list]
        left_dict["left_hold2wait"] = [left_hold2wait_conf_list, left_hold2wait_jawwidth_list, left_hold2wait_pose_list]

        left_dict["left_st2twait"] = [left_st2twait_conf_list, left_st2twait_jawwidth_list, left_st2twait_pose_list]
        left_dict["left_twait2st"] = [left_twait2st_conf_list, left_twait2st_jawwidth_list, left_twait2st_pose_list]

        left_dict["left_twait2tmove"] = [left_twait2tmove_conf_list, left_twait2tmove_jawwidth_list, left_twait2tmove_pose_list]
        left_dict["left_tmove2twait"] = [left_tmove2twait_conf_list, left_tmove2twait_jawwidth_list, left_tmove2twait_pose_list]

        left_dict["left_twait2thold"] = [left_twait2thold_conf_list, left_twait2thold_jawwidth_list, left_twait2thold_pose_list]
        left_dict["left_thold2twait"] = [left_thold2twait_conf_list, left_thold2twait_jawwidth_list, left_thold2twait_pose_list]

        left_dict["left_twait2wait"] = [left_twait2wait_conf_list, left_twait2wait_jawwidth_list, left_twait2wait_pose_list]
        left_dict["left_wait2twait"] = [left_wait2twait_conf_list, left_wait2twait_jawwidth_list, left_wait2twait_pose_list]

        left_dict["left_hold2thold"] = [left_hold2thold_conf_list, left_hold2thold_jawwidth_list, left_hold2thold_pose_list]
        left_dict["left_thold2hold"] = [left_thold2hold_conf_list, left_thold2hold_jawwidth_list, left_thold2hold_pose_list]


        with open("left_dict.pickle", "wb") as file1:
            pickle.dump(left_dict, file1)
        print("成功写入left")


    if pickle_right and enable_right:
        right_dict = {}
        right_dict["right_st2wait"] = [right_st2wait_conf_list, right_st2wait_jawwidth_list, right_st2wait_pose_list]
        right_dict["right_wait2st"] = [right_wait2st_conf_list, right_wait2st_jawwidth_list, right_wait2st_pose_list]

        right_dict["right_wait2move"] = [right_wait2move_conf_list, right_wait2move_jawwidth_list,
                                         right_wait2move_pose_list]
        right_dict["right_move2wait"] = [right_move2wait_conf_list, right_move2wait_jawwidth_list,
                                         right_move2wait_pose_list]

        right_dict["right_wait2hold"] = [right_wait2hold_conf_list, right_wait2hold_jawwidth_list,
                                         right_wait2hold_pose_list]
        right_dict["right_hold2wait"] = [right_hold2wait_conf_list, right_hold2wait_jawwidth_list,
                                         right_hold2wait_pose_list]

        right_dict["right_st2twait"] = [right_st2twait_conf_list, right_st2twait_jawwidth_list, right_st2twait_pose_list]
        right_dict["right_twait2st"] = [right_twait2st_conf_list, right_twait2st_jawwidth_list, right_twait2st_pose_list]

        right_dict["right_twait2tmove"] = [right_twait2tmove_conf_list, right_twait2tmove_jawwidth_list,
                                           right_twait2tmove_pose_list]
        right_dict["right_tmove2twait"] = [right_tmove2twait_conf_list, right_tmove2twait_jawwidth_list,
                                           right_tmove2twait_pose_list]

        right_dict["right_twait2thold"] = [right_twait2thold_conf_list, right_twait2thold_jawwidth_list,
                                           right_twait2thold_pose_list]
        right_dict["right_thold2twait"] = [right_thold2twait_conf_list, right_thold2twait_jawwidth_list,
                                           right_thold2twait_pose_list]

        right_dict["right_twait2wait"] = [right_twait2wait_conf_list, right_twait2wait_jawwidth_list,
                                          right_twait2wait_pose_list]
        right_dict["right_wait2twait"] = [right_wait2twait_conf_list, right_wait2twait_jawwidth_list,
                                          right_wait2twait_pose_list]

        right_dict["right_hold2thold"] = [right_hold2thold_conf_list, right_hold2thold_jawwidth_list,
                                          right_hold2thold_pose_list]
        right_dict["right_thold2hold"] = [right_thold2hold_conf_list, right_thold2hold_jawwidth_list,
                                          right_thold2hold_pose_list]

        with open("right_dict.pickle", "wb") as file1:
            pickle.dump(right_dict, file1)
        print("成功写入right")





    robot_attached_list = []
    rgt_object_attached_list = []
    lft_object_attached_list = []
    object_attached_list = []
    counter = [0]


    def update_dual_robot(robot_s,
                          rgt_hand_name,
                          cube,
                          rgt_robot_path,
                          rgt_jawwidth_path,
                          rgt_obj_path,
                          lft_hand_name,
                          lft_robot_path,
                          lft_jawwidth_path,
                          lft_obj_path,
                          robot_attached_list,
                          rgt_object_attached_list,
                          lft_object_attached_list,
                          counter,
                          task):
        if counter[0] >= len(rgt_robot_path):
            counter[0] = 0
            time.sleep(3)
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            # for rgt_object_attached in rgt_object_attached_list:
            #     rgt_object_attached.detach()
            # for lft_object_attrach in lft_object_attached_list:
            #     lft_object_attrach.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            # rgt_object_attached_list.clear()
            # lft_object_attached_list.clear()
            object_attached_list.clear()
        # rgt arm update
        rgt_pose = rgt_robot_path[counter[0]]  # obtain the robot pose
        print(f"rgt_pose:{rgt_pose}")
        robot_s.fk(rgt_hand_name, rgt_pose)  # robot forward kinematics
        robot_s.jaw_to(rgt_hand_name, rgt_jawwidth_path[counter[0]])  # gripper kinematics
        # lft arm update
        lft_pose = lft_robot_path[counter[0]]
        robot_s.fk(lft_hand_name, lft_pose)  # robot forward kinematics
        robot_s.jaw_to(lft_hand_name, lft_jawwidth_path[counter[0]])  # gripper kinematics

        robot_meshmodel = robot_s.gen_meshmodel()  # show the robot
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)

        # rgt hand object
        rgt_obj_pose = rgt_obj_path[counter[0]]
        # rgt_objb_copy = cube.copy()
        # rgt_objb_copy.set_homomat(rgt_obj_pose)
        # rgt_objb_copy.attach_to(base)

        # lft hand object
        lft_obj_pose = lft_obj_path[counter[0]]
        # lft_objb_copy = cube.copy()
        # lft_objb_copy.set_homomat(lft_obj_pose)
        # lft_objb_copy.attach_to(base)
        print(f"rgt_obj_pose: {rgt_obj_pose}")
        if (rgt_obj_pose == 0).all():
            cube_pose = lft_obj_pose
        else:
            cube_pose = rgt_obj_pose
        # cube_pose = 0
        objb_copy = cube.copy()
        objb_copy.set_homomat(cube_pose)
        objb_copy.attach_to(base)

        # rgt_object_attached_list.append(rgt_objb_copy)
        # lft_object_attached_list.append(lft_objb_copy)
        object_attached_list.append(objb_copy)
        counter[0] += 1
        return task.again


    def update_one_robot(robot_s,
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


    show_dual_arm_animation = 1
    show_lft_arm_animation = 0
    show_rgt_arm_animation = 0
    if show_dual_arm_animation:
        taskMgr.doMethodLater(0.03, update_dual_robot, "update",
                              extraArgs=[u3ed,
                                         rgt_hand_name,
                                         cube,
                                         right_conf_list,
                                         right_jawwidth_list,
                                         right_pose_list,
                                         lft_hand_name,
                                         left_conf_list,
                                         left_jawwidth_list,
                                         left_pose_list,
                                         robot_attached_list,
                                         rgt_object_attached_list,
                                         lft_object_attached_list,
                                         counter],
                              appendTask=True)

    if show_lft_arm_animation:
        taskMgr.doMethodLater(0.05, update_one_robot, "update",
                              extraArgs=[u3ed,
                                         lft_hand_name,
                                         cube,
                                         left_conf_list,
                                         left_jawwidth_list,
                                         left_pose_list,
                                         robot_attached_list,
                                         lft_object_attached_list,
                                         counter],
                              appendTask=True)

    if show_rgt_arm_animation:
        taskMgr.doMethodLater(0.05, update_one_robot, "update",
                              extraArgs=[u3ed,
                                         rgt_hand_name,
                                         cube,
                                         right_conf_list,
                                         right_jawwidth_list,
                                         right_pose_list,
                                         robot_attached_list,
                                         rgt_object_attached_list,
                                         counter],
                              appendTask=True)

    base.run()
