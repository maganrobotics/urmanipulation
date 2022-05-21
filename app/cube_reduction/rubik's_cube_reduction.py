import time, sys, pickle, traceback

sys.path.append("./")
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


def get_lnk_bdmodel(robot_s, component_name, lnk_id):
    print(
        f"robot_s.manipulator_dict[component_name].lnks:{len(robot_s.manipulator_dict[component_name].lnks)} | {robot_s.manipulator_dict[component_name].lnks}"
    )
    lnk = robot_s.manipulator_dict[component_name].lnks[lnk_id]
    bd_lnk = bdm.BDModel(lnk["collisionmodel"],
                         mass=0,
                         type="box",
                         dynamic=False)
    bd_lnk.set_homomat(rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
    return bd_lnk


def update_robot_bdmodel(robot_s, bd_lnk_list):
    cnter = 0
    for arm_name in ["lft_arm", "rgt_arm"]:
        for lnk_id in [1, 2, 3, 4, 5, 6]:
            lnk = robot_s.manipulator_dict[arm_name].lnks[lnk_id]
            bd_lnk_list[cnter].set_homomat(
                rm.homomat_from_posrot(lnk["gl_pos"], lnk["gl_rotmat"]))
            cnter += 1


def get_robot_bdmoel(robot_s):
    bd_lnk_list = []
    for arm_name in ["lft_arm", "rgt_arm"]:
        for lnk_id in [0, 1, 2, 3, 4, 5, 6]:
            bd_lnk_list.append(get_lnk_bdmodel(robot_s, arm_name, lnk_id))
    return bd_lnk_list


if __name__ == '__main__':
    # set reduction setps
    steps_list = "R2 L' F U"

    # load arms_sequence
    with open("dual_dict.pickle", "rb") as file:
        dual_dcit = pickle.load(file)

    # load model
    # load base environment
    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    # load dual_arm robot
    u3ed = ur3e_dual.UR3EDual()

    # define place planner
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

    # set_space_position
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

    left_grasp_info_list = gutil.load_pickle_file(
        objcm_name='hole', file_name='define_hole.pickle')
    left_grasp_info = left_grasp_info_list[0]
    left_jaw_width, left_jaw_center_pos, left_jaw_center_rotmat, left_hnd_pos, left_hnd_rotmat = left_grasp_info

    cube_grasp_info_list = gutil.load_pickle_file(
        objcm_name='roller', file_name='define_roller.pickle')
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

    direction_list = [[0, 0, -1], [0, 0, 1], [-1, 0, 0], [0, -1, 0], [1, 0, 0],
                      [0, 1, 0]]

    def rotational(radian,
                   last_conf,
                   last_jawwidth,
                   last_pose,
                   rotational_type=True,
                   movement_range=20):
        """
        rotate grasper
        :param radian: 
        :param last_conf:
        :param last_pose:
        :param rotational_type:
        :param movement_range:
        :return

        """
        if str(rotational_type) == "True":
            flag = 1
        elif str(rotational_type) == "False":
            flag = -1
        else:
            raise ValueError("旋转方向参数异常")

        rotate_list = []
        for i in range(1, movement_range + 1):
            add_radian = flag * i * radian / movement_range
            temp_list = last_conf.copy()
            temp_list[-1] += add_radian
            rotate_list.append(temp_list)

        return rotate_list, [last_jawwidth for _ in range(movement_range)
                             ], [last_pose for _ in range(movement_range)]

    def sleep(times=3):
        """
        sleep
        :param times
        :return

        """
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

    def dual_arm_init_generator(component_name,
                                wait_conf_list,
                                last_motion="",
                                obj_pos=cube_location_pos,
                                obj_rotmat=cube_location_rotmat):
        """
        generator dual arm init sequences
        :param component_name: 
        :param wait_conf_list:
        :param last_motion:
        :param obj_pos:
        :param obj_rotmat:
        :return

        """
        if str(last_motion) == "":
            last_motion = u3ed.get_jnt_values(component_name)
        arm_static_conf_list, \
        arm_static_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=wait_conf_list,
                                                               conf=last_motion,
                                                               jawwidth=0.041)

        arm_static_pose_list = ppplanner.gen_object_motion(
            component_name=component_name,
            conf_list=arm_static_conf_list,
            obj_pos=obj_pos,
            obj_rotmat=obj_rotmat,
            type='absolute')
        return arm_static_conf_list, arm_static_jawwidth_list, arm_static_pose_list

    def dual_arm_static_generator(component_name, wait_conf):
        """
        generator dual arm static sequences
        :param component_name: 
        :param wait_conf_list:
        :return

        """
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

    def none_pose_list_generator(none_conf_list,
                                 obj_pose=np.array([[0, 0, 0, 0]
                                                    for _ in range(4)])):
        """
        generator dual arm static sequences
        :param component_name: 
        :param wait_conf_list:
        :return

        """
        none_pose_list = [obj_pose for _ in range(len(none_conf_list))]
        return none_pose_list

    def steps_to_sequences(steps):
        """
        generator sequences by steps
        :param steps: 
        :return

        """
        steps_list = steps.replace("U", "W").replace("D", "Y").\
            replace("L", "O").replace("R", "R").replace("F","G").\
            replace("B","B").split(" ")
        steps_list = [i for i in steps_list if i]
        action_list = []
        for i in steps_list:
            if len(i) == 1:
                action_list.append((i, math.pi / 2, True))
            elif "2" in i:
                action_list.append((i[:-1], math.pi, True))
            elif "'" in i:
                action_list.append((i[:-1], math.pi / 2, False))

        color_space_dict = {
            "W": "U",
            "Y": "D",
            "O": "L",
            "R": "R",
            "G": "F",
            "B": "B"
        }

        hand_available_space = {"left": ["L", "U"], "right": ["R", "U"]}
        holded_hand = "right"
        free_hand = "left"
        conf_list = []

        for i in action_list:
            color_face = i[0]
            rotation_angle = i[1]
            rotation_direction = i[2]
            if color_space_dict.get(color_face) in hand_available_space.get(
                    free_hand):
                if color_space_dict.get(color_face) == "U":
                    conf_list.append(f"{free_hand}_wait2tmove")
                else:
                    conf_list.append(f"{free_hand}_wait2move")
                conf_list.append(
                    f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")
                if color_space_dict.get(color_face) == "U":
                    conf_list.append(f"{free_hand}_tmove2wait_spinback")
                else:
                    conf_list.append(f"{free_hand}_move2wait_spinback")
            elif color_space_dict.get(color_face) in hand_available_space.get(
                    holded_hand):
                conf_list.append(f"{free_hand}_wait2hold")
                conf_list.append(f"{holded_hand}_hold2wait")

                holded_hand, free_hand = free_hand, holded_hand

                conf_list.append(f"{free_hand}_wait2move")
                conf_list.append(
                    f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")
                conf_list.append(f"{free_hand}_move2wait_spinback")

            if color_space_dict.get(color_face) in ["D", "F", "B"]:
                cube_rotation_angle = math.pi / 2
                if color_space_dict.get(color_face) == "F":
                    if holded_hand == "left":
                        cube_rotation_direction = False
                    else:
                        cube_rotation_direction = True
                elif color_space_dict.get(color_face) == "B":
                    if holded_hand == "left":
                        cube_rotation_direction = True
                    else:
                        cube_rotation_direction = False
                elif color_space_dict.get(color_face) == "D":
                    cube_rotation_direction = True
                    cube_rotation_angle = math.pi
                conf_list.append(
                    f"{holded_hand}_spin_{cube_rotation_angle}_{cube_rotation_direction}"
                )
                color_space_dict[color_face] = "U"
                temp_y = "WBYGWBYG"
                for i in range(len(temp_y)):
                    if temp_y[i] == color_face:
                        color_space_dict[temp_y[i + 1]] = "B"
                        color_space_dict[temp_y[i + 2]] = "D"
                        color_space_dict[temp_y[i + 3]] = "F"
                        break

                conf_list.append(f"{free_hand}_wait2hold")
                conf_list.append(f"{holded_hand}_hold2wait_spinback")
                holded_hand, free_hand = free_hand, holded_hand
                conf_list.append(f"{free_hand}_wait2tmove")
                conf_list.append(
                    f"{free_hand}_spin_{rotation_angle}_{rotation_direction}")
                conf_list.append(f"{free_hand}_tmove2wait_spinback")
        print(conf_list)
        return (conf_list)

    def dual_conf_generate(steps_list):
        """
        generator dual conf
        :param steps_list: 
        :return

        """
        global left_conf_list, left_jawwidth_list, left_pose_list, \
            right_conf_list, right_jawwidth_list, right_pose_list
        spin_angle = 0
        temp_spin_angle = 0
        temp_spin_direction = True
        for i in steps_list:
            if not i:
                continue
            try:
                if "left" in i:
                    hand_name = "left"
                    last_conf = left_conf_list[-1]
                    last_jawwidth = left_jawwidth_list[-1]
                    last_pose = left_pose_list[-1]
                else:
                    hand_name = "right"
                    last_conf = right_conf_list[-1]
                    last_jawwidth = right_jawwidth_list[-1]
                    last_pose = right_pose_list[-1]

                if "spinback" in i:
                    step = i.split("_spinback")[0]
                    temp_conf_list, move_jawwidth_list, move_pose_list = dual_dcit.get(
                        step)
                    move_conf_list = []
                    for j in temp_conf_list:
                        temp = j.copy()
                        temp[-1] = float(temp_spin_angle)
                        move_conf_list.append(temp)
                    temp_spin_conf_list, temp_spin_jawwidth_list, temp_spin_pose_list = \
                        rotational(float(spin_angle), move_conf_list[-1], move_jawwidth_list[-1],
                                   move_pose_list[-1], rotational_type=temp_spin_direction)
                    move_conf_list += temp_spin_conf_list
                    move_jawwidth_list += temp_spin_jawwidth_list
                    move_pose_list += temp_spin_pose_list

                elif "spin" in i:
                    spin_angle, spin_direction = i.split("_")[-2:]
                    move_conf_list, move_jawwidth_list, move_pose_list = \
                        rotational(float(spin_angle), last_conf, last_jawwidth,
                                   last_pose, rotational_type=spin_direction)
                    temp_spin_angle = move_conf_list[-1][-1]
                    if spin_direction == "True":
                        temp_spin_direction = "False"
                    else:
                        temp_spin_direction = "True"
                else:
                    move_conf_list, move_jawwidth_list, move_pose_list = dual_dcit.get(
                        i)

                if hand_name == "left":
                    left_conf_list += move_conf_list
                    left_jawwidth_list += move_jawwidth_list
                    left_pose_list += none_pose_list_generator(
                        move_conf_list, cube_center_pose)

                    static_conf_list, static_jawwidth_list, static_pose_list = \
                        dual_arm_static_generator("rgt_arm",move_conf_list)

                    right_conf_list += static_conf_list
                    right_jawwidth_list += static_jawwidth_list
                    right_pose_list += static_pose_list
                elif hand_name == "right":
                    right_conf_list += move_conf_list
                    right_jawwidth_list += move_jawwidth_list
                    right_pose_list += none_pose_list_generator(
                        move_conf_list, cube_center_pose)

                    static_conf_list, static_jawwidth_list, static_pose_list = \
                        dual_arm_static_generator("lft_arm",move_conf_list)

                    left_conf_list += static_conf_list
                    left_jawwidth_list += static_jawwidth_list
                    left_pose_list += static_pose_list
                sleep(1)
            except:
                traceback.print_exc()
                print(f"错误动作: {i}")
                sys.exit(1)

    for i in sys.path:
        rubik_model_path = i + "rubik's_cube/rubiks.bam"
        if os.path.exists(rubik_model_path):
            break
    cube_model = base.loader.loadModel(rubik_model_path)
    cube_model.setScale(0.00645)

    cube = cm.CollisionModel(cube_model, cdprimit_type='box')

    cube.set_pos(cube_location_pos)
    cube.set_rotmat(cube_location_rotmat)

    s = cube_pick_pos[2]
    temp_direction_list = []
    for _ in range(1):
        flag = False
        cube_pick_pos[2] = s
        for i in direction_list:
            # 靠近魔方的动作序列
            cube_approach_conf_list, cube_approach_jawwidth_list = adplanner.gen_approach_motion(
                rgt_manipluator_name,
                cube_pick_pos,
                cube_pick_rotmat.dot(cube_jaw_center_rotmat),
                start_conf=u3ed.get_jnt_values(rgt_manipluator_name),
                approach_direction=np.array([0, 0, -1]),
                approach_distance=0.05,
                toggle_end_grasp=True,
                end_jawwidth=0.041,
                granularity=0.001)
            if cube_approach_conf_list:
                print(
                    f"正确的cube_pick_pos = {cube_pick_pos[2]}, direction = {i}")
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
    cube_approach_pose_list = ppplanner.gen_object_motion(
        component_name=rgt_hand_name,
        conf_list=cube_approach_jawwidth_list,
        obj_pos=cube_location_pos,
        obj_rotmat=cube_location_rotmat,
        type='absolute')

    left_conf_list = left_start_conf_list
    left_jawwidth_list = left_start_jawwidth_list
    left_pose_list = left_start_pose_list

    right_conf_list = cube_approach_conf_list
    right_jawwidth_list = cube_approach_jawwidth_list
    right_pose_list = cube_approach_pose_list

    cube_homomat_list.append(
        rm.homomat_from_posrot(cube_pick_pos, cube_location_rotmat))
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

    # 拼接魔方的运动序列
    cube_conf_list += cube_approach_conf_list
    cube_conf_list += cube_holding_moveto_conf_list

    # 拼接爪子抓魔方的运动序列
    cube_jawwidth_list += cube_approach_jawwidth_list
    cube_jawwidth_list += cube_holding_moveto_jawwidth_list

    # 拼接魔方姿态
    cube_pose_list += cube_approach_pose_list
    cube_pose_list += cube_holding_moveto_pose_list

    # 生成左臂初始静态的运动序列
    left_static_list = ppplanner.gen_object_motion(
        component_name=lft_hand_name,
        conf_list=cube_conf_list,
        obj_pos=cube_pick_pos,
        obj_rotmat=cube_pick_rotmat,
        type='absolute')
    left_static_conf_list, \
    left_static_jawwidth_list = ppplanner.gen_static_motion(
        the_other_conf_list=cube_conf_list,
        conf=u3ed.get_jnt_values(lft_manipulator_name),
        jawwidth=0.041)

    left_conf_list += left_static_conf_list
    left_jawwidth_list += left_static_jawwidth_list
    left_pose_list += left_static_list

    right_conf_list += cube_holding_moveto_conf_list
    right_jawwidth_list += cube_holding_moveto_jawwidth_list
    right_pose_list += cube_holding_moveto_pose_list

    s = left_wait_pos[1]
    temp_direction_list = []
    for _ in range(10):
        flag = False
        left_wait_pos[1] = s
        for i in direction_list:
            left_st2wait_conf_list, left_st2wait_jawwidth_list = adplanner.gen_approach_motion(
                lft_manipulator_name,
                left_wait_pos,
                left_wait_rotmat,
                start_conf=u3ed.get_jnt_values(lft_manipulator_name),
                approach_direction=np.array([0, 0, 1]),
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

    left_st2wait_pose_list = ppplanner.gen_object_motion(
        component_name=lft_hand_name,
        conf_list=left_st2wait_jawwidth_list,
        obj_pos=left_wait_pos,
        obj_rotmat=left_wait_rotmat,
        type='absolute')
    left_st2wait_none_pose_list = none_pose_list_generator(
        left_st2wait_conf_list)

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
    left_pose_list += [
        cube_center_pose for _ in range(len(left_st2wait_conf_list))
    ]

    steps = steps_to_sequences(steps_list)
    dual_conf_generate(steps)

    sleep(10)
    robot_attached_list = []
    rgt_object_attached_list = []
    lft_object_attached_list = []
    object_attached_list = []
    counter = [0]

    def update_dual_robot(robot_s, rgt_hand_name, cube, rgt_robot_path,
                          rgt_jawwidth_path, rgt_obj_path, lft_hand_name,
                          lft_robot_path, lft_jawwidth_path, lft_obj_path,
                          robot_attached_list, rgt_object_attached_list,
                          lft_object_attached_list, counter, task):
        if counter[0] >= len(rgt_robot_path):
            counter[0] = 0
            time.sleep(3)
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            object_attached_list.clear()

        # rgt arm update
        rgt_pose = rgt_robot_path[counter[0]]  # obtain the robot pose
        robot_s.fk(rgt_hand_name, rgt_pose)  # robot forward kinematics
        robot_s.jaw_to(rgt_hand_name,
                       rgt_jawwidth_path[counter[0]])  # gripper kinematics
        # lft arm update
        lft_pose = lft_robot_path[counter[0]]
        robot_s.fk(lft_hand_name, lft_pose)  # robot forward kinematics
        robot_s.jaw_to(lft_hand_name,
                       lft_jawwidth_path[counter[0]])  # gripper kinematics

        robot_meshmodel = robot_s.gen_meshmodel()  # show the robot
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)

        # rgt hand object
        rgt_obj_pose = rgt_obj_path[counter[0]]

        # lft hand object
        lft_obj_pose = lft_obj_path[counter[0]]
        if (rgt_obj_pose == 0).all():
            cube_pose = lft_obj_pose
        else:
            cube_pose = rgt_obj_pose
        objb_copy = cube.copy()
        objb_copy.set_homomat(cube_pose)
        objb_copy.attach_to(base)
        object_attached_list.append(objb_copy)
        counter[0] += 1
        return task.again

    taskMgr.doMethodLater(0.03,
                          update_dual_robot,
                          "update",
                          extraArgs=[
                              u3ed, rgt_hand_name, cube, right_conf_list,
                              right_jawwidth_list, right_pose_list,
                              lft_hand_name, left_conf_list,
                              left_jawwidth_list, left_pose_list,
                              robot_attached_list, rgt_object_attached_list,
                              lft_object_attached_list, counter
                          ],
                          appendTask=True)

    base.run()
