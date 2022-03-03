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

"""
Author: Ziqi Xu
date :  2022/01/17

"""





if __name__ == '__main__':

    # load model
    # load base environment
    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    # load dual_arm robot
    u3ed = ur3e_dual.UR3EDual()
    show_static_robot_model = False
    if show_static_robot_model:
        u3ed_meshmodel = u3ed.gen_meshmodel(toggle_tcpcs=False)  # toggle_tcpcs: whether to show the gripper's frame
        u3ed_meshmodel.attach_to(base)
        u3ed.gen_stickmodel().attach_to(base)

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

    # load roller and pole model
    # load roller
    rollerpath = os.path.join(basis.__path__[0], 'objects', 'roller.stl')
    roller = cm.CollisionModel(rollerpath, cdprimit_type='polygons')
    # set rotation
    rotmat = rm.rotmat_from_euler(math.radians(180), 0, 0)
    # load hole
    holepath = os.path.join(basis.__path__[0], 'objects', 'hole.stl')
    hole = cm.CollisionModel(holepath, cdprimit_type='polygons')
    # load the roller_grasp_info_list
    roller_grasp_info_list = gutil.load_pickle_file(objcm_name='roller', file_name='define_roller.pickle')
    roller_grasp_info = roller_grasp_info_list[0]
    # load the hole_grasp_info_list
    hole_grasp_info_list = gutil.load_pickle_file(objcm_name='hole', file_name='define_hole.pickle')
    hole_grasp_info = hole_grasp_info_list[0]
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

    # predefine roller_homomat_list and hole_homomat_list
    roller_homomat_list = []
    hole_homomat_list = []

    # define the pick pos and rotmat
    roller_pick_pos = np.array([0.7, -0.3, 0.85])
    roller_pick_rotmat = rotmat
    roller_homomat_list.append(rm.homomat_from_posrot(roller_pick_pos, roller_pick_rotmat))
    hole_pick_pos = np.array([0.7, 0.25, 0.85])
    hole_pick_rotmat = rotmat
    hole_homomat_list.append(rm.homomat_from_posrot(hole_pick_pos, hole_pick_rotmat))
    print("homomat are", rm.homomat_from_posrot(hole_pick_pos, hole_pick_rotmat))
    print("type of homomat", type(rm.homomat_from_posrot(hole_pick_pos, hole_pick_rotmat)))

    # show the model
    roller.set_rgba([0.2, 0, 0.7, 1.0])
    roller.attach_to(base)
    roller.set_pos(roller_pick_pos)
    roller.show_localframe()
    roller.set_rotmat(roller_pick_rotmat)
    hole.set_rgba([0.7, 0, 0.7, 0.6])
    hole.attach_to(base)
    hole.set_pos(hole_pick_pos)
    hole.show_localframe()
    hole.set_rotmat(hole_pick_rotmat)

    # define the roller tmp pos and rotmat
    tmp_roller = cm.CollisionModel(rollerpath, cdprimit_type='polygons')
    roller_tmp_pos = np.array([0.6, -0.05, 1.0])
    roller_tmp_rotmat = rm.rotmat_from_euler(math.radians(-90), 0, 0)
    roller_homomat_list.append(rm.homomat_from_posrot(roller_tmp_pos, roller_tmp_rotmat))
    # show the tmp_roller
    show_tmp_roller_pose = False
    if show_tmp_roller_pose:
        tmp_roller.set_pos(roller_tmp_pos)
        tmp_roller.set_rotmat(roller_tmp_rotmat)
        tmp_roller.set_rgba([0.2, 0, 0.7, 1.0])
        tmp_roller.show_localframe()
        tmp_roller.attach_to(base)

    # # define the hole tmp pos and rotmat
    tmp_hole = cm.CollisionModel(holepath, cdprimit_type='polygons')
    tmp_hole_pos = np.array([0.6, 0.02, 1.0])
    tmp_hole_rotmat = rm.rotmat_from_euler(math.radians(90), 0, 0)
    hole_homomat_list.append(rm.homomat_from_posrot(tmp_hole_pos, tmp_hole_rotmat))
    # show the tmp_hole
    show_tmp_hole = False
    if show_tmp_hole:
        tmp_hole.set_pos(tmp_hole_pos)
        tmp_hole.set_rotmat(tmp_hole_rotmat)
        tmp_hole.set_rgba([0.6, 0, 0.7, 0.5])
        tmp_hole.show_localframe()
        tmp_hole.attach_to(base)

    #
    # show the last assembly position
    # move the roller and the hole remain stationary
    tmp_roller_assembly = cm.CollisionModel(rollerpath, cdprimit_type='polygons')
    assembly_pos = np.array([0.6, 0.0035, 1.0])
    assembly_rotmat = rm.rotmat_from_euler(math.radians(-90), 0, 0)
    # roller_homomat_list.append(rm.homomat_from_posrot(assembly_pos, assembly_rotmat))
    # show assembly roller
    show_assembly_roller = False
    if show_assembly_roller:
        tmp_roller_assembly.set_pos(assembly_pos)
        tmp_roller_assembly.set_rotmat(assembly_rotmat)
        tmp_roller_assembly.set_rgba([0.2, 0, 0.7, 1.0])
        # tmp_roller.show_localframe()
        tmp_roller_assembly.attach_to(base)

    # define the place pose
    place_pos = np.array([0.7, 0.25, 0.83])
    place_rotmat = rotmat
    # define the move list
    hole_conf_list = []
    hole_jawwidth_list = []
    hole_pose_list = []

    roller_conf_list = []
    roller_jawwidth_list = []
    roller_pose_list = []


    # roller_pick
    roller_jaw_width, roller_jaw_center_pos, roller_jaw_center_rotmat, roller_hnd_pos, roller_hnd_rotmat = roller_grasp_info
    roller_approach_conf_list, roller_approach_jawwidth_list = adplanner.gen_approach_motion(rgt_manipluator_name,
                                                                                             roller_pick_rotmat.dot(roller_jaw_center_pos)+roller_pick_pos,
                                                                                             roller_pick_rotmat.dot(roller_jaw_center_rotmat),
                                                                                             start_conf=u3ed.get_jnt_values(
                                                                                                 rgt_manipluator_name),
                                                                                             approach_direction=np.array(
                                                                                                 [0, 0, -1]),
                                                                                             approach_distance=.07,
                                                                                             toggle_end_grasp=True,
                                                                                             end_jawwidth=0.011)
    roller_approach_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                            conf_list=roller_approach_jawwidth_list,
                                                            obj_pos=roller_pick_pos,
                                                            obj_rotmat=roller_pick_rotmat,
                                                            type='absolute')
    # roller_holding_moveto
    roller_holding_moveto_conf_list, \
    roller_holding_moveto_jawwidth_list, \
    roller_holding_moveto_pose_list = ppplanner.gen_holding_moveto(hand_name=rgt_hand_name,
                                                                   objcm=roller,
                                                                   grasp_info=roller_grasp_info,
                                                                   obj_pose_list=roller_homomat_list,
                                                                   depart_direction_list=np.array([0, 0, -1])*len(roller_homomat_list),
                                                                   approach_direction_list=np.array([0, 0, 1])*len(roller_homomat_list),
                                                                   depart_distance_list=[0.01]*len(roller_homomat_list),
                                                                   approach_distance_list=[0.01]*len(roller_homomat_list),
                                                                   ad_granularity=0.001,
                                                                   use_rrt=True,
                                                                   # seed_jnt_values=u3ed.get_jnt_ranges(rgt_manipluator_name)
                                                                   seed_jnt_values=roller_approach_conf_list[-1])

    # hole_pick
    hole_jaw_width, hole_jaw_center_pos, hole_jaw_center_rotmat, hole_hnd_pos, hole_hnd_rotmat = hole_grasp_info
    hole_approach_conf_list, hole_approach_jawwidth_list = adplanner.gen_approach_motion(lft_manipulator_name,
                                                                                         hole_pick_rotmat.dot(hole_jaw_center_pos)+hole_pick_pos,
                                                                                         hole_pick_rotmat.dot(hole_jaw_center_rotmat),
                                                                                         start_conf=u3ed.get_jnt_values(
                                                                                             lft_manipulator_name),
                                                                                         approach_direction=np.array(
                                                                                             [0, 0, -1]),
                                                                                         approach_distance=.05,
                                                                                         toggle_end_grasp=True,
                                                                                         end_jawwidth=0.012)
    hole_approach_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                          conf_list=hole_approach_jawwidth_list,
                                                          obj_pos=hole_pick_pos,
                                                          obj_rotmat=hole_pick_rotmat,
                                                          type='absolute')
    # print("the hole approach pose list are", len(hole_approach_pose_list))
    # hole_holding_moveto
    hole_holding_moveto_conf_list, \
    hole_holding_moveto_jawwidth_list, \
    hole_holding_moveto_pose_list = ppplanner.gen_holding_moveto(hand_name=lft_hand_name,
                                                                 objcm=hole,
                                                                 grasp_info=hole_grasp_info,
                                                                 obj_pose_list=hole_homomat_list,
                                                                 depart_direction_list=np.array([0, 0, -1])*len(hole_homomat_list),
                                                                 approach_direction_list=np.array([0, -1, 0])*len(hole_homomat_list),
                                                                 depart_distance_list=[0.05] * len(
                                                                     hole_homomat_list),
                                                                 approach_distance_list=[0.05] * len(
                                                                     hole_homomat_list),
                                                                 ad_granularity=0.001,
                                                                 use_rrt=True,
                                                                 seed_jnt_values=hole_approach_conf_list[-1])
    hole_conf_list += hole_approach_conf_list
    hole_conf_list += hole_holding_moveto_conf_list

    hole_jawwidth_list += hole_approach_jawwidth_list
    hole_jawwidth_list += hole_holding_moveto_jawwidth_list

    hole_pose_list += hole_approach_pose_list
    hole_pose_list +=hole_holding_moveto_pose_list
    # roller_assembly
    roller_assembly_conf_list = inik_svlr.gen_linear_motion(rgt_hand_name, start_tcp_pos=roller_tmp_pos,
                                                            start_tcp_rotmat=roller_tmp_rotmat,
                                                            goal_tcp_pos=assembly_pos,
                                                            goal_tcp_rotmat=assembly_rotmat)
    print("the roller assembly conf list are", roller_assembly_conf_list)
    roller_assembly_jawwidth_list = adplanner.gen_jawwidth_motion(roller_assembly_conf_list, jawwidth=0.011)
    roller_assembly_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                            conf_list=roller_assembly_conf_list,
                                                            obj_pos=assembly_pos,
                                                            obj_rotmat=assembly_rotmat,
                                                            type='relative')

    # # hole_depart_and_place_motion
    hole_depart_and_place_conf_list,\
    hole_depart_and_place_jawwidth_list,\
    hole_depart_and_place_pose_list = ppplanner.gen_holding_moveto(hand_name=lft_hand_name,
                                                                   objcm=hole,
                                                                   grasp_info=hole_grasp_info,
                                                                   obj_pose_list=hole_homomat_list[::-1],
                                                                   depart_direction_list=np.array([0, 0, -1]),
                                                                   depart_distance_list=[0.1]*len(hole_homomat_list),
                                                                   approach_direction_list=np.array([0, 0, 1]),
                                                                   approach_distance_list=[0.1]*len(hole_homomat_list),
                                                                   ad_granularity=-0.001,
                                                                   use_rrt=True,
                                                                   seed_jnt_values=hole_holding_moveto_conf_list[-1])

    # generate rgt_arm static motion  wait for the lft arm to grab. This is for digital twin.
    roller_static_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                     conf_list=hole_conf_list,
                                                     obj_pos=roller_pick_pos,
                                                     obj_rotmat=roller_pick_rotmat,
                                                     type='absolute')
    roller_static_conf_list, \
    roller_static_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=hole_approach_conf_list + hole_holding_moveto_conf_list,
                                                              conf=u3ed.get_jnt_values(rgt_manipluator_name),
                                                              jawwidth=0.05)
    # generate roller move with hole. wait for the lft arm depart and place. This is for digital twin.
    roller_move_with_hole_pose_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                                  conf_list=hole_depart_and_place_conf_list,
                                                                  obj_pos=place_pos,
                                                                  obj_rotmat=place_rotmat,
                                                                  type='relative')
    # generate rgt_arm static motion. wait for the lft arm depart and place. This is for digital twin.
    rgt_static_for_lft_depart_conf_list, \
    rgt_static_for_lft_depart_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=hole_depart_and_place_conf_list,
                                                                          conf=roller_assembly_conf_list[-1],
                                                                          jawwidth=0.04)

    # generate lft_arm static motion. Wait for the rgt arm to grab and assembly. This is for digital
    hole_static_list = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                   conf_list=roller_approach_conf_list + roller_holding_moveto_conf_list+roller_assembly_conf_list,
                                                   obj_pos=tmp_hole_pos,
                                                   obj_rotmat=tmp_hole_rotmat)
    hole_static_conf_list, hole_static_jawwidth_list = ppplanner.gen_static_motion(the_other_conf_list=hole_static_list,
                                                                                   conf=hole_holding_moveto_conf_list[-1],
                                                                                   jawwidth=0.012)

    # reset robot
    # rgt_arm reset
    rgt_arm_reset_conf_list, \
    rgt_hand_reset_jawwidth_list = adplanner.gen_depart_motion(component_name=rgt_manipluator_name,
                                                               start_tcp_pos=assembly_pos,
                                                               start_tcp_rotmat=assembly_rotmat,
                                                               end_conf=rgt_arm_home_conf,
                                                               depart_jawwidth=0.05)
    roller_static_while_reset = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                            conf_list=rgt_arm_reset_conf_list,
                                                            obj_pos=place_pos,
                                                            obj_rotmat=place_rotmat,
                                                            type='absolute')

    # lft_arm reset
    lft_arm_reset_conf_list, \
    lft_hand_reset_jawwidth_list = adplanner.gen_depart_motion(component_name=lft_manipulator_name,
                                                               start_tcp_pos=place_pos,
                                                               start_tcp_rotmat=place_rotmat,
                                                               end_conf=lft_arm_home_conf,
                                                               depart_direction=np.array([0,0,1]),
                                                               depart_jawwidth=0.05)
    hole_static_while_reset = ppplanner.gen_object_motion(component_name=lft_hand_name,
                                                          conf_list=lft_arm_reset_conf_list,
                                                          obj_pos=place_pos,
                                                          obj_rotmat=place_rotmat,
                                                          type='absolute')
    #
    print("the length of rgt_arm reset conf ", len(rgt_arm_reset_conf_list))
    print("the length of lft_arm reset conf ", len(lft_arm_reset_conf_list))
    has_reached_static_conf_range = abs(len(lft_arm_reset_conf_list)-len(rgt_arm_reset_conf_list))
    # When the right arm has reached the initial position and the left arm is still moving,
    # some static images need to be generated for digital twin
    rgt_arm_has_reached_static_conf_list, \
    rgt_hand_has_reached_static_jawwidth_list = ppplanner.gen_static_motion_with_range(conf_range=has_reached_static_conf_range,
                                                                                       conf=rgt_arm_home_conf,
                                                                                       jawwidth=0.05)
    roller_has_reached_static_pose_list = ppplanner.gen_object_motion(component_name=rgt_hand_name,
                                                                      conf_list=rgt_arm_has_reached_static_conf_list,
                                                                      obj_pos=place_pos,
                                                                      obj_rotmat=place_rotmat,
                                                                      type='absolute')
    # this is for digital twin
    roller_conf_list = roller_conf_list + roller_static_conf_list + roller_approach_conf_list + \
                       roller_holding_moveto_conf_list + \
                       roller_assembly_conf_list + \
                       rgt_static_for_lft_depart_conf_list + \
                       rgt_arm_reset_conf_list+ rgt_arm_has_reached_static_conf_list

    roller_jawwidth_list = roller_jawwidth_list + roller_static_jawwidth_list \
                           + roller_approach_jawwidth_list \
                           + roller_holding_moveto_jawwidth_list \
                           + roller_assembly_jawwidth_list \
                           + roller_static_jawwidth_list \
                           + rgt_hand_reset_jawwidth_list + rgt_hand_has_reached_static_jawwidth_list

    roller_pose_list = roller_pose_list + roller_static_list \
                       + roller_approach_pose_list \
                       + roller_holding_moveto_pose_list \
                       + roller_assembly_pose_list + roller_move_with_hole_pose_list \
                       + roller_static_while_reset + roller_has_reached_static_pose_list
    # roller_conf_list_without_static is use to generate trajectory
    roller_conf_list_without_static = roller_approach_conf_list + roller_holding_moveto_conf_list + roller_assembly_conf_list

    # lft arm
    # This is for digital twin.
    hole_conf_list = hole_conf_list \
                     + hole_static_conf_list \
                     + hole_depart_and_place_conf_list \
                     + lft_arm_reset_conf_list
    print("hole_static_conf_type", type(hole_static_conf_list[-1]))
    # lft hand
    hole_jawwidth_list = hole_jawwidth_list \
                         + hole_static_jawwidth_list \
                         + hole_depart_and_place_jawwidth_list \
                         + lft_hand_reset_jawwidth_list
    # hole pose
    hole_pose_list = hole_pose_list + hole_static_list + hole_depart_and_place_pose_list +hole_static_while_reset
    # this is for generate trajectory
    hole_conf_list_without_static = hole_approach_conf_list + hole_holding_moveto_conf_list


    print("the length of roller_conf_list", len(roller_conf_list))
    print("the length of hole_conf_list", len(hole_conf_list))

    print("the length of roller_jaw_list", len(roller_jawwidth_list))
    print("the length of hole_jaw_list", len(hole_jawwidth_list))

    print("the length of roller_pos_list", len(roller_pose_list))
    print("the length of hole_pos_list", len(hole_pose_list))


    # generate trajectory
    toggle_gen_traj = False
    if toggle_gen_traj:
        rgt_arm_interpolated_confs, \
        rgt_arm_interpolated_spds, \
        rgt_arm_interpolated_accs, \
        rgt_arm_interpolated_x = traj_gen.interpolate(path=roller_conf_list,
                                                      # path=roller_conf_list
                                                      control_frequency=0.005,
                                                      time_interval=0.5)
        print("the length of the rgt_arm_interpolated_confs ", len(rgt_arm_interpolated_confs))

        lft_arm_interpolated_confs, \
        lft_arm_interpolated_spds, \
        lft_arm_interpolated_accs, \
        lft_arm_interpolated_x = traj_gen.interpolate(path=hole_conf_list,
                                                      # path=hole_conf_list
                                                      control_frequency=0.005,
                                                      time_interval=0.5)
        print("the length of the lft_arm_interpolated_confs ", len(lft_arm_interpolated_confs))
        print("the type of the lft_arm_interpolated_confs", type(lft_arm_interpolated_confs))

        rgt_arm_conf_dataframe = pd.DataFrame(rgt_arm_interpolated_confs)
        lft_arm_conf_dataframe = pd.DataFrame(lft_arm_interpolated_confs)

        rgt_arm_conf_dataframe.to_excel('D:/wrs-main-new/wrs-main/app\demo/lft_arm_conf.xls')
        lft_arm_conf_dataframe.to_excel('D:/wrs-main-new/wrs-main/app\demo/lft_arm_conf.xls')


    robot_attached_list = []
    rgt_object_attached_list = []
    lft_object_attached_list = []
    counter = [0]


    def update_dual_robot(robot_s,
                          rgt_hand_name,
                          rgt_objcm,
                          rgt_robot_path,
                          rgt_jawwidth_path,
                          rgt_obj_path,
                          lft_hand_name,
                          lft_objcm,
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
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for rgt_object_attached in rgt_object_attached_list:
                rgt_object_attached.detach()
            for lft_object_attrach in lft_object_attached_list:
                lft_object_attrach.detach()
            robot_attached_list.clear()
            rgt_object_attached_list.clear()
            lft_object_attached_list.clear()
        # rgt arm update
        rgt_pose = rgt_robot_path[counter[0]]  # obtain the robot pose
        robot_s.fk(rgt_hand_name, rgt_pose)    # robot forward kinematics
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
        rgt_objb_copy = rgt_objcm.copy()
        rgt_objb_copy.set_homomat(rgt_obj_pose)
        rgt_objb_copy.attach_to(base)
        # lft hand object
        lft_obj_pose = lft_obj_path[counter[0]]
        lft_objb_copy = lft_objcm.copy()
        lft_objb_copy.set_homomat(lft_obj_pose)
        lft_objb_copy.attach_to(base)
        rgt_object_attached_list.append(rgt_objb_copy)
        lft_object_attached_list.append(lft_objb_copy)
        counter[0] += 1
        return task.again
#
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

    show_dual_arm_animation = True
    show_lft_arm_animation = False
    show_rgt_arm_animation = False
    if show_dual_arm_animation:
        taskMgr.doMethodLater(0.03, update_dual_robot, "update",
                              extraArgs=[u3ed,
                                         rgt_hand_name,
                                         roller,
                                         roller_conf_list,
                                         roller_jawwidth_list,
                                         roller_pose_list,
                                         lft_hand_name,
                                         hole,
                                         hole_conf_list,
                                         hole_jawwidth_list,
                                         hole_pose_list,
                                         robot_attached_list,
                                         rgt_object_attached_list,
                                         lft_object_attached_list,
                                         counter],
                              appendTask=True)

    if show_lft_arm_animation:
        taskMgr.doMethodLater(0.05, update_one_robot, "update",
                              extraArgs=[u3ed,
                                         lft_hand_name,
                                         hole,
                                         hole_conf_list,
                                         hole_jawwidth_list,
                                         hole_pose_list,
                                         robot_attached_list,
                                         lft_object_attached_list,
                                         counter],
                              appendTask=True)

    if show_rgt_arm_animation:
        taskMgr.doMethodLater(0.05, update_one_robot, "update",
                              extraArgs=[u3ed,
                                         rgt_hand_name,
                                         roller,
                                         roller_conf_list,
                                         roller_jawwidth_list,
                                         roller_pose_list,
                                         robot_attached_list,
                                         rgt_object_attached_list,
                                         counter],
                              appendTask=True)



    base.run()







