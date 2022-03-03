# -*- coding: utf-8-*-

from QPanda3D.Panda3DWorld import Panda3DWorld
from QPanda3D.QPanda3DWidget import QPanda3DWidget

# import PyQt5 stuff
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
from panda3d.core import *
from direct.interval.LerpInterval import LerpHprInterval
from direct.interval.IntervalGlobal import *
# from visualization.panda import world

from app.Widgets.robot_Widgets import show_arm_data
from PyQt5 import QtWidgets

import cv2
from app.Widgets.camera_Widgets.camThread import camThread

from robot_sim.robots.ur3e_dual.ur3e_dual import UR3EDual
from app.example.pyqt5_create_video_main import tstgui1

import robot_sim.robots.ur3e_dual.ur3e_dual as ur3ed
import modeling.dynamics.bullet.bdmodel as bdm
import modeling.geometric_model as gm
import modeling.collision_model as cm
import basis.robot_math as rm
import numpy as np
import robot_con.ur.ur3e_dual_x as Ur3edual
from threading import Timer

from manipulation.pick_place_planner import PickPlacePlanner
import grasping.annotation.utils as gutil


class RepeatingTimer(Timer):
   def run(self):
       while not self.finished.is_set():
           self.function(*self.args, **self.kwargs)
           self.finished.wait(self.interval)

def obtain_jointValue():
    global lft_value
    global rgt_value
    lft_value = robot_s.get_jnt_values("lft_arm")
    rgt_value = robot_s.get_jnt_values("rgt_arm")





# this time joints angle
class plotwindows(QtWidgets.QWidget):
    # # # real lef arm joints from left robot

    def __init__(self):
        super(plotwindows, self).__init__()
        # show left arm joints
        layout = QFormLayout()
        self.edita3 = QLineEdit()
        self.edita4 = QLineEdit()
        self.edita5 = QLineEdit()
        self.edita6 = QLineEdit()
        self.edita7 = QLineEdit()
        self.edita8 = QLineEdit()
        layout.addRow("lef joint1", self.edita3)
        layout.addRow("lef joint2", self.edita4)
        layout.addRow("lef joint3", self.edita5)
        layout.addRow("lef joint4", self.edita6)
        layout.addRow("lef joint5", self.edita7)
        layout.addRow("lef joint6", self.edita8)
#         #layout.setContentsMargins(10, 10, 500, 100)
        layout.setSpacing(10)
        self.setLayout(layout)
        self.Mytimer()
#
#
    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1)
#
    def update(self):
        # update lef arm
        self.edita3.setText(str(lft_value[0]))
        self.edita4.setText(str(lft_value[1]))
        self.edita5.setText(str(lft_value[2]))
        self.edita6.setText(str(lft_value[3]))
        self.edita7.setText(str(lft_value[4]))
        self.edita8.setText(str(lft_value[5]))
# # this time joints angle
class plotwindows_rgt_arm(QtWidgets.QWidget):
    # real rgt arm joints from right robot
    global Joint1_value_rgt
    global Joint2_value_rgt
    global Joint3_value_rgt
    global Joint4_value_rgt
    global Joint5_value_rgt
    global Joint6_value_rgt

    def __init__(self):
        super(plotwindows_rgt_arm, self).__init__()
        # show left arm joints
        layout = QFormLayout()
        self.edita3_rgt = QLineEdit()
        self.edita4_rgt = QLineEdit()
        self.edita5_rgt = QLineEdit()
        self.edita6_rgt = QLineEdit()
        self.edita7_rgt = QLineEdit()
        self.edita8_rgt = QLineEdit()
        layout.addRow("rgt joint1", self.edita3_rgt)
        layout.addRow("rgt joint2", self.edita4_rgt)
        layout.addRow("rgt joint3", self.edita5_rgt)
        layout.addRow("rgt joint4", self.edita6_rgt)
        layout.addRow("rgt joint5", self.edita7_rgt)
        layout.addRow("rgt joint6", self.edita8_rgt)
        #layout.setContentsMargins(10, 10, 500, 100)
        layout.setSpacing(10)
        self.setLayout(layout)
        self.Mytimer()


    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1)

#
    def update(self):
        # update lef arm
        self.edita3_rgt.setText(str(rgt_value[0]))
        self.edita4_rgt.setText(str(rgt_value[1]))
        self.edita5_rgt.setText(str(rgt_value[2]))
        self.edita6_rgt.setText(str(rgt_value[3]))
        self.edita7_rgt.setText(str(rgt_value[4]))
        self.edita8_rgt.setText(str(rgt_value[5]))

# # next time joints angle
class plotwindows_lef_arm_next(QtWidgets.QWidget):

    def __init__(self):
        super(plotwindows_lef_arm_next, self).__init__()
        # show left arm joints
        layout = QFormLayout()
        self.edita3 = QLineEdit()
        self.edita4 = QLineEdit()
        self.edita5 = QLineEdit()
        self.edita6 = QLineEdit()
        self.edita7 = QLineEdit()
        self.edita8 = QLineEdit()
        layout.addRow("lef joint1 next", self.edita3)
        layout.addRow("lef joint2 next", self.edita4)
        layout.addRow("lef joint3 next", self.edita5)
        layout.addRow("lef joint4 next", self.edita6)
        layout.addRow("lef joint5 next", self.edita7)
        layout.addRow("lef joint6 next", self.edita8)
        #layout.setContentsMargins(10, 10, 500, 100)
        layout.setSpacing(10)
        self.setLayout(layout)
        self.Mytimer()


    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1)
#
#
#     def update(self):
#         # update lef arm
#         self.edita3.setText(str(Joint1_value))
#         self.edita4.setText(str(Joint2_value))
#         self.edita5.setText(str(Joint3_value))
#         self.edita6.setText(str(Joint4_value))
#         self.edita7.setText(str(Joint5_value))
#         self.edita8.setText(str(Joint6_value))


# # next time joints angle
class plotwindows_rgt_arm_next(QtWidgets.QWidget):

    def __init__(self):
        super(plotwindows_rgt_arm_next, self).__init__()
        # show left arm joints
        layout = QFormLayout()
        self.edita3_rgt = QLineEdit()
        self.edita4_rgt = QLineEdit()
        self.edita5_rgt = QLineEdit()
        self.edita6_rgt = QLineEdit()
        self.edita7_rgt = QLineEdit()
        self.edita8_rgt = QLineEdit()
        layout.addRow("rgt joint1 next", self.edita3_rgt)
        layout.addRow("rgt joint2 next", self.edita4_rgt)
        layout.addRow("rgt joint3 next", self.edita5_rgt)
        layout.addRow("rgt joint4 next", self.edita6_rgt)
        layout.addRow("rgt joint5 next", self.edita7_rgt)
        layout.addRow("rgt joint6 next", self.edita8_rgt)
        #layout.setContentsMargins(10, 10, 500, 100)
        layout.setSpacing(10)
        self.setLayout(layout)
        self.Mytimer()


    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1)


#     def update(self):
#         # update lef arm
#         self.edita3_rgt.setText(str(Joint1_value_rgt))
#         self.edita4_rgt.setText(str(Joint2_value_rgt))
#         self.edita5_rgt.setText(str(Joint3_value_rgt))
#         self.edita6_rgt.setText(str(Joint4_value_rgt))
#         self.edita7_rgt.setText(str(Joint5_value_rgt))
#         self.edita8_rgt.setText(str(Joint6_value_rgt))
#
# # show gripper and sensor info
class plotwindows_gripper_and_sensor(QtWidgets.QWidget):
    def __init__(self):
        super(plotwindows_gripper_and_sensor, self).__init__()
        # show left arm joints
        layout = QFormLayout()
        self.edita_lef_gripper_force = QLineEdit()
        self.edita_lef_gripper_dist = QLineEdit()
        self.edita_rgt_gripper_force = QLineEdit()
        self.edita_rgt_gripper_dist = QLineEdit()
        # self.edita7_rgt = QLineEdit()
        # self.edita8_rgt = QLineEdit()
        layout.addRow("lef_gripper_force", self.edita_lef_gripper_force)
        layout.addRow("lef_gripper_dist", self.edita_lef_gripper_dist)
        layout.addRow("lef_gripper_force", self.edita_rgt_gripper_force)
        layout.addRow("lef_gripper_dist", self.edita_rgt_gripper_dist)
        layout.setSpacing(10)
        self.setLayout(layout)
        self.Mytimer()

    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1)

#     def update(self):
#         # update lef arm
#         self.edita_lef_gripper_force.setText(str(lef_gripper_force))
#         self.edita_lef_gripper_dist.setText(str(lef_gripper_dist))
#         self.edita_rgt_gripper_force.setText(str(rgt_gripper_force))
#         self.edita_rgt_gripper_dist.setText(str(rgt_gripper_dist))




if __name__ == "__main__":

    base = Panda3DWorld(cam_pos=[10, 0, 5], lookat_pos=[0, 0, 1])
    # here we can add some robot model to display the digital twin
    # ------------------------------------------------------------------

    # show our robots with platform in app
    # ----------- written by ZiQi ----------
    gm.gen_frame().attach_to(base)  # add frame in base world
    # u3ed = UR3EDual()               # dual arm class instantiation
    # u3ed.gen_stickmodel().attach_to(base)  # add robot's stick model
    # # add 3 dimension of robot in app
    # u3ed_meshmodel = u3ed.gen_meshmodel(toggle_tcpcs=True)
    # u3ed_meshmodel.attach_to(base)
    # add sphere in app
    # gm.gen_sphere([0.5, 0.2, 0.8], 0.02).attach_to(base)
    robot_s = UR3EDual(enable_cc=True)
    u3ed_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)

    # use a timer to obtain joints angle
    timer = RepeatingTimer(0.01, obtain_jointValue)
    timer.start()

    # u3ed_meshmodel.attach_to(base)
    # define the planner
    pp_planner = PickPlacePlanner(robot_s)
    manipulator_name = 'lft_arm'
    hand_name = 'lft_arm'
    # predefine the goal_homo_list and grasp_info_list
    goal_homomat_list = []
    grasp_info_list = []
    # set up conf_info, start_goal_info, end_goal_info
    # conf_info
    start_conf = robot_s.get_jnt_values(manipulator_name)
    # start_goal_info
    start_goal_pos = np.array([0.5, 0.2, 0.9])
    start_goal_rotmat = np.eye(3)
    start_goal_homomat = rm.homomat_from_posrot(start_goal_pos, start_goal_rotmat)
    goal_homomat_list.append(start_goal_homomat)
    # end_goal_info
    end_goal_pos = np.array([0.7, 0.2, 1.0])
    end_goal_rotmat = np.eye(3)
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

    # solve the conf_list, jawwidth_list, objpose_list
    conf_list, jawwidth_list, objpose_list = \
        pp_planner.gen_pick_and_place_motion(hnd_name=hand_name,
                                             objcm=objcm,
                                             grasp_info_list=grasp_info_list,
                                             # goal_homomat_list=goal_homomat_list,
                                             goal_homomat_list=[start_goal_homomat, end_goal_homomat],
                                             start_conf=start_conf,
                                             end_conf=start_conf,
                                             depart_direction_list=[np.array([0, 0, 1])] * len(goal_homomat_list),
                                             approach_direction_list=[np.array([0, 0, -1])] * len(goal_homomat_list),
                                             # depart_distance_list=[None] * len(goal_homomat_list),
                                             # approach_distance_list=[None] * len(goal_homomat_list),
                                             depart_distance_list=[.2] * len(goal_homomat_list),
                                             approach_distance_list=[.2] * len(goal_homomat_list),
                                             approach_jawwidth=None,
                                             depart_jawwidth=None,
                                             ad_granularity=.003,
                                             use_rrt=True,
                                             obstacle_list=[],
                                             use_incremental=False)
    # print("the conf list are", conf_list)
    # print("the jaw width list are", jawwidth_list)
    # print("the obj pos list are ", objpose_list)

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

    # base.run()



    # ------------------------------------------

    # connect the real Ur3E robot
    # ---------- written by ZiQi  ---------------
    # Ur3edualx = Ur3edual.Ur3EDualUrx(lft_robot_ip='', rgt_robot_ip='', pc_ip='')  # this ip address should be add
    # # obtain the real robot angle
    # global lft_arm_value
    # global rgt_arm_value
    # lft_arm_value = Ur3edualx.get_jnt_values("lft_arm")
    # rgt_arm_value = Ur3edualx.get_jnt_values("rgt_arm")




    # ----------------------------------------

    # ----------The screen is displayed below----------------

    app = QApplication(sys.argv)
    appw = QMainWindow()
    appw.setWindowTitle("SZTU Robotics Laboratory")
    appw.setGeometry(0, 0, 1680, 1050)
    main_widget = QWidget()
    main_widget.setLayout(QGridLayout())
    # SPanda3D Widget
    pandaWidget = QPanda3DWidget(base)
    pandaWidget.setMaximumSize(1024, 768)

    # camera Widget
    # cam_widget = tstgui1.VideoBox()

    # cam_widget.move(1024, 768)
    # cam_widget.start()
    # cam_widget.setGeometry(800, 0, 600, 400)

    # robot Widget

    # show this time joints angle
    #rob_widget_lef = show_arm_data.plotwindows()
    # # rob_widget_lef = plotwindows()
    # rob_widget_rgt = show_arm_data.plotwindows_rgt_arm()
    # # show next time joints angle
    # rob_widget_lef_next = show_arm_data.plotwindows_lef_arm_next()
    # rob_widget_rgt_next = show_arm_data.plotwindows_rgt_arm_next()
    # # gripper info
    # gripper_widget = show_arm_data.plotwindows_gripper_and_sensor()

    # # show real joints data from robot
    rob_widget_lef = plotwindows()
    rob_widget_rgt = plotwindows_rgt_arm()
    rob_widget_lef_next = plotwindows_lef_arm_next()
    rob_widget_rgt_next = plotwindows_rgt_arm_next()
    gripper_widget = plotwindows_gripper_and_sensor()




    # Add them to the window
    main_widget.layout().addWidget(pandaWidget, 0, 0, 3, 3)
    # main_widget.layout().addWidget(btn_widget)
    # main_widget.layout().addWidget(cam_widget, 0, 4, 3, 4)
    main_widget.layout().addWidget(rob_widget_lef, 4, 0)
    main_widget.layout().addWidget(rob_widget_rgt, 4, 1)
    main_widget.layout().addWidget(rob_widget_lef_next, 4, 2)
    main_widget.layout().addWidget(rob_widget_rgt_next, 4, 3)
    main_widget.layout().addWidget(gripper_widget, 4, 4)


    appw.setCentralWidget(main_widget)
    appw.show()
    sys.exit(app.exec_())



