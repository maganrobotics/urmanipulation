import os
import math
import numpy as np
import basis.robot_math as rm
import modeling.model_collection as mc
import modeling.collision_model as cm
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.ur3e.ur3e as ur
import robot_sim.end_effectors.grippers.robotiqhe.robotiqhe as rtq
from panda3d.core import CollisionNode, CollisionBox, Point3
import robot_sim.robots.robot_interface as ri
import modeling.dynamics.bullet.bdmodel as bdm


class UR3EDual(ri.RobotInterface):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name='ur3edual', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        # left side
        self.lft_body = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=np.zeros(0), name='lft_body_jl')
        self.lft_body.jnts[1]['loc_pos'] = np.array([.365, .345, 1.33])  # define the robot pos relative to base frame
        self.lft_body.jnts[1]['loc_rotmat'] = rm.rotmat_from_euler(math.pi / 2.0, 0,
                                                                   math.pi / 2.0)  # left from robot_s view
        self.lft_body.lnks[0]['name'] = "ur3e_dual_base"  # my_ur3e_dual_base.stl

        self.lft_body.lnks[0]['loc_pos'] = np.array([0, 0, 0])  # define the base pos
        self.lft_body.lnks[0]['collisionmodel'] = cm.CollisionModel(
            os.path.join(this_dir, "meshes", "new_base.stl"),
            cdprimit_type="user_defined", expand_radius=.005,
            userdefined_cdprimitive_fn=self._base_combined_cdnp)
        self.lft_body.lnks[0]['rgba'] = [.3, .3, .3, 1.0]  # define the color of the base
        self.lft_body.reinitialize()

        # define the robot joints angle, the sixth joint angle is zero
        lft_arm_homeconf = np.zeros(6)
        lft_arm_homeconf[0] = -math.pi * 2.0 / 3.0
        lft_arm_homeconf[1] = -math.pi * 2.0 / 3.0
        lft_arm_homeconf[2] = math.pi / 2.0
        lft_arm_homeconf[3] = math.pi
        lft_arm_homeconf[4] = -math.pi / 2.0
        # instance the robot, parameters are from above
        self.lft_arm = ur.UR3E(pos=self.lft_body.jnts[-1]['gl_posq'],
                               rotmat=self.lft_body.jnts[-1]['gl_rotmatq'],
                               homeconf=lft_arm_homeconf,
                               enable_cc=False)
        # lft hand offset (if needed)
        self.lft_hnd_offset = np.zeros(3)
        lft_hnd_pos, lft_hnd_rotmat = self.lft_arm.cvt_loc_tcp_to_gl(loc_pos=self.lft_hnd_offset)    # the lft_hnd_pos and the lft_hnd_rotmat are relative to base frame
        print("the lft_hnd_pos", lft_hnd_pos )
        self.lft_hnd = rtq.RobotiqHE(pos=lft_hnd_pos,
                                     rotmat=self.lft_arm.jnts[-1]['gl_rotmatq'],
                                     enable_cc=False)
        # rigth side
        self.rgt_body = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=np.zeros(0), name='rgt_body_jl')
        self.rgt_body.jnts[1]['loc_pos'] = np.array([.365, -.345, 1.33])  # right from robot_s view
        self.rgt_body.jnts[1]['loc_rotmat'] = rm.rotmat_from_euler(math.pi / 2.0, 0,
                                                                   math.pi / 2.0)  # left from robot_s view
        self.rgt_body.lnks[0]['name'] = "ur3e_dual_base"
        self.rgt_body.lnks[0]['loc_pos'] = np.array([0, 0, 0])
        self.rgt_body.lnks[0]['meshfile'] = None
        self.rgt_body.lnks[0]['rgba'] = [.3, .3, .3, 1.0]
        self.rgt_body.reinitialize()
        rgt_arm_homeconf = np.zeros(6)
        rgt_arm_homeconf[0] = math.pi * 2.0 / 3.0
        rgt_arm_homeconf[1] = -math.pi / 3.0
        rgt_arm_homeconf[2] = -math.pi / 2.0
        rgt_arm_homeconf[4] = math.pi / 2.0
        self.rgt_arm = ur.UR3E(pos=self.rgt_body.jnts[-1]['gl_posq'],
                               rotmat=self.rgt_body.jnts[-1]['gl_rotmatq'],
                               homeconf=rgt_arm_homeconf,
                               enable_cc=False)
        # rgt hand offset (if needed)
        self.rgt_hnd_offset = np.zeros(3)
        rgt_hnd_pos, rgt_hnd_rotmat = self.rgt_arm.cvt_loc_tcp_to_gl(loc_pos=self.rgt_hnd_offset)
        print("the rgt hnd pos", rgt_hnd_pos)
        # TODO replace using copy
        self.rgt_hnd = rtq.RobotiqHE(pos=rgt_hnd_pos,
                                     rotmat=self.rgt_arm.jnts[-1]['gl_rotmatq'],
                                     enable_cc=False)
        # tool center point
        # lft
        self.lft_arm.tcp_jntid = -1
        self.lft_arm.tcp_loc_pos = self.lft_hnd.jaw_center_pos
        self.lft_arm.tcp_loc_rotmat = self.lft_hnd.jaw_center_rotmat
        # rgt
        self.rgt_arm.tcp_jntid = -1
        self.rgt_arm.tcp_loc_pos = self.lft_hnd.jaw_center_pos
        self.rgt_arm.tcp_loc_rotmat = self.lft_hnd.jaw_center_rotmat
        # a list of detailed information about objects in hand, see CollisionChecker.add_objinhnd
        self.lft_oih_infos = []
        self.rgt_oih_infos = []
        # collision detection
        if enable_cc:
            self.enable_cc()
        # component map
        self.manipulator_dict['rgt_arm'] = self.rgt_arm
        self.manipulator_dict['lft_arm'] = self.lft_arm
        self.manipulator_dict['rgt_hnd'] = self.rgt_arm  # specify which hand is a gripper installed to
        self.manipulator_dict['lft_hnd'] = self.lft_arm  # specify which hand is a gripper installed to
        self.hnd_dict['rgt_hnd'] = self.rgt_hnd
        self.hnd_dict['lft_hnd'] = self.lft_hnd
        self.hnd_dict['rgt_arm'] = self.rgt_hnd
        self.hnd_dict['lft_arm'] = self.lft_hnd

    @staticmethod
    def _base_combined_cdnp(name, radius):
        collision_node = CollisionNode(name)
        collision_primitive_c0 = CollisionBox(Point3(0.54, 0.0, 0.39),
                                              x=.54 + radius, y=.6 + radius, z=.39 + radius)
        collision_node.addSolid(collision_primitive_c0)
        collision_primitive_c1 = CollisionBox(Point3(0.06, 0.0, 0.9),
                                              x=.06 + radius, y=.375 + radius, z=.9 + radius)
        collision_node.addSolid(collision_primitive_c1)
        collision_primitive_c2 = CollisionBox(Point3(0.18, 0.0, 1.77),
                                              x=.18 + radius, y=.21 + radius, z=.03 + radius)
        collision_node.addSolid(collision_primitive_c2)
        collision_primitive_l0 = CollisionBox(Point3(0.2425, 0.345, 1.33),
                                              x=.1225 + radius, y=.06 + radius, z=.06 + radius)
        collision_node.addSolid(collision_primitive_l0)
        collision_primitive_r0 = CollisionBox(Point3(0.2425, -0.345, 1.33),
                                              x=.1225 + radius, y=.06 + radius, z=.06 + radius)
        collision_node.addSolid(collision_primitive_r0)
        collision_primitive_l1 = CollisionBox(Point3(0.21, 0.405, 1.07),
                                              x=.03 + radius, y=.06 + radius, z=.29 + radius)
        collision_node.addSolid(collision_primitive_l1)
        collision_primitive_r1 = CollisionBox(Point3(0.21, -0.405, 1.07),
                                              x=.03 + radius, y=.06 + radius, z=.29 + radius)
        collision_node.addSolid(collision_primitive_r1)
        return collision_node
    # It is added by Ziqi
    # -------------------------------
    def enable_cc(self):
        super().enable_cc()
        #raise NotImplementedError
        # this is so important
        # print("the info about lft_arm", self.lft_arm.lnks[2]['cdprimit_childid'] )
        self.cc.add_cdlnks(self.lft_body, [0])  # add base
        self.cc.add_cdlnks(self.lft_arm, [1, 2, 3, 4, 5, 6])  # add left arm
        self.cc.add_cdlnks(self.lft_hnd.lft, [0, 1])  # add left hand's base and left finger
        self.cc.add_cdlnks(self.lft_hnd.rgt, [1])  # add right finger
        self.cc.add_cdlnks(self.rgt_arm, [1, 2, 3, 4, 5, 6])  # add  right arm
        self.cc.add_cdlnks(self.rgt_hnd.lft, [0, 1])  # add right hand's base and its left finger
        self.cc.add_cdlnks(self.rgt_hnd.rgt, [1])   # add right hand's right finger
        activelist = [self.lft_arm.lnks[1],
                      self.lft_body.lnks[0],
                      self.lft_arm.lnks[2],
                      self.lft_arm.lnks[3],
                      self.lft_arm.lnks[4],
                      self.lft_arm.lnks[5],
                      self.lft_arm.lnks[6],
                      self.lft_hnd.lft.lnks[0],
                      self.lft_hnd.lft.lnks[1],
                      self.lft_hnd.rgt.lnks[1],
                      self.rgt_arm.lnks[1],
                      self.rgt_arm.lnks[2],
                      self.rgt_arm.lnks[3],
                      self.rgt_arm.lnks[4],
                      self.rgt_arm.lnks[5],
                      self.rgt_arm.lnks[6],
                      self.rgt_hnd.lft.lnks[0],
                      self.rgt_hnd.lft.lnks[1],
                      self.rgt_hnd.rgt.lnks[1]]
        self.cc.set_active_cdlnks(activelist)
        # set up self collision check
        # set up collision check between body and arm/hand
        # fromlist = [self.lft_body.lnks[0]]
        # #             self.lft_arm.lnks[1],
        # #             self.lft_arm.lnks[2],
        # #             self.rgt_arm.lnks[1],
        # #             self.rgt_arm.lnks[2]]
        # # intolist = [self.lft_arm.lnks[3],
        # #             self.lft_arm.lnks[4],
        # #             self.lft_arm.lnks[5],
        # #             self.lft_arm.lnks[6],
        # #             self.lft_hnd.lft.lnks[0],
        # #             self.lft_hnd.lft.lnks[1],
        # #             self.lft_hnd.rgt.lnks[1],
        # #             self.rgt_arm.lnks[3],
        # #             self.rgt_arm.lnks[4],
        # #             self.rgt_arm.lnks[5],
        # #             self.rgt_arm.lnks[6],
        # intolist = [self.rgt_hnd.lft.lnks[0],
        #             self.rgt_hnd.lft.lnks[1],
        #             self.rgt_hnd.rgt.lnks[1],
        #             self.lft_hnd.lft.lnks[0],
        #             self.lft_hnd.lft.lnks[1],
        #             self.lft_hnd.rgt.lnks[1]]
        # self.cc.set_cdpair(fromlist, intolist)
        # fromlist = [self.rgt_hnd.lft.lnks[0],
        #             self.rgt_hnd.lft.lnks[1],
        #             self.rgt_hnd.rgt.lnks[1]]
        # intolist = [self.rgt_arm.lnks[4],
        #             self.rgt_arm.lnks[5]]
        # self.cc.set_cdpair(fromlist, intolist)
        # fromlist = [self.lft_hnd.lft.lnks[0],
        #             self.lft_hnd.lft.lnks[1],
        #             self.lft_hnd.rgt.lnks[1]]
        # intolist = [self.lft_arm.lnks[5],
        #             self.lft_arm.lnks[4]]
        # self.cc.set_cdpair(fromlist, intolist)
        # #  set up arm-body collision detection --extra
        # fromlist = [self.lft_body.lnks[0]]
        # intolist = [self.lft_arm.lnks[2],
        #             self.rgt_arm.lnks[2]]
        # self.cc.set_cdpair(fromlist, intolist)
        # #  set up arm-arm collision check
        # fromlist = [self.lft_arm.lnks[3],
        #             self.lft_arm.lnks[4],
        #             self.lft_arm.lnks[5],
        #             self.lft_arm.lnks[6],
        #             self.lft_hnd.lft.lnks[0],
        #             self.lft_hnd.lft.lnks[1],
        #             self.lft_hnd.rgt.lnks[1]]
        # intolist = [self.rgt_arm.lnks[3],
        #             self.rgt_arm.lnks[4],
        #             self.rgt_arm.lnks[5],
        #             self.rgt_arm.lnks[6],
        #             self.rgt_hnd.lft.lnks[0],
        #             self.rgt_hnd.lft.lnks[1],
        #             self.rgt_hnd.rgt.lnks[1]]
        # self.cc.set_cdpair(fromlist, intolist)
        # fromlist = [self.lft_hnd.lft.lnks[0],
        #             self.lft_arm.lnks[6]]
        # intolist = [self.lft_arm.lnks[2]]
        # self.cc.set_cdpair(fromlist, intolist)
        # lnks used for arm-body collision detection
        fromlist = [self.lft_body.lnks[0],
                    self.lft_arm.lnks[1],
                    self.rgt_arm.lnks[1]]
        intolist = [self.lft_arm.lnks[3],
                    self.lft_arm.lnks[4],
                    self.lft_arm.lnks[5],
                    self.lft_arm.lnks[6],
                    self.lft_hnd.lft.lnks[0],
                    self.lft_hnd.lft.lnks[1],
                    self.lft_hnd.rgt.lnks[1],
                    self.rgt_arm.lnks[3],
                    self.rgt_arm.lnks[4],
                    self.rgt_arm.lnks[5],
                    self.rgt_arm.lnks[6],
                    self.rgt_hnd.lft.lnks[0],
                    self.rgt_hnd.lft.lnks[1],
                    self.rgt_hnd.rgt.lnks[1]]
        self.cc.set_cdpair(fromlist, intolist)
        # lnks used for arm-body collision detection -- extra
        fromlist = [self.lft_body.lnks[0]]  # body
        intolist = [self.lft_arm.lnks[2],
                    self.rgt_arm.lnks[2]]
        self.cc.set_cdpair(fromlist, intolist)
        # lnks used for in-arm collision detection
        fromlist = [self.lft_arm.lnks[2]]
        intolist = [self.lft_arm.lnks[4],
                    self.lft_arm.lnks[5],
                    self.lft_hnd.lft.lnks[0],
                    self.lft_hnd.lft.lnks[1],
                    self.lft_hnd.rgt.lnks[1]]
        self.cc.set_cdpair(fromlist, intolist)
        fromlist = [self.rgt_arm.lnks[2]]
        intolist = [self.rgt_arm.lnks[4],
                    self.rgt_arm.lnks[5],
                    self.rgt_hnd.lft.lnks[0],
                    self.rgt_hnd.lft.lnks[1],
                    self.rgt_hnd.rgt.lnks[1]]
        self.cc.set_cdpair(fromlist, intolist)
        # arm-arm collision
        fromlist = [self.lft_arm.lnks[3],
                    self.lft_arm.lnks[4],
                    self.lft_arm.lnks[5],
                    self.lft_arm.lnks[6],
                    self.lft_hnd.lft.lnks[0],
                    self.lft_hnd.lft.lnks[1],
                    self.lft_hnd.rgt.lnks[1]]
        intolist = [self.rgt_arm.lnks[3],
                    self.rgt_arm.lnks[4],
                    self.rgt_arm.lnks[5],
                    self.rgt_arm.lnks[6],
                    self.rgt_hnd.lft.lnks[0],
                    self.rgt_hnd.lft.lnks[1],
                    self.rgt_hnd.rgt.lnks[1]]
        self.cc.set_cdpair(fromlist, intolist)


    def release(self, hnd_name, objcm, jaw_width=None):
        """
        the objcm is added as a part of the robot_s to the cd checker
        :param jaw_width:
        :param objcm:
        :param hnd_name:

        :return:
        """
        if hnd_name == 'lft_hnd' or hnd_name == 'lft_arm':
            oih_infos = self.lft_oih_infos
        elif hnd_name == 'rgt_hnd'or hnd_name == 'rgt_arm':
            oih_infos = self.rgt_oih_infos
        else:
            raise ValueError("hnd_name must be lft_hnd or rgt_hnd!")
        if jaw_width is not None:
            self.jaw_to(hnd_name, jaw_width)
        for obj_info in oih_infos:
            if obj_info['collisionmodel'] is objcm:
                self.cc.delete_cdobj(obj_info)
                oih_infos.remove(obj_info)
                break

    def release_all(self, jaw_width=None, hnd_name='lft_hnd'):
        """
        release all objects from the specified hand
        :param jaw_width:
        :param hnd_name:
        :return:

        """
        if hnd_name == 'lft_hnd':
            oih_infos = self.lft_oih_infos
        elif hnd_name == 'rgt_hnd':
            oih_infos = self.rgt_oih_infos
        else:
            raise ValueError("hnd_name must be lft_hnd or rgt_hnd!")
        if jaw_width is not None:
            self.jaw_to(hnd_name, jaw_width)
        for obj_info in oih_infos:
            self.cc.delete_cdobj(obj_info)
        oih_infos.clear()


    # ------------------------------------------------------------


    def move_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.lft_body.fix_to(self.pos, self.rotmat)
        self.lft_arm.fix_to(pos=self.lft_body.jnts[-1]['gl_posq'], rotmat=self.lft_body.jnts[-1]['gl_rotmatq'])
        lft_hnd_pos, lft_hnd_rotmat = self.lft_arm.get_worldpose(relpos=self.rgt_hnd_offset)
        self.lft_hnd.fix_to(pos=lft_hnd_pos, rotmat=lft_hnd_rotmat)
        self.rgt_body.fix_to(self.pos, self.rotmat)
        self.rgt_arm.fix_to(pos=self.rgt_body.jnts[-1]['gl_posq'], rotmat=self.rgt_body.jnts[-1]['gl_rotmatq'])
        rgt_hnd_pos, rgt_hnd_rotmat = self.rgt_arm.get_worldpose(relpos=self.rgt_hnd_offset)
        self.rgt_hnd.fix_to(pos=rgt_hnd_pos, rotmat=rgt_hnd_rotmat)

    def get_hnd_on_manipulator(self, manipulator_name):
        if manipulator_name == 'rgt_arm':
            return self.rgt_hnd
        elif manipulator_name == 'lft_arm':
            return self.lft_hnd
        else:
            raise ValueError("The given jlc does not have a hand!")

    def fk(self, component_name, jnt_values):
        """
        :param jnt_values: 1x6 or 1x12 nparray
        :hnd_name 'lft_arm', 'rgt_arm', 'both_arm'
        :param component_name:
        :return:

        """

        def update_oih(component_name='rgt_arm'):
            # inline function for update objects in hand
            if component_name == 'rgt_arm':
                oih_info_list = self.rgt_oih_infos
            elif component_name == 'lft_arm':
                oih_info_list = self.lft_oih_infos
            for obj_info in oih_info_list:
                gl_pos, gl_rotmat = self.cvt_loc_tcp_to_gl(component_name, obj_info['rel_pos'], obj_info['rel_rotmat'])
                obj_info['gl_pos'] = gl_pos
                obj_info['gl_rotmat'] = gl_rotmat

        def update_component(component_name, jnt_values):
            self.manipulator_dict[component_name].fk(jnt_values=jnt_values)
            self.get_hnd_on_manipulator(component_name).fix_to(
                pos=self.manipulator_dict[component_name].jnts[-1]['gl_posq'],
                rotmat=self.manipulator_dict[component_name].jnts[-1]['gl_rotmatq'])
            update_oih(component_name=component_name)

        super().fk(component_name, jnt_values)
        # examine length
        if component_name == 'lft_arm' or component_name == 'rgt_arm':
            if not isinstance(jnt_values, np.ndarray) or jnt_values.size != 6:
                raise ValueError("An 1x6 npdarray must be specified to move a single arm!")
            update_component(component_name, jnt_values)
        elif component_name == 'both_arm':
            if (jnt_values.size != 12):
                raise ValueError("A 1x12 npdarrays must be specified to move both arm!")
            update_component('lft_arm', jnt_values[0:6])
            update_component('rgt_arm', jnt_values[6:12])
        elif component_name == 'all':
            raise NotImplementedError
        else:
            raise ValueError("The given component name is not available!")

    def rand_conf(self, component_name):
        """
        override robot_interface.rand_conf
        :param component_name:
        :return:

        """
        if component_name == 'lft_arm' or component_name == 'rgt_arm':
            return super().rand_conf(component_name)
        elif component_name == 'both_arm':
            return np.hstack((super().rand_conf('lft_arm'), super().rand_conf('rgt_arm')))
        else:
            raise NotImplementedError

    def gen_stickmodel(self,
                       tcp_jntid=None,
                       tcp_loc_pos=None,
                       tcp_loc_rotmat=None,
                       toggle_tcpcs=False,
                       toggle_jntscs=False,
                       toggle_connjnt=False,
                       name='ur3e_dual_stickmodel'):
        stickmodel = mc.ModelCollection(name=name)
        self.lft_body.gen_stickmodel(tcp_loc_pos=None,
                                     tcp_loc_rotmat=None,
                                     toggle_tcpcs=False,
                                     toggle_jntscs=toggle_jntscs).attach_to(stickmodel)
        self.lft_arm.gen_stickmodel(tcp_jntid=tcp_jntid,
                                    tcp_loc_pos=tcp_loc_pos,
                                    tcp_loc_rotmat=tcp_loc_rotmat,
                                    toggle_tcpcs=toggle_tcpcs,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.lft_hnd.gen_stickmodel(toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.rgt_body.gen_stickmodel(tcp_loc_pos=None,
                                     tcp_loc_rotmat=None,
                                     toggle_tcpcs=False,
                                     toggle_jntscs=toggle_jntscs).attach_to(stickmodel)
        self.rgt_arm.gen_stickmodel(tcp_jntid=tcp_jntid,
                                    tcp_loc_pos=tcp_loc_pos,
                                    tcp_loc_rotmat=tcp_loc_rotmat,
                                    toggle_tcpcs=toggle_tcpcs,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        self.rgt_hnd.gen_stickmodel(toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    toggle_connjnt=toggle_connjnt).attach_to(stickmodel)
        return stickmodel

    def gen_meshmodel(self,
                      tcp_jntid=None,
                      tcp_loc_pos=None,
                      tcp_loc_rotmat=None,
                      toggle_tcpcs=False,
                      toggle_jntscs=False,
                      rgba=None,
                      name='ur3e_dual_meshmodel'):
        mm_collection = mc.ModelCollection(name=name)
        self.lft_body.gen_meshmodel(tcp_loc_pos=None,
                                    tcp_loc_rotmat=None,
                                    toggle_tcpcs=False,
                                    toggle_jntscs=toggle_jntscs,
                                    rgba=rgba).attach_to(mm_collection)
        self.lft_arm.gen_meshmodel(tcp_jntid=tcp_jntid,
                                   tcp_loc_pos=tcp_loc_pos,
                                   tcp_loc_rotmat=tcp_loc_rotmat,
                                   toggle_tcpcs=toggle_tcpcs,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(mm_collection)
        self.lft_hnd.gen_meshmodel(toggle_tcpcs=False,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(mm_collection)
        self.rgt_arm.gen_meshmodel(tcp_jntid=tcp_jntid,
                                   tcp_loc_pos=tcp_loc_pos,
                                   tcp_loc_rotmat=tcp_loc_rotmat,
                                   toggle_tcpcs=toggle_tcpcs,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(mm_collection)
        self.rgt_hnd.gen_meshmodel(toggle_tcpcs=False,
                                   toggle_jntscs=toggle_jntscs,
                                   rgba=rgba).attach_to(mm_collection)
        for obj_info in self.lft_oih_infos:
            objcm = obj_info['collisionmodel']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            objcm.copy().attach_to(mm_collection)
        for obj_info in self.rgt_oih_infos:
            objcm = obj_info['collisionmodel']
            objcm.set_pos(obj_info['gl_pos'])
            objcm.set_rotmat(obj_info['gl_rotmat'])
            objcm.copy().attach_to(mm_collection)
        return mm_collection

    # this is written by Ziqi
    def hold(self, hnd_name, objcm, jaw_width=None):
        """
        the objcm is added as a part of the robot_s to the cd checker
        :param jaw_width:
        :param objcm:
        :return:
        """
        if hnd_name == 'lft_arm' or hnd_name == 'lft_hnd':
            rel_pos, rel_rotmat = self.lft_arm.cvt_gl_to_loc_tcp(objcm.get_pos(), objcm.get_rotmat())
            intolist = [self.lft_body.lnks[0],
                        # self.lft_body.lnks[1],
                        self.lft_arm.lnks[1],
                        self.lft_arm.lnks[2],
                        self.lft_arm.lnks[3],
                        self.lft_arm.lnks[4],
                        self.rgt_arm.lnks[1],
                        self.rgt_arm.lnks[2],
                        self.rgt_arm.lnks[3],
                        self.rgt_arm.lnks[4],
                        self.rgt_arm.lnks[5],
                        self.rgt_arm.lnks[6],
                        self.rgt_hnd.lft.lnks[0],
                        self.rgt_hnd.lft.lnks[1],
                        self.rgt_hnd.rgt.lnks[1]]
            self.lft_oih_infos.append(self.cc.add_cdobj(objcm, rel_pos, rel_rotmat, intolist))
        elif hnd_name == 'rgt_hnd'or hnd_name == 'rgt_arm':
            rel_pos, rel_rotmat = self.rgt_arm.cvt_gl_to_loc_tcp(objcm.get_pos(), objcm.get_rotmat())
            intolist = [self.lft_body.lnks[0],
                        # self.lft_body.lnks[1],
                        self.rgt_arm.lnks[1],
                        self.rgt_arm.lnks[2],
                        self.rgt_arm.lnks[3],
                        self.rgt_arm.lnks[4],
                        self.lft_arm.lnks[1],
                        self.lft_arm.lnks[2],
                        self.lft_arm.lnks[3],
                        self.lft_arm.lnks[4],
                        self.lft_arm.lnks[5],
                        self.lft_arm.lnks[6],
                        self.lft_hnd.lft.lnks[0],
                        self.lft_hnd.lft.lnks[1],
                        self.lft_hnd.rgt.lnks[1]]
            self.rgt_oih_infos.append(self.cc.add_cdobj(objcm, rel_pos, rel_rotmat, intolist))
        else:
            raise ValueError("hnd_name must be lft_hnd or rgt_hnd!")
        if jaw_width is not None:
            self.jaw_to(hnd_name, jaw_width)
        return rel_pos, rel_rotmat


if __name__ == '__main__':
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import basis


    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    u3ed = UR3EDual()

    # this  is written by Ziqi Xu
    # --------------------------------
    u3ed.gen_stickmodel().attach_to(base)
    u3ed_meshmodel = u3ed.gen_meshmodel(toggle_tcpcs=True)
    u3ed_meshmodel.attach_to(base)
    # u3ed_meshmodel.show_cdprimit()
    red_ball = gm.gen_sphere([0.5, 0.2, 0.8], 0.02)
    red_ball.attach_to(base)
    red_ball.show_localframe()

    # add roller model
    objpath = os.path.join(basis.__path__[0], 'objects', 'roller.stl')
    roller = cm.CollisionModel(objpath, cdprimit_type='polygons')
    roller.set_rgba([0.7, 0.7, 0.7, 1.0])

    # # bunnycm.show_localframe()
    # roller.attach_to(base)
    # roller.set_pos([0.7, -0.3, 0.8])
    # roller.show_localframe()
    #
    # # add hole model
    objpath1 = os.path.join(basis.__path__[0], 'objects', 'hole.stl')
    hole = cm.CollisionModel(objpath1, cdprimit_type='polygons')
    hole.set_rgba([0.7, 0, 0.7, 0.6])
    # hole.attach_to(base)
    # hole.set_pos([0.7, 0.3, 0.8])
    # hole.show_localframe()
    # home conf
    rgt_arm_home_pose = u3ed.get_gl_tcp(manipulator_name='rgt_arm')
    rgt_arm_home_pos = rgt_arm_home_pose[0]
    rgt_arm_home_rotmat = rgt_arm_home_pose[1]
    print("the rgt_arm home pos", rgt_arm_home_pos)
    # define lft arm home pose
    lft_arm_home_pose = u3ed.get_gl_tcp(manipulator_name='lft_arm')
    lft_arm_home_pos = lft_arm_home_pose[0]
    lft_arm_home_rotmat = lft_arm_home_pose[1]
    print("lft_arm_home pos",lft_arm_home_pos)
    solve = u3ed.lft_arm.ik(tgt_pos=lft_arm_home_pos, tgt_rotmat=lft_arm_home_rotmat)
    print("the ik solve are", solve)
    # num = 89
    # num2 =96
    # for num1 in range(num2-num):
    #     print("i am here")
    # u3ed.get_jawwidth(hand_name="rgt_arm")


    # u3ed.enable_cc()



    # -------------------------------

    # # u3ed.fk(.85)
    # u3ed_meshmodel = u3ed.gen_meshmodel(toggle_tcpcs=True)
    # u3ed_meshmodel.attach_to(base)
    # # u3ed_meshmodel.show_cdprimit()
    # u3ed.gen_stickmodel().attach_to(base)
    base.run()
