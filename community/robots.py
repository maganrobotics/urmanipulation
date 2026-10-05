"""Adapters around the original WRS models; no replacement IK or RRT algorithm."""
import numpy as np
from robot_sim.manipulators.ur3e.ur3e import UR3E
from robot_sim.robots.ur3e_dual.ur3e_dual import UR3EDual


class SingleArm:
    """Expose the component-name protocol expected by WRS RRT-Connect."""
    def __init__(self, arm=None):
        self.arm = arm if arm is not None else UR3E(enable_cc=False)
        if self.arm.cc is None:
            # UR3E.__init__ calls only the parent enable_cc, which creates an
            # empty checker. Explicitly populate the original UR3E link pairs.
            self.arm.enable_cc()

    def copy(self):
        return SingleArm(self.arm.copy())

    def fk(self, component_name, jnt_values):
        if component_name != 'arm':
            raise ValueError('SingleArm component must be arm')
        self.arm.fk(jnt_values)

    def rand_conf(self, component_name):
        return self.arm.rand_conf()

    def get_jnt_ranges(self, component_name):
        return self.arm.get_jnt_ranges()

    def is_collided(self, obstacle_list=(), otherrobot_list=()):
        return self.arm.is_collided(list(obstacle_list), list(otherrobot_list))

    def gen_meshmodel(self):
        return self.arm.gen_meshmodel()


class MobileDual(UR3EDual):
    """Kinematic base relocation. This is not a mobile-base dynamics model."""
    def set_base(self, pos, rotmat):
        self.pos, self.rotmat = np.array(pos), np.array(rotmat)
        for side in ('lft', 'rgt'):
            body = getattr(self, side + '_body')
            arm = getattr(self, side + '_arm')
            q = arm.get_jnt_values().copy()
            body.fix_to(self.pos, self.rotmat)
            arm.fix_to(body.jnts[-1]['gl_posq'], body.jnts[-1]['gl_rotmatq'])
            # Original fk also updates the gripper and held-object collision pose.
            self.fk(side + '_arm', q)
