"""OmindOS Manipulation Community — Quick Start

A minimal simulation-only introduction to a joint-link chain, inverse
kinematics, forward kinematics, and Panda3D visualization.

Run from the repository root:
    python community/quick_start.py
"""

from pathlib import Path
import sys

# Preserve the documented direct-script invocation from any working directory.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from community.common import parser, seed_rng, require, report

import basis.robot_math as rm
import modeling.geometric_model as gm
import robot_sim._kinematics.jlchain as jlc
import visualization.panda.world as wd


def main():
    args = parser(__doc__).parse_args()
    seed_rng(args.seed)

    # Build a small educational joint-link chain.
    robot = jlc.JLChain(homeconf=np.zeros(5), name="omindos_quick_start")
    robot.jnts[4]["type"] = "prismatic"
    robot.jnts[4]["loc_motionax"] = np.array([1.0, 0.0, 0.0])
    robot.jnts[4]["motion_rng"] = [-0.5, 0.5]
    robot.reinitialize()

    # Define the tool center point (TCP).
    robot.set_tcp(
        tcp_jntid=4,
        tcp_loc_pos=np.array([0.2, -0.13, 0.0]),
        tcp_loc_rotmat=rm.rotmat_from_axangle(np.array([0.0, 0.0, 1.0]), np.pi / 8),
    )

    target_pos = np.array([0.3, 0.1, 0.0])
    target_rotmat = np.eye(3)

    joint_values = robot.ik(
        target_pos,
        target_rotmat,
        seed_jnt_values=None,
        local_minima="accept",
        toggle_debug=False,
    )

    if joint_values is None:
        raise RuntimeError("IK did not find a solution for the Quick Start target.")

    robot.fk(jnt_values=joint_values)
    actual_pos, actual_rot = robot.get_gl_tcp()
    pos_error = float(np.linalg.norm(actual_pos - target_pos))
    rot_error = float(np.linalg.norm(actual_rot - target_rotmat))
    require(pos_error < .002 and rot_error < .01, 'Quick Start IK residual exceeds tolerance')
    report(args, '01-quick-start', position_error_m=pos_error, rotation_matrix_error=rot_error)
    if not args.headless:
        world = wd.World(cam_pos=[1.2, .8, 1], lookat_pos=[.15, 0, .1])
        gm.gen_frame(length=.15).attach_to(world)
        gm.gen_mycframe(pos=target_pos, rotmat=target_rotmat, length=.12, thickness=.008).attach_to(world)
        robot.gen_stickmodel(toggle_jntscs=True).attach_to(world)
        world.run()


if __name__ == '__main__':
    main()
