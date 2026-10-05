"""OmindOS Manipulation Community — Quick Start

A minimal simulation-only introduction to a joint-link chain, inverse
kinematics, forward kinematics, and Panda3D visualization.

Run from the repository root:
    python community/quick_start.py
"""

import numpy as np

import basis.robot_math as rm
import modeling.geometric_model as gm
import robot_sim._kinematics.jlchain as jlc
import visualization.panda.world as wd


def main():
    world = wd.World(cam_pos=[1.2, 0.8, 1.0], lookat_pos=[0.15, 0.0, 0.1])
    gm.gen_frame(length=0.15).attach_to(world)

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

    gm.gen_mycframe(
        pos=target_pos,
        rotmat=target_rotmat,
        length=0.12,
        thickness=0.008,
    ).attach_to(world)

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
    robot.gen_stickmodel(toggle_jntscs=True).attach_to(world)

    print("OmindOS Manipulation Community Quick Start")
    print("IK solution:", joint_values)
    print("A Panda3D window should now show the robot and target frame.")
    print("Close the window to exit.")

    world.run()


if __name__ == "__main__":
    main()
