import robot_sim.robots.ur3e_dual.ur3e_dual as u3ed
import manipulation.handover.handover as hd
import visualization.panda.world as wd
import modeling.geometric_model as gm
import basis
import os
import trimesh
import modeling.collision_model as cm
import numpy as np


if __name__ == '__main__':
    # load base environment
    base = wd.World(cam_pos=[5, 0, 3], lookat_pos=[0, 0, 1])
    gm.gen_frame().attach_to(base)
    # load obj path
    objpath0 = os.path.join(basis.__path__[0], 'objects', 'roller.stl')
    objpath1 = os.path.join(basis.__path__[0], 'objects', 'hole.stl')
    pose_gen = hd.FloatingPoses(objpath0=objpath0, objpath1=objpath1)
    posmat0, posmat1 = pose_gen.genPandaRotmat4()
    print(posmat0[0])
    print(posmat1[0])

    print(type(posmat0[0]))
    nposmat = np.array(posmat0)
    print(type(nposmat))
    print(nposmat)

    roller_mesh = cm.CollisionModel(objpath0, cdprimit_type='polygons')
    roller_mesh.set_homomat(nposmat[0])
    roller_mesh.attach_to(base)

    roller_mesh1 = cm.CollisionModel(objpath0, cdprimit_type='polygons')
    roller_mesh1.set_homomat(nposmat[1])
    roller_mesh1.attach_to(base)

    roller_mesh2 = cm.CollisionModel(objpath0, cdprimit_type='polygons')
    roller_mesh2.set_homomat(nposmat[2])
    roller_mesh2.attach_to(base)

    # roller_mesh.attach_to(base)
    # roller_mesh = trimesh.load_mesh(objpath0)
    # # print(roller_mesh.face_normals)
    base.run()


