import motionplanning.collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85nm
from motionplanning.rrt import ddrrtconnect as ddrrtc
# from motionplanning.rrt import ddrrtconnect_rtree as ddrrtc
from robotsim.nextage import nxt
from robotsim.nextage import nxtmesh
from robotsim.nextage import nxtball

def iscollidedfunc(point, obstaclelist = [], robot = None, cdchecker = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param robot the object defined in robotsim/robot
    :param cdchecker: a collisionchecker object
    :return:


    """

    robot.movearmfk(point)
    isselfcollided = cdchecker.isCollided(robot, obstaclelist)
    robot.goinitpose()

    if isselfcollided:
        return True
    else:
        return False

if __name__ == '__main__':
    from manipulation.grip.robotiq85 import rtq85

    base = pandactrl.World()

    robot = nxt.NxtRobot()
    rgthnd = rtq85.Rtq85()
    lfthnd = rtq85.Rtq85()
    robotmesh = nxtmesh.NxtMesh(rgthand=rgthnd, lfthand=lfthnd)
    robotball = nxtball.NxtBall()
    cdchecker = cdck.CollisionCheckerBall(robotball)

    start = [50.0,0.0,-143.0,0.0,0.0,0.0]
    goal = [-45.0,0.0,-143.0,0.0,0.0,0.0]
    # plot init and goal
    robot.movearmfk(armjnts = start, armid = 'rgt')
    rmi = robotmesh.genmnp(robot)
    rmi.reparentTo(base.render)
    robot.movearmfk(armjnts = goal, armid = 'rgt')
    rmg = robotmesh.genmnp(robot)
    rmg.reparentTo(base.render)

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']]]
    import os
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    obsrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,355.252044678,-150.073120117,200.0000038147,1.0)
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "manipulation/grip", "objects", "tool2.stl")
    objmnp = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objmnp.setMat(obsrotmat4)
    objmnp.reparentTo(base.render)
    #
    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                              cdchecker = cdchecker, jointlimits = jointlimits, starttreesamplerate=30,
                            expanddis = 5, robot = robot)

    import time
    tic = time.perf_counter()
    [path, sampledpoints] = planner.planning(obstaclelist = [objmnp])
    toc = time.perf_counter()
    print (toc-tic)
    import smoother as sm
    smoother = sm.Smoother()
    path = smoother.pathsmoothing(path, planner, 200)

    # if you want to plot the final sequence
    # for pose in path:
    #     robot.movearmfk(pose, armid = 'rgt')
    #     robotstick = robotmesh.gensnp(robot = robot)
    #     robotstick.reparentTo(base.render)
    # base.run()

    # if you want to show a video
    robot.goinitpose()
    rbtmnp = [None]
    objmnp = [None]
    counter = [0]
    def updateshow(rbtmnp, objmnp, counter, robot, objpath, task):
        if counter[0] < len(path):
            if rbtmnp[0] is not None:
                # rbtmnp[0].removeNode()
                pass
            if objmnp[0] is not None:
                objmnp[0].removeNode()
            robot.movearmfk(path[counter[0]], armid = 'rgt')
            rbtmnp[0] = robotmesh.gensnp(robot = robot)
            rbtmnp[0].reparentTo(base.render)
            objmnp[0] = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
            objmnp[0].setMat(obsrotmat4)
            objmnp[0].reparentTo(base.render)
            counter[0] += 1
        else:
            if rbtmnp[0] is not None:
                rbtmnp[0].removeNode()
                robot.goinitpose()
            rbtmnp[0] = robotmesh.genmnp(robot)
            rbtmnp[0].reparentTo(base.render)
        return task.again
    taskMgr.doMethodLater(.12, updateshow, "updateshow",
                          extraArgs = [rbtmnp, objmnp, counter, robot, objpath],
                          appendTask = True)
    base.run()