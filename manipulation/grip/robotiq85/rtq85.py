# TODO: reduce the dependency on panda3d

import math
import os

import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *

from utils import designpattern

class Rtq85():
    '''
    use utils.designpattern.singleton() to get a single instance of this class
    '''

    def __init__(self, jawwidth=85):
        '''
        load the robotiq85 model, set jawwidth and return a nodepath
        the rtq85 gripper is composed of a parallelism and a fixed triangle,
        the parallelism: 1.905-1.905; 5.715-5.715; 70/110 degree
        the triangle: 4.75 (finger) 5.715 (inner knuckle) 3.175 (outer knuckle)

        ## input
        pandabase:
            the showbase() object
        jawwidth:
            the distance between fingertips

        ## output
        rtq85np:
            the nodepath of this rtq85 hand

        author: weiwei
        date: 20160627
        '''
        self.rtq85np = NodePath("rtq85hnd")
        self.handnp = self.rtq85np
        self.jawwidth = jawwidth

        this_dir, this_filename = os.path.split(__file__)
        rtq85basepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_base_link.egg"))
        rtq85fingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_finger_link.egg"))
        rtq85fingertippath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_finger_tip_link.egg"))
        rtq85innerknucklepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_inner_knuckle_link.egg"))
        rtq85knucklepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_knuckle_link.egg"))

        rtq85base = NodePath("rtq85base")
        rtq85lknuckle = NodePath("rtq85lknuckle")
        rtq85rknuckle = NodePath("rtq85rknuckle")
        rtq85lfgr = NodePath("rtq85lfgr")
        rtq85rfgr = NodePath("rtq85rfgr")
        rtq85ilknuckle = NodePath("rtq85ilknuckle")
        rtq85irknuckle = NodePath("rtq85irknuckle")
        rtq85lfgrtip = NodePath("rtq85lfgrtip")
        rtq85rfgrtip = NodePath("rtq85rfgrtip")

        # loader is a global variable defined by panda3d
        rtq85_basel = loader.loadModel(rtq85basepath)
        rtq85_fingerl = loader.loadModel(rtq85fingerpath)
        rtq85_fingertipl = loader.loadModel(rtq85fingertippath)
        rtq85_innerknucklel = loader.loadModel(rtq85innerknucklepath)
        rtq85_knucklel = loader.loadModel(rtq85knucklepath)

        # base
        rtq85_basel.instanceTo(rtq85base)
        rtq85base.setPos(0,0,0)

        # left and right outer knuckle
        rtq85_knucklel.instanceTo(rtq85lknuckle)
        rtq85lknuckle.setPos(-30.60114443, 54.90451627, 0)
        rtq85lknuckle.setHpr(0, 0, 180)
        rtq85lknuckle.reparentTo(rtq85base)
        rtq85_knucklel.instanceTo(rtq85rknuckle)
        rtq85rknuckle.setPos(30.60114443, 54.90451627, 0)
        rtq85rknuckle.setHpr(0, 0, 0)
        rtq85rknuckle.reparentTo(rtq85base)

        # left and right finger
        rtq85_fingerl.instanceTo(rtq85lfgr)
        rtq85lfgr.setPos(31.48504435, -4.08552455, 0)
        rtq85lfgr.reparentTo(rtq85lknuckle)
        rtq85_fingerl.instanceTo(rtq85rfgr)
        rtq85rfgr.setPos(31.48504435, -4.08552455, 0)
        rtq85rfgr.reparentTo(rtq85rknuckle)

        # left and right inner knuckle
        rtq85_innerknucklel.instanceTo(rtq85ilknuckle)
        rtq85ilknuckle.setPos(-12.7, 61.42, 0)
        rtq85ilknuckle.setHpr(0, 0, 180)
        rtq85ilknuckle.reparentTo(rtq85base)
        rtq85_innerknucklel.instanceTo(rtq85irknuckle)
        rtq85irknuckle.setPos(12.7, 61.42, 0)
        rtq85irknuckle.setHpr(0, 0, 0)
        rtq85irknuckle.reparentTo(rtq85base)

        # left and right fgr tip
        rtq85_fingertipl.instanceTo(rtq85lfgrtip)
        rtq85lfgrtip.setPos(37.59940821, 43.03959807, 0)
        rtq85lfgrtip.reparentTo(rtq85ilknuckle)
        rtq85_fingertipl.instanceTo(rtq85rfgrtip)
        rtq85rfgrtip.setPos(37.59940821, 43.03959807, 0)
        rtq85rfgrtip.setHpr(0, 0, 0)
        rtq85rfgrtip.reparentTo(rtq85irknuckle)

        # rotate to x, y, z coordinates (this one rotates the base, not the self.rtq85np)
        # the default x direction is facing the ee, the default z direction is facing downward
        # execute this file to see the default pose
        rtq85base.setMat(pandageom.cvtMat4(rm.rodrigues([0,0,1], 90))*rtq85base.getMat())
        rtq85base.reparentTo(self.rtq85np)
        self.setJawwidth(jawwidth)

        self.__jawwidthopen = 85.0
        self.__jawwidthclosed = 0.0

    @property
    def jawwidthopen(self):
        # read-only property
        return self.__jawwidthopen

    @property
    def jawwidthclosed(self):
        # read-only property
        return self.__jawwidthclosed

    def setJawwidth(self, jawwidth):
        '''
        set the jawwidth of rtq85hnd
        the formulea is deduced on a note book
        the rtq85 gripper is composed of a parallelism and a fixed triangle,
        the parallelism: 1.905-1.905; 5.715-5.715; 70/110 degree
        the triangle: 4.75 (finger) 5.715 (inner knuckle) 3.175 (outer knuckle)

        ## input
        rtq85hnd:
            nodepath of a robotiq85hand
        jawwidth:
            the width of the jaw

        author: weiwei
        date: 20160627
        '''
        assert(jawwidth <= 85)
        assert(jawwidth >= 0)

        self.jawwidth = jawwidth

        rotiknuckle = 0
        if jawwidth/2 >= 5:
            rotiknuckle=41-math.asin((jawwidth/2-5)/57.15)*180/math.pi
        else:
            rotiknuckle=41+math.asin((5-jawwidth/2)/57.15)*180/math.pi
        # print rotiknuckle

        # right finger
        rtq85irknuckle = self.rtq85np.find("**/rtq85irknuckle")
        rtq85irknucklehpr = rtq85irknuckle.getHpr()
        rtq85irknuckle.setHpr(rotiknuckle, rtq85irknucklehpr[1], rtq85irknucklehpr[2])
        rtq85rknuckle = self.rtq85np.find("**/rtq85rknuckle")
        rtq85rknucklehpr = rtq85rknuckle.getHpr()
        rtq85rknuckle.setHpr(rotiknuckle, rtq85rknucklehpr[1], rtq85rknucklehpr[2])
        rtq85rfgrtip = self.rtq85np.find("**/rtq85rfgrtip")
        rtq85rfgrtiphpr = rtq85rfgrtip.getHpr()
        rtq85rfgrtip.setHpr(-rotiknuckle, rtq85rfgrtiphpr[1], rtq85rfgrtiphpr[2])

        # left finger
        rtq85ilknuckle = self.rtq85np.find("**/rtq85ilknuckle")
        rtq85ilknucklehpr = rtq85ilknuckle.getHpr()
        rtq85ilknuckle.setHpr(-rotiknuckle, rtq85ilknucklehpr[1], rtq85ilknucklehpr[2])
        rtq85lknuckle = self.rtq85np.find("**/rtq85lknuckle")
        rtq85lknucklehpr = rtq85lknuckle.getHpr()
        rtq85lknuckle.setHpr(-rotiknuckle, rtq85lknucklehpr[1], rtq85lknucklehpr[2])
        rtq85lfgrtip = self.rtq85np.find("**/rtq85lfgrtip")
        rtq85lfgrtiphpr = rtq85rfgrtip.getHpr()
        rtq85lfgrtip.setHpr(-rotiknuckle, rtq85lfgrtiphpr[1], rtq85lfgrtiphpr[2])

    def setPos(self, npvec3):
        """
        set the pose of the hand
        changes self.rtq85np

        :param npvec3
        :return:
        """

        self.rtq85np.setPos(npvec3)

    def getPos(self):
        """
        set the pose of the hand
        changes self.rtq85np

        :param npvec3
        :return:
        """

        return self.rtq85np.getPos()

    def setMat(self, npmat4):
        """
        set the translation and rotation of a robotiq hand
        changes self.rtq85np

        :param npmat4: follows panda3d, a LMatrix4f matrix
        :return: null

        date: 20161109
        author: weiwei
        """

        self.rtq85np.setMat(npmat4)

    def getMat(self):
        """
        get the rotation matrix of the hand

        :return: npmat4: follows panda3d, a LMatrix4f matrix

        date: 20161109
        author: weiwei
        """

        return self.rtq85np.getMat()

    def reparentTo(self, nodepath):
        """
        add to scene, follows panda3d

        :param nodepath: a panda3d nodepath
        :return: null

        date: 20161109
        author: weiwei
        """
        self.rtq85np.reparentTo(nodepath)

    def removeNode(self):
        """

        :return:
        """

        self.rtq85np.removeNode()

    def lookAt(self, direct0, direct1, direct2):
        """
        set the Y axis of the hnd

        author: weiwei
        date: 20161212
        """

        self.rtq85np.lookAt(direct0, direct1, direct2)

    def gripAt(self, fcx, fcy, fcz, c0nx, c0ny, c0nz, rotangle = 0, jawwidth = 82):
        '''
        set the hand to grip at fcx, fcy, fcz, fc = finger center
        the normal of the sglfgr contact is set to be c0nx, c0ny, c0nz
        the rotation around the normal is set to rotangle
        the jawwidth is set to jawwidth

        date: 20170322
        author: weiwei
        '''

        self.rtq85np.setMat(Mat4.identMat())
        self.setJawwidth(jawwidth)
        self.rtq85np.lookAt(c0nx, c0ny, c0nz)
        rotmat4x = Mat4.rotateMat(rotangle, Vec3(c0nx, c0ny, c0nz))
        self.rtq85np.setMat(self.rtq85np.getMat()*rotmat4x)
        rotmat4 = Mat4(self.rtq85np.getMat())
        handtipvec3 = rotmat4.getRow3(0)*(145.0)
        rotmat4.setRow(3, Vec3(fcx, fcy, fcz)+handtipvec3)
        self.rtq85np.setMat(rotmat4)

    # def plot(self, nodepath, pos=None, ydirect=None, zdirect=None, rgba=None):
    #     '''
    #     plot the hand under the given nodepath
    #
    #     ## input
    #     nodepath:
    #         the parent node this hand is going to be attached to
    #     pos:
    #         the position of the hand
    #     ydirect:
    #         the y direction of the hand
    #     zdirect:
    #         the z direction of the hand
    #     rgba:
    #         the rgba color
    #
    #     ## note:
    #         dot(ydirect, zdirect) must be 0
    #
    #     date: 20160628
    #     author: weiwei
    #     '''
    #
    #     if pos is None:
    #         pos = Vec3(0,0,0)
    #     if ydirect is None:
    #         ydirect = Vec3(0,1,0)
    #     if zdirect is None:
    #         zdirect = Vec3(0,0,1)
    #     if rgba is None:
    #         rgba = Vec4(1,1,1,0.5)
    #
    #     # assert(ydirect.dot(zdirect)==0)
    #
    #     placeholder = nodepath.attachNewNode("rtq85holder")
    #     self.rtq85np.instanceTo(placeholder)
    #     xdirect = ydirect.cross(zdirect)
    #     transmat4 = Mat4()
    #     transmat4.setCol(0, xdirect)
    #     transmat4.setCol(1, ydirect)
    #     transmat4.setCol(2, zdirect)
    #     transmat4.setCol(3, pos)
    #     self.rtq85np.setMat(transmat4)
    #     placeholder.setColor(rgba)

if __name__=='__main__':

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        # result = base.world.contactTestPair(bcollidernp.node(), lftcollidernp.node())
        # result1 = base.world.contactTestPair(bcollidernp.node(), ilkcollidernp.node())
        # result2 = base.world.contactTestPair(lftcollidernp.node(), ilkcollidernp.node())
        # print result
        # print result.getContacts()
        # print result1
        # print result1.getContacts()
        # print result2
        # print result2.getContacts()
        # for contact in result.getContacts():
        #     cp = contact.getManifoldPoint()
        #     print cp.getLocalPointA()
        return task.cont

    base = pandactrl.World()
    rtq85hnd = designpattern.singleton(Rtq85)
    rtq85hnd.setJawwidth(70)
    hndpos = Vec3(0,0,0)
    ydirect = Vec3(0,1,0)
    zdirect = Vec3(0,0,1)
    # rtq85hnd.gripAt(0,0,0,1,0,0)
    rtq85hnd.reparentTo(base.render)
    base.pggen.plotAxis(base.render)

    bullcldrnp = base.render.attachNewNode("bulletcollider")
    base.world = BulletWorld()

    # hand base
    # rtq85hnd.rtq85np.find("**/rtq85base").showTightBounds()
    gbnp = rtq85hnd.rtq85np.find("**/rtq85base").find("**/+GeomNode")
    gb = gbnp.node().getGeom(0)
    gbts = gbnp.getTransform(base.render)
    gbmesh = BulletTriangleMesh()
    gbmesh.addGeom(gb)
    bbullnode = BulletRigidBodyNode('gb')
    bbullnode.addShape(BulletTriangleMeshShape(gbmesh, dynamic=True), gbts)
    bcollidernp=bullcldrnp.attachNewNode(bbullnode)
    base.world.attachRigidBody(bbullnode)
    bcollidernp.setCollideMask(BitMask32.allOn())

    # rtq85hnd.rtq85np.find("**/rtq85lfgrtip").showTightBounds()
    glftnp = rtq85hnd.rtq85np.find("**/rtq85lfgrtip").find("**/+GeomNode")
    glft = glftnp.node().getGeom(0)
    glftts = glftnp.getTransform(base.render)
    glftmesh = BulletTriangleMesh()
    glftmesh.addGeom(glft)
    # lftbullnode = BulletRigidBodyNode('glft')
    # lftbullnode.addShape(BulletTriangleMeshShape(glftmesh, dynamic=True), glftts)
    # lftcollidernp=bullcldrnp.attachNewNode(lftbullnode)
    # base.world.attachRigidBody(lftbullnode)
    # lftcollidernp.setCollideMask(BitMask32.allOn())
    # base.world.attachRigidBody(glftbullnode)

    # rtq85hnd.rtq85np.find("**/rtq85ilknuckle").showTightBounds()
    gilknp = rtq85hnd.rtq85np.find("**/rtq85ilknuckle").find("**/+GeomNode")
    gilk = gilknp.node().getGeom(0)
    gilkts = gilknp.getTransform(base.render)
    gilkmesh = BulletTriangleMesh()
    gilkmesh.addGeom(gilk)
    ilkbullnode = BulletRigidBodyNode('gilk')
    ilkbullnode.addShape(BulletTriangleMeshShape(gilkmesh, dynamic=True), gilkts)
    ilkbullnode.addShape(BulletTriangleMeshShape(glftmesh, dynamic=True), glftts)
    ilkcollidernp=bullcldrnp.attachNewNode(ilkbullnode)
    base.world.attachRigidBody(ilkbullnode)
    ilkcollidernp.setCollideMask(BitMask32.allOn())
    # rtq85hnd.rtq85np.find("**/rtq85ilknuckle").showTightBounds()
    # rtq85hnd.rtq85np.showTightBounds()

    base.taskMgr.add(updateworld, "updateworld", extraArgs=[base.world], appendTask=True)
    result = base.world.contactTestPair(bbullnode, ilkbullnode)
    print (result)
    print (result.getContacts())
    for contact in result.getContacts():
        cp = contact.getManifoldPoint()
        print (cp.getLocalPointA())
        base.pggen.plotSphere(base.render, pos=cp.getLocalPointA(), radius=1, rgba=Vec4(1,0,0,1))

    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()

    base.world.setDebugNode(debugNP.node())

    base.run()