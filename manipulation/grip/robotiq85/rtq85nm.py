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
import copy

from utils import designpattern

# import rtq85

class Rtq85NM():
    '''
    use utils.designpattern.singleton() to get a single instance of this class
    '''

    def __init__(self, jawwidth=85, hndcolor=None):
        '''
        load the robotiq85 model, set jawwidth and return a nodepath
        the rtq85 gripper is composed of a parallelism and a fixed triangle,
        the parallelism: 1.905-1.905; 5.715-5.715; 70/110 degree
        the triangle: 4.75 (finger) 5.715 (inner knuckle) 3.175 (outer knuckle)

        NOTE: the setColor function is only useful when the models dont have any materials

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
        rtq85basepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg/nomat", "robotiq_85_base_link_nm.egg"))
        rtq85fingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg/nomat", "robotiq_85_finger_link_nm.egg"))
        rtq85fingertippath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg/nomat", "robotiq_85_finger_tip_link_nm.egg"))
        rtq85innerknucklepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg/nomat", "robotiq_85_inner_knuckle_link_nm.egg"))
        rtq85knucklepath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg/nomat", "robotiq_85_knuckle_link_nm.egg"))

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
        if hndcolor is None:
            # rtq85base.setColor(.2,.2,.2,1)
            pass
        else:
            rtq85base.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85base.setTransparency(TransparencyAttrib.MAlpha)

        # left and right outer knuckle
        rtq85_knucklel.instanceTo(rtq85lknuckle)
        rtq85lknuckle.setPos(-30.60114443, 54.90451627, 0)
        rtq85lknuckle.setHpr(0, 0, 180)
        if hndcolor is None:
            # rtq85lknuckle.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85lknuckle.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85lknuckle.setTransparency(TransparencyAttrib.MAlpha)
        rtq85lknuckle.reparentTo(rtq85base)
        rtq85_knucklel.instanceTo(rtq85rknuckle)
        rtq85rknuckle.setPos(30.60114443, 54.90451627, 0)
        rtq85rknuckle.setHpr(0, 0, 0)
        if hndcolor is None:
            # rtq85rknuckle.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85rknuckle.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85rknuckle.setTransparency(TransparencyAttrib.MAlpha)
        rtq85rknuckle.reparentTo(rtq85base)

        # left and right finger
        rtq85_fingerl.instanceTo(rtq85lfgr)
        rtq85lfgr.setPos(31.48504435, -4.08552455, 0)
        if hndcolor is None:
            # rtq85lfgr.setColor(0.2,0.2,0.2,1)
            pass
        else:
            rtq85lfgr.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85lfgr.setTransparency(TransparencyAttrib.MAlpha)
        rtq85lfgr.reparentTo(rtq85lknuckle)
        rtq85_fingerl.instanceTo(rtq85rfgr)
        rtq85rfgr.setPos(31.48504435, -4.08552455, 0)
        if hndcolor is None:
            # rtq85rfgr.setColor(0.2,0.2,0.2,1)
            pass

        else:
            rtq85rfgr.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85rfgr.setTransparency(TransparencyAttrib.MAlpha)
        rtq85rfgr.reparentTo(rtq85rknuckle)

        # left and right inner knuckle
        rtq85_innerknucklel.instanceTo(rtq85ilknuckle)
        rtq85ilknuckle.setPos(-12.7, 61.42, 0)
        rtq85ilknuckle.setHpr(0, 0, 180)
        if hndcolor is None:
            # rtq85ilknuckle.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85ilknuckle.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85ilknuckle.setTransparency(TransparencyAttrib.MAlpha)
        rtq85ilknuckle.reparentTo(rtq85base)
        rtq85_innerknucklel.instanceTo(rtq85irknuckle)
        rtq85irknuckle.setPos(12.7, 61.42, 0)
        rtq85irknuckle.setHpr(0, 0, 0)
        if hndcolor is None:
            # rtq85irknuckle.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85irknuckle.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85irknuckle.setTransparency(TransparencyAttrib.MAlpha)
        rtq85irknuckle.reparentTo(rtq85base)

        # left and right fgr tip
        rtq85_fingertipl.instanceTo(rtq85lfgrtip)
        rtq85lfgrtip.setPos(37.59940821, 43.03959807, 0)
        if hndcolor is None:
            # rtq85lfgrtip.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85lfgrtip.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85lfgrtip.setTransparency(TransparencyAttrib.MAlpha)
        rtq85lfgrtip.reparentTo(rtq85ilknuckle)
        rtq85_fingertipl.instanceTo(rtq85rfgrtip)
        rtq85rfgrtip.setPos(37.59940821, 43.03959807, 0)
        rtq85rfgrtip.setHpr(0, 0, 0)
        if hndcolor is None:
            # rtq85rfgrtip.setColor(.5,.5,.5,1)
            pass
        else:
            rtq85rfgrtip.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
        rtq85rfgrtip.setTransparency(TransparencyAttrib.MAlpha)
        rtq85rfgrtip.reparentTo(rtq85irknuckle)

        # rotate to x, y, z coordinates (this one rotates the base, not the self.rtq85np)
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

    @property
    def handnp(self):
        return self.rtq85np

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
        if jawwidth/2.0 >= 5:
            rotiknuckle=41-math.asin((jawwidth/2.0-5)/57.15)*180/math.pi
        else:
            rotiknuckle=41+math.asin((5-jawwidth/2.0)/57.15)*180/math.pi
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

    def setColor(self, rgbacolor=[1,0,0,.1]):
        """
        set the color of the hand

        :param rgbacolor:
        :return:

        author: weiwei
        date: 20161212
        """

        # base
        rtq85base = self.rtq85np.find("**/rtq85base")
        rtq85base.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])

        # right finger
        rtq85irknuckle = self.rtq85np.find("**/rtq85irknuckle")
        rtq85irknuckle.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85rknuckle = self.rtq85np.find("**/rtq85rknuckle")
        rtq85rknuckle.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85rfgr = self.rtq85np.find("**/rtq85rfgr")
        rtq85rfgr.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85rfgrtip = self.rtq85np.find("**/rtq85rfgrtip")
        rtq85rfgrtip.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])

        # left finger
        rtq85ilknuckle = self.rtq85np.find("**/rtq85ilknuckle")
        rtq85ilknuckle.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85lknuckle = self.rtq85np.find("**/rtq85lknuckle")
        rtq85lknuckle.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85lfgr = self.rtq85np.find("**/rtq85lfgr")
        rtq85lfgr.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        rtq85lfgrtip = self.rtq85np.find("**/rtq85lfgrtip")
        rtq85lfgrtip.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])

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

        :return:npvec3
        """

        return self.rtq85np.getPos()

    def setMat(self, nodepath = None, pandanpmat4 = Mat4.identMat()):
        """
        set the translation and rotation of a robotiq hand
        changes self.rtq85np

        :param npmat4: follows panda3d, a LMatrix4f matrix
        :return: null

        date: 20161109
        author: weiwei
        """

        self.rtq85np.setMat(pandanpmat4)

    def getMat(self):
        """
        get the rotation matrix of the hand

        :return: npmat4: follows panda3d, a LMatrix4f matrix

        date: 20161109
        author: weiwei
        """

        return copy.deepcopy(self.rtq85np.getMat())

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

    def detachNode(self):
        """

        :return:
        """

        self.rtq85np.detachNode()

    def lookAt(self, direct0, direct1, direct2):
        """
        set the Y axis of the hnd

        author: weiwei
        date: 20161212
        """

        self.rtq85np.lookAt(direct0, direct1, direct2)

    def xAlong(self, x, y, z):
        """
        set the X axis of the hnd along [x,y,z]

        author: weiwei
        date: 20170313
        """

        rotmat4z = Mat4.rotateMat(90, Vec3(0, 0, 1))

        self.rtq85np.setMat(Mat4.identMat())
        self.rtq85np.lookAt(x, y, z)
        self.rtq85np.setMat(rotmat4z*self.rtq85np.getMat())

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

def newHandNM(jawwidth = 85, hndcolor = None):
    return Rtq85NM(jawwidth, hndcolor)

def newHand(jawwidth = 85):
    return rtq85.Rtq85(jawwidth)

def newHandFgrpcc():
    this_dir, this_filename = os.path.split(__file__)
    handfgrpccpath = Filename.fromOsSpecific(os.path.join(this_dir, "rtq85egg", "robotiq_85_tip_precc.egg"))
    handfgrpcc = loader.loadModel(handfgrpccpath)
    return handfgrpcc

def getHandName():
    return "rtq85"

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

    base = pandactrl.World(lookatp=[0,0,0])
    rtq85hnd = Rtq85NM(hndcolor=[.5,.5,0.5,.7])
    rtq85hnd.setJawwidth(20)
    rtq85hnd.gripAt(0,0,0,1,0,0,30,0.0)
    rtq85hnd.reparentTo(base.render)

    # axis = loader.loadModel('zup-axis.egg')
    # axis.reparentTo(base.render)

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
    # rtq85hnd.rtq85np.find("**/rtq85base").showTightBounds()


    # rtq85hnd.rtq85np.find("**/rtq85lfgrtip").showTightBounds()
    glftnp = rtq85hnd.rtq85np.find("**/rtq85lfgrtip").find("**/+GeomNode")
    glft = glftnp.node().getGeom(0)
    glftts = glftnp.getTransform(base.render)
    glftmesh = BulletTriangleMesh()
    glftmesh.addGeom(glft)
    lftbullnode = BulletRigidBodyNode('glft')
    lftbullnode.addShape(BulletTriangleMeshShape(glftmesh, dynamic=True), glftts)
    lftcollidernp=bullcldrnp.attachNewNode(lftbullnode)
    base.world.attachRigidBody(lftbullnode)
    lftcollidernp.setCollideMask(BitMask32.allOn())
    # rtq85hnd.rtq85np.find("**/rtq85lfgrtip").showTightBounds()

    # rtq85hnd.rtq85np.find("**/rtq85ilknuckle").showTightBounds()
    gilknp = rtq85hnd.rtq85np.find("**/rtq85ilknuckle").find("**/+GeomNode")
    gilk = gilknp.node().getGeom(0)
    gilkts = gilknp.getTransform(base.render)
    gilkmesh = BulletTriangleMesh()
    gilkmesh.addGeom(gilk)
    ilkbullnode = BulletRigidBodyNode('gilk')
    ilkbullnode.addShape(BulletTriangleMeshShape(gilkmesh, dynamic=True), gilkts)
    ilkcollidernp=bullcldrnp.attachNewNode(ilkbullnode)
    base.world.attachRigidBody(ilkbullnode)
    ilkcollidernp.setCollideMask(BitMask32.allOn())
    # rtq85hnd.rtq85np.find("**/rtq85ilknuckle").showTightBounds()
    # rtq85hnd.rtq85np.showTightBounds()

    glknp = rtq85hnd.rtq85np.find("**/rtq85lknuckle").find("**/+GeomNode")
    glk = glknp.node().getGeom(0)
    glkts = glknp.getTransform(base.render)
    glkmesh = BulletTriangleMesh()
    glkmesh.addGeom(glk)
    lkbullnode = BulletRigidBodyNode('glk')
    lkbullnode.addShape(BulletTriangleMeshShape(glkmesh, dynamic=True), glkts)
    lkcollidernp=bullcldrnp.attachNewNode(lkbullnode)
    base.world.attachRigidBody(lkbullnode)
    lkcollidernp.setCollideMask(BitMask32.allOn())
    # rtq85hnd.rtq85np.find("**/rtq85lknuckle").showTightBounds()
    # rtq85hnd.rtq85np.showTightBounds()


    glfgrnp = rtq85hnd.rtq85np.find("**/rtq85lfgr").find("**/+GeomNode")
    glfgr = glfgrnp.node().getGeom(0)
    glfgrts = glfgrnp.getTransform(base.render)
    glfgrmesh = BulletTriangleMesh()
    glfgrmesh.addGeom(glfgr)
    lfgrbullnode = BulletRigidBodyNode('glfgr')
    lfgrbullnode.addShape(BulletTriangleMeshShape(glfgrmesh, dynamic=True), glfgrts)
    lfgrcollidernp=bullcldrnp.attachNewNode(lfgrbullnode)
    base.world.attachRigidBody(lfgrbullnode)
    lfgrcollidernp.setCollideMask(BitMask32.allOn())
    # rtq85hnd.rtq85np.find("**/rtq85lfgr").showTightBounds()
    # rtq85hnd.rtq85np.showTightBounds()

    base.taskMgr.add(updateworld, "updateworld", extraArgs=[base.world], appendTask=True)
    result = base.world.contactTestPair(ilkbullnode, lftbullnode)
    print (result)
    print (result.getContacts())
    import pandaplotutils.pandageom as pandageom
    pggen = pandageom.PandaGeomGen()
    for contact in result.getContacts():
        cp = contact.getManifoldPoint()
        print( cp.getLocalPointA())
        pggen.plotSphere(base, pos=cp.getLocalPointA(), radius=10, rgba=Vec4(1,0,0,1))

        pggen.plotAxisSelf(base.render, spos = Vec3(0,0,0))

    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()

    base.world.setDebugNode(debugNP.node())
    pggen.plotAxis(base.render)

    base.run()