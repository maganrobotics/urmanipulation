import numpy as np
import os
from panda3d.core import *
from utils import robotmath as rm
from shapely.geometry import Polygon
from trimesh import geometry as trigeom
import math
import trimesh

class PandaGeomGen(object):
    """
    use class to preload files
    and generate various models
    """

    def __init__(self):
        """
        prepload the files
        the models will be instanceTo nodepaths to avoid frequent disk access
        """

        this_dir, this_filename = os.path.split(__file__)
        cylinderpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cylinder.egg"))
        spherepath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "sphere.egg"))
        conepath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cone.egg"))
        boxpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "box.egg"))
        self.cylinder = loader.loadModel(cylinderpath)
        self.sphere = loader.loadModel(spherepath)
        self.cone = loader.loadModel(conepath)
        self.box  = loader.loadModel(boxpath)

    def genDumbbell(self, spos = None, epos = None, length = None, thickness = 1.5,
                    rgba = None, plotname = "dumbbell", headscale = 2):
        """
        generate a dumbbell to plot the stick model of a robot
        the function is essentially a copy of pandaplotutils/pandageom.plotDumbbell
        it uses preloaded models to avoid repeated disk access

        :param spos: 1-by-3 nparray or list, starting position of the arrow
        :param epos: 1-by-3 nparray or list, goal position of the arrow
        :param length: if length is None, its value will be computed using np.linalg.norm(epos-spos)
        :param thickness:
        :param rgba: 1-by-4 nparray or list
        :param plotname:
        :param headscale: a ratio between headbell and stick
        :return: a dumbbell nodepath


        """

        if spos is None:
            spos = np.array([0,0,0])
        if epos is None:
            epos = np.array([0,0,1])
        if length is None:
            length = np.linalg.norm(epos-spos)
        if rgba is None:
            rgba = np.array([1,1,1,1])

        dumbbell = NodePath(plotname)
        dumbbellbody_nodepath = NodePath("dumbbellbody")
        dumbbellhead0_nodepath = NodePath("dumbbellhead0")
        dumbbellhead1_nodepath = NodePath("dumbbellhead1")
        self.cylinder.instanceTo(dumbbellbody_nodepath)
        self.sphere.instanceTo(dumbbellhead0_nodepath)
        self.sphere.instanceTo(dumbbellhead1_nodepath)
        dumbbellbody_nodepath.setPos(0,0,0)
        dumbbellbody_nodepath.setScale(thickness, length, thickness)
        dumbbellhead0_nodepath.setPos(dumbbellbody_nodepath.getX(),
                                      length, dumbbellbody_nodepath.getZ())
        dumbbellhead0_nodepath.setScale(thickness*headscale, thickness*headscale, thickness*headscale)
        dumbbellhead1_nodepath.setPos(dumbbellbody_nodepath.getX(),
                                      dumbbellbody_nodepath.getY(),
                                      dumbbellbody_nodepath.getZ())
        dumbbellhead1_nodepath.setScale(thickness*headscale, thickness*headscale, thickness*headscale)
        dumbbellbody_nodepath.reparentTo(dumbbell)
        dumbbellhead0_nodepath.reparentTo(dumbbell)
        dumbbellhead1_nodepath.reparentTo(dumbbell)

        dumbbell.setPos(spos[0], spos[1], spos[2])
        dumbbell.lookAt(epos[0], epos[1], epos[2])
        dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

        return dumbbell

    def genBox(self, x = 1.0, y = 1.0, z = 1.0, plotname = "box"):
        """
        Generate a box for plot
        This function should not be called explicitly

        ## input
        x,y,z:
            the thickness of the box along x, y, and z axis

        ## output
        box: pathnode


        """

        x = x/2.0
        y = y/2.0
        z = z/2.0

        verts = [np.array([-x,-y,-z]),
                 np.array([+x,-y,-z]),
                 np.array([-x,+y,-z]),
                 np.array([+x,+y,-z]),
                 np.array([-x,-y,+z]),
                 np.array([+x,-y,+z]),
                 np.array([-x,+y,+z]),
                 np.array([+x,+y,+z])]

        faces = [np.array([0,4,2]),
                 np.array([6,2,4]),
                 np.array([2,3,0]),
                 np.array([1,0,3]),
                 np.array([2,6,3]),
                 np.array([7,3,6]),
                 np.array([7,5,3]),
                 np.array([1,3,5]),
                 np.array([0,1,4]),
                 np.array([5,4,1]),
                 np.array([6,4,7]),
                 np.array([5,7,4])]

        normals = []
        for face in faces:
            vert0 = verts[face[0]]
            vert1 = verts[face[1]]
            vert2 = verts[face[2]]
            vec10 = vert1-vert0
            vec20 = vert2-vert0
            rawnormal = np.cross(vec10, vec20)
            normals.append(rawnormal/np.linalg.norm(rawnormal))

        cobnp = packpandanp_fn(np.asarray(verts), np.asarray(normals), np.asarray(faces), name = plotname)
        return cobnp

    def genArrow(self, length, thickness = 1.5, plotname = "arrow"):
        """
        Generate a arrow node for plot
        This function should not be called explicitly

        ## input
        length:
            length of the arrow
        thickness:
            thickness of the arrow, set to 0.005 as default

        ## output

        """

        arrow = NodePath(plotname)
        arrowbody = NodePath(plotname+"body")
        arrowhead = NodePath(plotname+"head")
        self.cylinder.instanceTo(arrowbody)
        self.cone.instanceTo(arrowhead)
        arrowbody.setPos(0,0,0)
        arrowbody.setScale(thickness, length, thickness)
        arrowbody.reparentTo(arrow)
        arrowhead.setPos(arrow.getX(), length, arrow.getZ())
        # set scale (consider relativitly)
        arrowhead.setScale(thickness*2, thickness*4, thickness*2)
        arrowhead.reparentTo(arrow)

        return arrow

    def genStick(self, length, thickness = 2, plotname="stick"):
        """
        Generate a stick node for plot
        This function should not be called explicitly

        ## input
        length:
            length of the stick
        thickness:
            thickness of the stick, set to 0.005 as default

        ## output

        """

        stick = NodePath(plotname)
        stickbody = NodePath(plotname+"body")
        self.cylinder.instanceTo(stickbody)
        stickbody.setPos(0,0,0)
        stickbody.setScale(thickness, length, thickness)
        stickbody.reparentTo(stick)

        return stick

    def genSphere(self, radius = None, plotname = "sphere"):
        """
        Generate a sphere for plot
        This function should not be called explicitly

        ## input
        radius:
            the radius of the sphere

        ## output
        sphere: pathnode


        """

        if radius is None:
            radius = 0.05

        spherenp = NodePath(plotname)
        spherenpbody = NodePath(plotname+"body")
        self.sphere.instanceTo(spherenpbody)
        spherenpbody.setPos(0,0,0)
        spherenpbody.setScale(radius, radius, radius)
        spherenpbody.reparentTo(spherenp)

        return spherenp

    def plotArrow(self, nodepath, spos = None, epos = None, length = None, thickness = 1.5, rgba=None):
        """
        plot an arrow to nodepath

        ## input:
        pandabase:
            the panda direct.showbase.ShowBase object
            will be sent to _genArrow
        nodepath:
            defines which parent should the arrow be attached to
        spos:
            1-by-3 nparray or list, starting position of the arrow
        epos:
            1-by-3 nparray or list, goal position of the arrow
        length:
            will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
        thickness:
            will be sent to _genArrow
        rgba:
            1-by-3 nparray or list


        """

        if spos is None:
            spos = np.array([0,0,0])
        if epos is None:
            epos = np.array([0,0,1])
        if length is None:
            length = np.linalg.norm(epos-spos)
        if rgba is None:
            rgba = np.array([1,1,1,1])

        arrow = self.genArrow(length, thickness)
        arrow.setPos(spos[0], spos[1], spos[2])
        arrow.lookAt(epos[0], epos[1], epos[2])
        # lookAt points y+ to epos, use the following command to point x+ to epos
        # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
        # arrow.setHpr(arrow, Vec3(0,0,90))
        arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        arrow.reparentTo(nodepath)

        return arrow

    def plotDumbbell(self, nodepath, spos=None, epos=None, length=None, thickness=1.5, rgba=None, plotname="dumbbell",
                     headscale=2):
        """
        plot a dumbbell to nodepath

        ## input:
        nodepath:
            defines which parent should the arrow be attached to
        spos:
            1-by-3 nparray or list, starting position of the arrow
        epos:
            1-by-3 nparray or list, goal position of the arrow
        length:
            will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
        thickness:
            will be sent to _genArrow
        rgba:
            1-by-4 nparray or list


        """

        if spos is None:
            spos = np.array([0, 0, 0])
        if epos is None:
            epos = np.array([0, 0, 1])
        if length is None:
            length = np.linalg.norm(epos - spos)
        if rgba is None:
            rgba = np.array([1, 1, 1, 1])

        dumbbell = self.genDumbbell(length=length, thickness=thickness, plotname=plotname, headscale=headscale)
        dumbbell.setPos(spos[0], spos[1], spos[2])
        dumbbell.lookAt(epos[0], epos[1], epos[2])
        # lookAt points y+ to epos, use the following command to point x+ to epos
        # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
        # arrow.setHpr(arrow, Vec3(0,0,90))
        dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        dumbbell.reparentTo(nodepath)

        return dumbbell

    def plotBox(self, nodepath, pos = None, x = 1.0, y = 1.0, z = 1.0, rgba=None):
        """
        plot a box to nodepath

        ## input:
        nodepath:
            defines which parent should the arrow be attached to
        pos:
            1-by-3 nparray or list, position of the sphere
        x,y,z:
            will be sent to _genBox
        rgba:
            1-by-3 nparray or list

        """

        if pos is None:
            pos = np.array([0,0,0])
        if rgba is None:
            rgba = np.array([1,1,1,1])

        boxnp = self.genBox(x, y, z)
        boxnp.setPos(pos[0], pos[1], pos[2])
        boxnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        boxnp.setTransparency(TransparencyAttrib.MAlpha)

        boxnp.reparentTo(nodepath)

    def plotLinesegs(self, nodepath, verts, thickness = 1.5, rgba=None, plotname="linesegs", headscale = 1):
        """
        plot a dumbbell to nodepath

        ## input:
        nodepath:
            defines which parent should the arrow be attached to
        verts:
            1-by-3 nparray or list, verts on the lineseg
        thickness:
            will be sent to _genArrow
        rgba:
            1-by-4 nparray or list


        """

        if rgba is None:
            rgba = np.array([1,1,1,1])

        linesegs = NodePath(plotname)
        for i in range(len(verts)-1):
            diff0 = verts[i][0]-verts[i+1][0]
            diff1 = verts[i][1]-verts[i+1][1]
            diff2 = verts[i][2]-verts[i+1][2]
            length = math.sqrt(diff0*diff0+diff1*diff1+diff2*diff2)
            dumbbell = self.genDumbbell(length=length, thickness=thickness, plotname=plotname, headscale=headscale)
            dumbbell.setPos(verts[i][0], verts[i][1], verts[i][2])
            dumbbell.lookAt(verts[i+1][0], verts[i+1][1], verts[i+1][2])
            dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
            dumbbell.setTransparency(TransparencyAttrib.MAlpha)
        # lookAt points y+ to epos, use the following command to point x+ to epos
        # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
        # arrow.setHpr(arrow, Vec3(0,0,90))
            dumbbell.reparentTo(linesegs)
        linesegs.reparentTo(nodepath)
        return linesegs

    def plotSphere(self, nodepath, pos=None, radius=None, rgba=None, plotname="sphere"):
        """
        plot a sphere to nodepath

        ## input:
        nodepath:
            defines which parent should the arrow be attached to
        pos:
            1-by-3 nparray or list, position of the sphere
        radius:
            will be sent to _genSphere
        rgba:
            1-by-3 nparray or list


        """

        if pos is None:
            pos = np.array([0, 0, 0])
        if rgba is None:
            rgba = np.array([1, 1, 1, 1])
        if radius is None:
            radius = 1

        spherend = self.genSphere(radius, plotname)
        spherend.setPos(pos[0], pos[1], pos[2])
        spherend.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        spherend.setTransparency(TransparencyAttrib.MAlpha)

        spherend.reparentTo(nodepath)

        return spherend

    def plotStick(self, nodepath, spos = None, epos = None, length = None, thickness = 1.5, rgba=None, plotname="dumbbell"):
        """
        plot a stick to nodepath

        ## input:
        nodepath:
            defines which parent should the arrow be attached to
        spos:
            1-by-3 nparray or list, starting position of the arrow
        epos:
            1-by-3 nparray or list, goal position of the arrow
        length:
            will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
        thickness:
            will be sent to _genArrow
        rgba:
            1-by-4 nparray or list


        """

        if spos is None:
            spos = np.array([0,0,0])
        if epos is None:
            epos = np.array([0,0,1])
        if length is None:
            length = np.linalg.norm(epos-spos)
        if rgba is None:
            rgba = np.array([1,1,1,1])

        stick = self.genStick(length, thickness, plotname)
        stick.setPos(spos[0], spos[1], spos[2])
        stick.lookAt(epos[0], epos[1], epos[2])
        # lookAt points y+ to epos, use the following command to point x+ to epos
        # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
        # arrow.setHpr(arrow, Vec3(0,0,90))
        stick.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        stick.setTransparency(TransparencyAttrib.MAlpha)
        stick.reparentTo(nodepath)
        return stick

    def plotAxis(self, nodepath, spos=Vec3(0,0,0), pandamat4=Mat4.identMat(), length=300, thickness = 10):
        """
        plot an axis to the scene, using self-defined arrows
        note: pos is in the coordiante sys of nodepath

        :param pandamat4: a panda3d LMatrix4f matrix
        :return: null


        """

        self.plotArrow(nodepath, spos, spos+pandamat4.getRow3(0), length=length, thickness=thickness, rgba=[1,0,0,1])
        self.plotArrow(nodepath, spos, spos+pandamat4.getRow3(1), length=length, thickness=thickness, rgba=[0,1,0,1])
        self.plotArrow(nodepath, spos, spos+pandamat4.getRow3(2), length=length, thickness=thickness, rgba=[0,0,1,1])

    def plotCircArrow(self, nodepath, axis = None, torqueportion = 0.0, center = None, radius = 5.0,
                      thickness = 1.5, rgba = None, discretization = 24, plotname="circarrow"):
        """
        Draw a circule arrow
        Especially used for visualization of torque
        the arraw starts from (axis cross [1,0,0])*radius+center
        if axis cross [1,0,0] equals [0,0,0], the arrow starts from (axis cross [0,1,0])*radius+center
        the portion of the circ arrow indicates the magnitude of t, maxt is set to torque/25.0
        if torque/25.0 is larger than 1.0, set it to 1.0

        :param nodepath:
        :param torque portion: measured torque over maximum value 0.0~1.0
        :param center: center of the circle
        :param radius: radius of the circle
        :param rgba: color of the arrow
        :param discretization: number sticks used
        :return:


        """

        if axis is None:
            axis = np.array([0.0,0.0,1.0])
        if center is None:
            center = np.array([0.0,0.0,0.0])
        if rgba is None:
            rgba = np.array([1.0,1.0,1.0,1.0])

        xaxis = np.array([1.0,0.0,0.0])
        yaxis = np.array([0.0,1.0,0.0])

        startingaxis = np.cross(axis, xaxis)
        if np.lingalg.norm(startingaxis) < .1:
            startingaxis = np.cross(axis, yaxis)
        startingaxis = startingaxis/np.linalg.norm(startingaxis)
        startingpos = startingaxis*radius+center

        cirarrnp = NodePath(plotname)
        discretizedangle = 360.0/discretization
        ndist = int(torqueportion*discretization)
        lastpos = startingpos
        if abs(ndist)>1:
            for i in range(1*np.sign(ndist), ndist, 1*np.sign(ndist)):
                nxtpos = center+np.dot(rm.rodrigues(axis, i*discretizedangle), startingaxis.T).T*radius
                self.plotStick(cirarrnp, spos = lastpos, epos = nxtpos, thickness = thickness, rgba = rgba, plotname = "circarrseg")
                lastpos = nxtpos
            nxtposend = center+np.dot(rm.rodrigues(axis, ndist*discretizedangle), startingaxis.T).T*radius

            self.plotArrow(cirarrnp, lastpos, nxtposend, length=1.0, thickness=thickness, rgba=rgba)

        cirarrnp.reparentTo(nodepath)

        return cirarrnp

    def plotPortionCircArrow(self, nodepath, axis = None, torqueportion = 0.0, center = None, radius = 5.0,
                      thickness = 1.5, rgba = None, discretization = 24, plotname="circarrow"):
        """
        Draw a circule arrow on a solid circle
        Especially used for visualization of torque
        the arraw starts from (axis cross [1,0,0])*radius+center
        if axis cross [1,0,0] equals [0,0,0], the arrow starts from (axis cross [0,1,0])*radius+center
        the portion of the circ arrow indicates the magnitude of t, maxt is set to torque/25.0
        if torque/25.0 is larger than 1.0, set it to 1.0

        :param nodepath:
        :param torque portion: measured torque over maximum value 0.0~1.0
        :param center: center of the circle
        :param radius: radius of the circle
        :param rgba: color of the arrow
        :param discretization: number sticks used
        :return:


        """

        if axis is None:
            axis = np.array([0.0,0.0,1.0])
        if center is None:
            center = np.array([0.0,0.0,0.0])
        else:
            center = np.array([center[0], center[1], center[2]])
        if rgba is None:
            rgba = np.array([1.0,1.0,1.0,1.0])

        xaxis = np.array([1.0,0.0,0.0])
        yaxis = np.array([0.0,1.0,0.0])

        startingaxis = np.cross(axis, xaxis)
        if np.linalg.norm(startingaxis) < .1:
            startingaxis = np.cross(axis, yaxis)
        startingaxis = startingaxis/np.linalg.norm(startingaxis)
        startingpos = startingaxis*radius+center

        cirarrnp = NodePath(plotname)
        discretizedangle = 360.0/discretization
        ndist = int(torqueportion*discretization)
        lastpos = startingpos
        if abs(ndist)>1:
            for i in range(1*np.sign(ndist), ndist, 1*np.sign(ndist)):
                nxtpos = center+np.dot(rm.rodrigues(axis, i*discretizedangle), startingaxis.T).T*radius
                self.plotStick(cirarrnp, spos = lastpos, epos = nxtpos, thickness = thickness, rgba = rgba, plotname = "circarrseg")
                lastpos = nxtpos
            nxtposend = center+np.dot(rm.rodrigues(axis, ndist*discretizedangle), startingaxis.T).T*radius

            self.plotArrow(cirarrnp, lastpos, nxtposend, length=1.0, thickness=thickness, rgba=rgba)

        cirarrnpbg = NodePath(plotname+"bg")
        discretizedangle = 360.0/discretization
        lastpos = startingpos
        for i in range(1, discretization+1):
            nxtpos = center+np.dot(rm.rodrigues(axis, i*discretizedangle), startingaxis.T).T*radius
            bgrgba = np.array([rgba[0], rgba[1], rgba[2], .1])
            self.plotStick(cirarrnp, spos = lastpos, epos = nxtpos, thickness = thickness*.9, rgba = bgrgba, plotname = "circarrseg")
            lastpos = nxtpos
        cirarrnpbg.reparentTo(cirarrnp)

        cirarrnp.reparentTo(nodepath)

        return cirarrnp

    def plotText(self, content = "None", rgba = np.array([1,0,0,1]), scale = 0.05,
                 pos = np.array([-0.4, 0, -0.4]), plotname="text"):
        text = TextNode(plotname)
        text.setText(content)
        text.setTextColor(rgba[0], rgba[1], rgba[2], rgba[3])
        arial = loader.loadFont('cmr12.egg')
        text.setFont(arial)
        textNodePath = aspect2d.attachNewNode(text)
        textNodePath.setScale(scale)
        textNodePath.setPos(pos[0], pos[1], pos[2])

        return textNodePath

def packpandageom_fn(vertices, facenormals, triangles, name=''):
    """
    package the vertices and triangles into a panda3d geom

    # in 2017, this function was deprecated
    # but, by only using vertex normals, the shading was smoothed.
    # sharp edges cannot be seem.
    # in 2018, I compared the number of vertices loaded by loader.loadModelf and
    # the one loaded by this function and packpandageom_vn
    # the code is as follows
            with open('loader.txt', 'wb') as fp:
            ur3base_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur3egg", "base.egg"))
            ur3base_model2  = loader.loadModel(ur3base_filepath)
            geomNodeCollection = ur3base_model2.findAllMatches('**/+GeomNode')
            for nodePath in geomNodeCollection:
                geomNode = nodePath.node()
                for i in range(geomNode.getNumGeoms()):
                    geom = geomNode.getGeom(i)
                    vdata = geom.getVertexData()
                    vertex = GeomVertexReader(vdata, 'vertex')
                    normal = GeomVertexReader(vdata, 'normal')
                    while not vertex.isAtEnd():
                        v = vertex.getData3f()
                        t = normal.getData3f()
                        fp.write("v = %s, t = %s\n" % (repr(v), repr(t)))
                    break
                break

        with open('selfloader.txt', 'wb') as fp2:
            ur3base_model  = pg.loadstlaspandanp_fn(ur3base_filepath)
            ur3base_filepath = os.path.join(this_dir, "ur3stl", "base.stl")
            geomNodeCollection = ur3base_model.findAllMatches('**/+GeomNode')
            for nodePath in geomNodeCollection:
                geomNode = nodePath.node()
                for i in range(geomNode.getNumGeoms()):
                    geom = geomNode.getGeom(i)
                    vdata = geom.getVertexData()
                    vertex = GeomVertexReader(vdata, 'vertex')
                    normal = GeomVertexReader(vdata, 'normal')
                    while not vertex.isAtEnd():
                        v = vertex.getData3f()
                        t = normal.getData3f()
                        fp2.write("v = %s, t = %s\n" % (repr(v), repr(t)))
                    break
                break
    # the results showed that the loader.loadmodel also repeated the vertices to
    # let each vertex have multiple normals
    # and get better rendering effects.
    # Thus, the function is reused
    # The negative point is the method costs much memory
    # I think it could be solved by developing an independent procedure
    # to analyze normals and faces beforehand


    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    facenormals:
        a n-by-3 nparray, each row is the normal of a face
    triangles:
        a n-by-3 nparray, each row is three idx to the vertices
    name:
        not as important

    ## output
    geom
        a Geom model which is ready to be added to a node


    """

    # vertformat = GeomVertexFormat.getV3()
    vertformat = GeomVertexFormat.getV3n3()
    # vertformat = GeomVertexFormat.getV3n3c4()
    vertexdata = GeomVertexData(name, vertformat, Geom.UHStatic)
    # vertexdata.setNumRows(triangles.shape[0]*3)
    vertwritter = GeomVertexWriter(vertexdata, 'vertex')
    normalwritter = GeomVertexWriter(vertexdata, 'normal')
    # colorwritter = GeomVertexWriter(vertexdata, 'color')
    primitive = GeomTriangles(Geom.UHStatic)
    for i, fvidx in enumerate(triangles):
        vert0 = vertices[fvidx[0],:]
        vert1 = vertices[fvidx[1],:]
        vert2 = vertices[fvidx[2],:]
        vertwritter.addData3f(vert0[0], vert0[1], vert0[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        vertwritter.addData3f(vert1[0], vert1[1], vert1[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        vertwritter.addData3f(vert2[0], vert2[1], vert2[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        primitive.addVertices(i*3, i*3+1, i*3+2)
    geom = Geom(vertexdata)
    geom.addPrimitive(primitive)

    return geom


def packpandanp_fn(vertices, facenormals, triangles, name=''):
    """
    package the vertices and triangles into a panda3d geom

    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    facenormals:
        a n-by-3 nparray, each row is the normal of a face
    triangles:
        a n-by-3 nparray, each row is three idx to the vertices
    name:
        not as important

    ## output
    pandanode
        a panda node


    """

    objgeom = packpandageom_fn(vertices, facenormals, triangles, name+'geom')
    geomnodeobj = GeomNode( name+'geomnode')
    geomnodeobj.addGeom(objgeom)
    pandanp = NodePath( name+'nodepath')
    pandanp.attachNewNode(geomnodeobj)

    return pandanp

def packpandageom_vn(vertices, vertnormals, triangles, name=''):
    """
    package the vertices and triangles into a panda3d geom

    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    vertnormals:
        a n-by-3 nparray, each row is the normal of a vertex
    triangles:
        a n-by-3 nparray, each row is three idx to the vertices
    name:
        not as important

    ## output
    geom
        a Geom model which is ready to be added to a node


    """

    vertformat = GeomVertexFormat.getV3n3()
    vertexdata = GeomVertexData(name, vertformat, Geom.UHStatic)
    vertwritter = GeomVertexWriter(vertexdata, 'vertex')
    normalwritter = GeomVertexWriter(vertexdata, 'normal')
    primitive = GeomTriangles(Geom.UHStatic)
    for i, vert in enumerate(vertices):
        vertwritter.addData3f(vert[0], vert[1], vert[2])
        normalwritter.addData3f(vertnormals[i,0], vertnormals[i,1], vertnormals[i,2])
    for triangle in triangles:
        primitive.addVertices(triangle[0], triangle[1], triangle[2])
    primitive.setShadeModel(GeomEnums.SM_uniform)
    geom = Geom(vertexdata)
    geom.addPrimitive(primitive)

    return geom

def packpandanp_vn(vertices, vertnormals, triangles, name=''):
    """
    package the vertices and triangles into a panda3d geom

    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    vertnormals:
        a n-by-3 nparray, each row is the normal of a vert
    triangles:
        a n-by-3 nparray, each row is three idx to the vertices
    name:
        not as important

    ## output
    pandanode
        a panda node


    """

    objgeom = packpandageom_vn(vertices, vertnormals, triangles, name+'geom')
    geomnodeobj = GeomNode('GeomNode')
    geomnodeobj.addGeom(objgeom)
    pandanp = NodePath( name+'nodepath')
    pandanp.attachNewNode(geomnodeobj)

    return pandanp

def packpandageompnts(vertices, colors=[], name=''):
    """
    package the vertices and triangles into a panda3d geom

    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    colors:
        a n-by-4 nparray, each row is a rgba
    name:
        not as important

    ## output
    geom
        a Geom model which is ready to be added to a node


    """

    vertformat = GeomVertexFormat.getV3c4()
    vertexdata = GeomVertexData(name, vertformat, Geom.UHStatic)
    vertwritter = GeomVertexWriter(vertexdata, 'vertex')
    colorwritter = GeomVertexWriter(vertexdata, 'color')
    primitive = GeomPoints(Geom.UHStatic)
    for i,vert in enumerate(vertices):
        vertwritter.addData3f(vert[0], vert[1], vert[2])
        if len(colors) == 0:
            # default
            colorwritter.addData4f(.2, .2, .2, 1)
        else:
            colorwritter.addData4f(colors[i][0], colors[i][1], colors[i][2], colors[i][3])
        primitive.addVertex(i)
    geom = Geom(vertexdata)
    geom.addPrimitive(primitive)

    return geom

def loadstlaspandanp_fn(objpath):
    """
    load stl objects into pandanp
    use face normals to pack

    ## output
    pandanode
        a panda node


    """

    objtrimesh = trimesh.load_mesh(objpath)
    objnp = packpandanp_fn(objtrimesh.vertices, objtrimesh.face_normals, objtrimesh.faces)

    return objnp

def loadstlaspandanp_vn(objpath):
    """
    load stl objects into pandanp
    use vertex normals to pack

    ## output
    pandanode
        a panda node


    """

    objtrimesh = trimesh.load_mesh(objpath)
    objnp = packpandanp_vn(objtrimesh.vertices, objtrimesh.vertex_normals, objtrimesh.faces)

    return objnp

def randomColorArray(ncolors=1, alpha = 1):
    """
    Generate an array of random colors
    if ncolor = 1, returns a 4-element list

    :param ncolors: the number of colors genrated
    :return: colorarray


    """

    if ncolors == 1:
        return [np.random.random(), np.random.random(), np.random.random(), alpha]
    colorarray = []
    for i in range(ncolors):
        colorarray.append([np.random.random(), np.random.random(), np.random.random(), alpha])
    return colorarray
#
# def _genArrow(length, thickness = 1.5):
#     """
#     Generate a arrow node for plot
#     This function should not be called explicitly
#
#     ## input
#     length:
#         length of the arrow
#     thickness:
#         thickness of the arrow, set to 0.005 as default
#
#     ## output
#
#     """
#
#     this_dir, this_filename = os.path.split(__file__)
#     cylinderpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cylinder.egg"))
#     conepath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cone.egg"))
#
#     arrow = NodePath("arrow")
#     arrowbody = loader.loadModel(cylinderpath)
#     arrowhead = loader.loadModel(conepath)
#     arrowbody.setPos(0,0,0)
#     arrowbody.setScale(thickness, length, thickness)
#     arrowbody.reparentTo(arrow)
#     arrowhead.setPos(arrow.getX(), length, arrow.getZ())
#     # set scale (consider relativitly)
#     arrowhead.setScale(thickness*2, thickness*4, thickness*2)
#     arrowhead.reparentTo(arrow)
#
#     return arrow
#
# def plotArrow(nodepath, spos = None, epos = None, length = None, thickness = 1.5, rgba=None):
#     """
#     plot an arrow to nodepath
#
#     ## input:
#     pandabase:
#         the panda direct.showbase.ShowBase object
#         will be sent to _genArrow
#     nodepath:
#         defines which parent should the arrow be attached to
#     spos:
#         1-by-3 nparray or list, starting position of the arrow
#     epos:
#         1-by-3 nparray or list, goal position of the arrow
#     length:
#         will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
#     thickness:
#         will be sent to _genArrow
#     rgba:
#         1-by-3 nparray or list
#

#     """
#
#     if spos is None:
#         spos = np.array([0,0,0])
#     if epos is None:
#         epos = np.array([0,0,1])
#     if length is None:
#         length = np.linalg.norm(epos-spos)
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#
#     arrow = _genArrow(length, thickness)
#     arrow.setPos(spos[0], spos[1], spos[2])
#     arrow.lookAt(epos[0], epos[1], epos[2])
#     # lookAt points y+ to epos, use the following command to point x+ to epos
#     # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
#     # arrow.setHpr(arrow, Vec3(0,0,90))
#     arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#
#     arrow.reparentTo(nodepath)
#
#     return arrow
#
# def _genDumbbell(length, thickness = 2, plotname="dumbbell", headscale = 2):
#     """
#     Generate a dumbbell node for plot
#     This function should not be called explicitly
#
#     ## input
#     length:
#         length of the dumbbell
#     thickness:
#         thickness of the dumbbell, set to 0.005 as default
#
#     ## output
#
#     """
#
#     this_dir, this_filename = os.path.split(__file__)
#     cylinderpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cylinder.egg"))
#     conepath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "sphere.egg"))
#
#     dumbbell = NodePath(plotname)
#     dumbbellbody = loader.loadModel(cylinderpath)
#     dumbbellhead = loader.loadModel(conepath)
#     dumbbellbody.setPos(0,0,0)
#     dumbbellbody.setScale(thickness, length, thickness)
#     dumbbellbody.reparentTo(dumbbell)
#     dumbbellhead0 = NodePath("dumbbellhead0")
#     dumbbellhead1 = NodePath("dumbbellhead1")
#     dumbbellhead0.setPos(dumbbellbody.getX(), length, dumbbellbody.getZ())
#     dumbbellhead1.setPos(dumbbellbody.getX(), dumbbellbody.getY(), dumbbellbody.getZ())
#     dumbbellhead.instanceTo(dumbbellhead0)
#     dumbbellhead.instanceTo(dumbbellhead1)
#     # set scale (consider relativitly)
#     dumbbellhead0.setScale(thickness*headscale, thickness*headscale, thickness*headscale)
#     dumbbellhead1.setScale(thickness*headscale, thickness*headscale, thickness*headscale)
#     dumbbellhead0.reparentTo(dumbbell)
#     dumbbellhead1.reparentTo(dumbbell)
#
#     return dumbbell
#
# def plotDumbbell(nodepath, spos = None, epos = None, length = None, thickness = 1.5, rgba=None, plotname="dumbbell", headscale = 2):
#     """
#     plot a dumbbell to nodepath
#
#     ## input:
#     nodepath:
#         defines which parent should the arrow be attached to
#     spos:
#         1-by-3 nparray or list, starting position of the arrow
#     epos:
#         1-by-3 nparray or list, goal position of the arrow
#     length:
#         will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
#     thickness:
#         will be sent to _genArrow
#     rgba:
#         1-by-4 nparray or list
#

#     """
#
#     if spos is None:
#         spos = np.array([0,0,0])
#     if epos is None:
#         epos = np.array([0,0,1])
#     if length is None:
#         length = np.linalg.norm(epos-spos)
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#
#     dumbbell = _genDumbbell(length, thickness, plotname, headscale)
#     dumbbell.setPos(spos[0], spos[1], spos[2])
#     dumbbell.lookAt(epos[0], epos[1], epos[2])
#     # lookAt points y+ to epos, use the following command to point x+ to epos
#     # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
#     # arrow.setHpr(arrow, Vec3(0,0,90))
#     dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#
#     dumbbell.reparentTo(nodepath)
#     return dumbbell
#
# def _genBox(x=1.0, y=1.0, z=1.0, plotname="box"):
#     """
#     Generate a box for plot
#     This function should not be called explicitly
#
#     ## input
#     x,y,z:
#         the thickness of the box along x, y, and z axis
#
#     ## output
#     box: pathnode
#

#     """
#
#     this_dir, this_filename = os.path.split(__file__)
#     boxpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "box.egg"))
#     box = loader.loadModel(boxpath)
#
#     boxnp = NodePath(plotname)
#     box.reparentTo(boxnp)
#     box.setPos(0, 0, 0)
#     box.setScale(x, y, z)
#
#     return boxnp
#
# def plotBox(nodepath, pos = None, x = 1.0, y = 1.0, z = 1.0, rgba=None):
#     """
#     plot a box to nodepath
#
#     ## input:
#     nodepath:
#         defines which parent should the arrow be attached to
#     pos:
#         1-by-3 nparray or list, position of the sphere
#     x,y,z:
#         will be sent to _genBox
#     rgba:
#         1-by-3 nparray or list
#

#     """
#
#     if pos is None:
#         pos = np.array([0,0,0])
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#
#     boxnp = _genBox(x, y, z)
#     boxnp.setPos(pos[0], pos[1], pos[2])
#     boxnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#     boxnp.setTransparency(TransparencyAttrib.MAlpha)
#
#     boxnp.reparentTo(nodepath)
#
# def plotLinesegs(nodepath, verts, thickness = 1.5, rgba=None, plotname="linesegs", headscale = 1):
#     """
#     plot a dumbbell to nodepath
#
#     ## input:
#     nodepath:
#         defines which parent should the arrow be attached to
#     verts:
#         1-by-3 nparray or list, verts on the lineseg
#     thickness:
#         will be sent to _genArrow
#     rgba:
#         1-by-4 nparray or list
#

#     """
#
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#
#     linesegs = NodePath(plotname)
#     for i in range(len(verts)-1):
#         diff0 = verts[i][0]-verts[i+1][0]
#         diff1 = verts[i][1]-verts[i+1][1]
#         diff2 = verts[i][2]-verts[i+1][2]
#         length = math.sqrt(diff0*diff0+diff1*diff1+diff2*diff2)
#         dumbbell = _genDumbbell(length, thickness, plotname, headscale=headscale)
#         dumbbell.setPos(verts[i][0], verts[i][1], verts[i][2])
#         dumbbell.lookAt(verts[i+1][0], verts[i+1][1], verts[i+1][2])
#         dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#         dumbbell.setTransparency(TransparencyAttrib.MAlpha)
#     # lookAt points y+ to epos, use the following command to point x+ to epos
#     # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
#     # arrow.setHpr(arrow, Vec3(0,0,90))
#
#         dumbbell.reparentTo(linesegs)
#
#     linesegs.reparentTo(nodepath)
#     return linesegs
#
#     boxnd.reparentTo(nodepath)
#
# def _genSphere(radius = None):
#     """
#     Generate a sphere for plot
#     This function should not be called explicitly
#
#     ## input
#     radius:
#         the radius of the sphere
#
#     ## output
#     sphere: pathnode
#

#     """
#
#     if radius is None:
#         radius = 0.05
#
#     this_dir, this_filename = os.path.split(__file__)
#     spherepath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "sphere.egg"))
#
#     spherepnd = NodePath("sphere")
#     spherend = loader.loadModel(spherepath)
#     spherend.setPos(0,0,0)
#     spherend.setScale(radius, radius, radius)
#     spherend.reparentTo(spherepnd)
#
#     return spherepnd
#
# def plotSphere(nodepath, pos = None, radius=None, rgba=None):
#     """
#     plot a sphere to nodepath
#
#     ## input:
#     nodepath:
#         defines which parent should the arrow be attached to
#     pos:
#         1-by-3 nparray or list, position of the sphere
#     radius:
#         will be sent to _genSphere
#     rgba:
#         1-by-3 nparray or list
#

#     """
#
#     if pos is None:
#         pos = np.array([0,0,0])
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#     if radius is None:
#         radius = 1
#
#     spherend = _genSphere(radius)
#     spherend.setPos(pos[0], pos[1], pos[2])
#     spherend.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#     spherend.setTransparency(TransparencyAttrib.MAlpha)
#
#     spherend.reparentTo(nodepath)
#
# def _genStick(length, thickness = 2, plotname="dumbbell"):
#     """
#     Generate a stick node for plot
#     This function should not be called explicitly
#
#     ## input
#     length:
#         length of the stick
#     thickness:
#         thickness of the stick, set to 0.005 as default
#
#     ## output
#
#     """
#
#     this_dir, this_filename = os.path.split(__file__)
#     cylinderpath = Filename.fromOsSpecific(os.path.join(this_dir, "geomprim", "cylinder.egg"))
#
#     stick = NodePath(plotname)
#     stickbody = loader.loadModel(cylinderpath)
#     stickbody.setPos(0,0,0)
#     stickbody.setScale(thickness, length, thickness)
#     stickbody.reparentTo(stick)
#
#     return stick
#
# def plotStick(nodepath, spos = None, epos = None, length = None, thickness = 1.5, rgba=None, plotname="dumbbell"):
#     """
#     plot a stick to nodepath
#
#     ## input:
#     nodepath:
#         defines which parent should the arrow be attached to
#     spos:
#         1-by-3 nparray or list, starting position of the arrow
#     epos:
#         1-by-3 nparray or list, goal position of the arrow
#     length:
#         will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
#     thickness:
#         will be sent to _genArrow
#     rgba:
#         1-by-4 nparray or list
#

#     """
#
#     if spos is None:
#         spos = np.array([0,0,0])
#     if epos is None:
#         epos = np.array([0,0,1])
#     if length is None:
#         length = np.linalg.norm(epos-spos)
#     if rgba is None:
#         rgba = np.array([1,1,1,1])
#
#     stick = _genStick(length, thickness, plotname)
#     stick.setPos(spos[0], spos[1], spos[2])
#     stick.lookAt(epos[0], epos[1], epos[2])
#     # lookAt points y+ to epos, use the following command to point x+ to epos
#     # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
#     # arrow.setHpr(arrow, Vec3(0,0,90))
#     stick.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
#
#     stick.reparentTo(nodepath)
#     return stick
#

def cvtMat3(npmat3):
    """
    convert numpy.2darray to LMatrix3f defined in Panda3d

    :param npmat3: a 3x3 numpy ndarray
    :return: a LMatrix3f object, see panda3d


    """
    return Mat3(npmat3[0, 0], npmat3[1, 0], npmat3[2, 0], \
                npmat3[0, 1], npmat3[1, 1], npmat3[2, 1], \
                npmat3[0, 2], npmat3[1, 2], npmat3[2, 2])

def npToMat4(npmat3, npvec3=np.array([0,0,0])):
    """
    # updated from cvtMat4
    convert numpy.2darray to LMatrix4 defined in Panda3d

    :param npmat3: a 3x3 numpy ndarray
    :param npvec3: a 1x3 numpy ndarray
    :return: a LMatrix3f object, see panda3d


    """
    return Mat4(npmat3[0, 0], npmat3[1, 0], npmat3[2, 0], 0, \
                npmat3[0, 1], npmat3[1, 1], npmat3[2, 1], 0, \
                npmat3[0, 2], npmat3[1, 2], npmat3[2, 2], 0, \
                npvec3[0], npvec3[1], npvec3[2], 1)

def cvtMat4(npmat3, npvec3=np.array([0,0,0])):
    """
    # use npToMat4 instead
    convert numpy.2darray to LMatrix4 defined in Panda3d

    :param npmat3: a 3x3 numpy ndarray
    :param npvec3: a 1x3 numpy ndarray
    :return: a LMatrix3f object, see panda3d


    """
    return Mat4(npmat3[0, 0], npmat3[1, 0], npmat3[2, 0], 0, \
                npmat3[0, 1], npmat3[1, 1], npmat3[2, 1], 0, \
                npmat3[0, 2], npmat3[1, 2], npmat3[2, 2], 0, \
                npvec3[0], npvec3[1], npvec3[2], 1)

def cvtMat4np4(npmat4):
    """
    convert numpy.2darray to LMatrix4 defined in Panda3d

    :param npmat4: a 4x4 numpy ndarray
    :return: a LMatrix4f object, see panda3d


    """
    return cvtMat4(npmat4[:3,:3], npmat4[:3,3])

def mat4ToStr(pdmat4):
    """
    convert a mat4 matrix to a string like e00, e01, e02, ...

    :param pdmat4:
    :return: a string


    """

    row0 = pdmat4.getRow(0)
    row1 = pdmat4.getRow(1)
    row2 = pdmat4.getRow(2)

    return ','.join(row0)+','+','.join(row1)+','+','.join(row2)

def mat3ToNp(pdmat3):
    """
    convert a mat3 matrix to a numpy 2darray...

    :param pdmat3:
    :return: numpy 2darray


    """

    row0 = pdmat3.getRow(0)
    row1 = pdmat3.getRow(1)
    row2 = pdmat3.getRow(2)

    return np.array([[row0[0], row1[0], row2[0]], [row0[1], row1[1], row2[1]], [row0[2], row1[2], row2[2]]])

def mat4ToNp(pdmat4):
    """
    convert a mat4 matrix to a numpy 2darray...

    :param pdmat4
    :return: numpy 2darray


    """

    # TODO translation should be vertical?

    row0 = pdmat4.getRow(0)
    row1 = pdmat4.getRow(1)
    row2 = pdmat4.getRow(2)
    row3 = pdmat4.getRow(3)

    return np.array([[row0[0], row1[0], row2[0], row3[0]], [row0[1], row1[1], row2[1], row3[2]],
                     [row0[2], row1[2], row2[2], row3[2]], [row0[3], row1[3], row2[3], row3[3]]])

def v3ToNp(pdv3):
    """
    convert vbase3 to a numpy array...

    :param pdmat3:
    :return: numpy 2darray


    """

    return np.array([pdv3[0], pdv3[1], pdv3[2]])

def npToV3(npv3):
    """
    convert a numpy array to Panda3d V3...

    :param npv3:
    :return: panda3d vec3

    """

    return Vec3(npv3[0], npv3[1], npv3[2])

def makelsnodepath(linesegs,thickness=1, rgbacolor=[1,1,1,1]):
    """
    create linesegs pathnode

    :param linesegs: [[pnt0, pn1], [pn0, pnt1], ...]
    :param thickness:
    :return: a panda3d pathnode


    """

    # Create a set of line segments
    ls = LineSegs()
    ls.setThickness(thickness)

    for p0p1tuple in linesegs:
        pnt00, pnt01, pnt02 = p0p1tuple[0]
        pnt10, pnt11, pnt12 = p0p1tuple[1]
        ls.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])
        ls.moveTo(pnt00, pnt01, pnt02)
        ls.drawTo(pnt10, pnt11, pnt12)

    # Create and return a node with the segments
    lsnp = NodePath(ls.create())
    lsnp.setTransparency(TransparencyAttrib.MAlpha)
    return lsnp


def facetboundary(objtrimesh, facet, facetcenter, facetnormal):
    """
    compute a boundary polygon for facet
    assumptions:
    1. there is only one boundary
    2. the facet is convex

    :param objtrimesh: a datatype defined in trimesh
    :param facet: a data type defined in trimesh
    :param facetcenter and facetnormal used to compute the transform, see trimesh.geometry.plane_transform
    :return: [a list of 3d points, a shapely polygon, facetmat4 (the matrix that cvt the facet to 2d 4x4)]


    """

    facetp = None

    # use -facetnormal to let the it face downward
    facetmat4 = trigeom.plane_transform(facetcenter, -facetnormal)

    for i, faceidx in enumerate(facet):
        vert0 = objtrimesh.vertices[objtrimesh.faces[faceidx][0]]
        vert1 = objtrimesh.vertices[objtrimesh.faces[faceidx][1]]
        vert2 = objtrimesh.vertices[objtrimesh.faces[faceidx][2]]
        vert0p = rm.transformmat4(facetmat4, vert0)
        vert1p = rm.transformmat4(facetmat4, vert1)
        vert2p = rm.transformmat4(facetmat4, vert2)
        facep = Polygon([vert0p[:2], vert1p[:2], vert2p[:2]])
        if facetp is None:
            facetp = facep
        else:
            facetp = facetp.union(facep)

    verts2d = list(facetp.exterior.coords)
    verts3d = []
    for vert2d in verts2d:
        vert3d = rm.transformmat4(rm.homoinverse(facetmat4), np.array([vert2d[0], vert2d[1], 0]))[:3]
        verts3d.append(vert3d)

    return [verts3d, verts2d, facetmat4]

def genObjmnp(objpath, objname = 'obj', color = Vec4(1,0,0,1)):
    """
    gen objmnp

    :param objpath:
    :return:
    """

    objtrimesh = trimesh.load_mesh(objpath)
    geom = packpandageom_fn(objtrimesh.vertices,
                         objtrimesh.face_normals,
                         objtrimesh.faces)

    node = GeomNode(objname)
    node.addGeom(geom)
    objmnp = NodePath(objname)
    objmnp.attachNewNode(node)
    objmnp.setColor(color)
    objmnp.setTransparency(TransparencyAttrib.MAlpha)

    return objmnp

def genPntsnp(verts, colors = [], pntsize = 1):
    """
    gen objmnp

    :param objpath:
    :return:
    """

    geom = packpandageompnts(verts, colors)
    node = GeomNode('pnts')
    node.addGeom(geom)
    objmnp = NodePath('pnts')
    objmnp.attachNewNode(node)
    objmnp.setRenderMode(RenderModeAttrib.MPoint, pntsize)

    return objmnp

def genSurfacenp(verts, tris, color = Vec4(1,0,0,1)):
    """

    :param verts: np.array([[v00, v01, v02], [v10, v11, v12], ...]
    :param tris: np.array([[ti00, ti01, ti02], [ti10, ti11, ti12], ...]
    :param color: Vec4 rgba
    :return:


    """

    # gen vert normals
    vertnormals = np.zeros((len(verts), 3))
    for tri in tris:
        vert0 = verts[tri[0], :]
        vert1 = verts[tri[1], :]
        vert2 = verts[tri[2], :]
        facenormal = np.cross(vert2-vert1, vert0-vert1)
        vertnormals[tri[0], :] = vertnormals[tri[0]] + facenormal
        vertnormals[tri[1], :] = vertnormals[tri[1]] + facenormal
        vertnormals[tri[2], :] = vertnormals[tri[2]] + facenormal
        # vertnormals[tri[0], :] = facenormal
        # vertnormals[tri[1], :] = facenormal
        # vertnormals[tri[2], :] = facenormal
    for i in range(0,len(vertnormals)):
        vertnormals[i, :] = vertnormals[i,:]/np.linalg.norm(vertnormals[i,:])

    geom = packpandageom_vertnormals(verts,
                                   vertnormals,
                                   tris)
    node = GeomNode('surface')
    node.addGeom(geom)
    surfacemnp = NodePath('surface')
    surfacemnp.attachNewNode(node)
    surfacemnp.setColor(color)
    surfacemnp.setTransparency(TransparencyAttrib.MAlpha)
    surfacemnp.setTwoSided(True)

    return surfacemnp

def genPolygonsnp(verts, colors = [], thickness = 2.0):
    """
    gen objmnp

    :param objpath:
    :return:
    """

    segs = LineSegs()
    segs.setThickness(thickness)
    if len(colors) == 0:
        segs.setColor(Vec4(.2, .2, .2, 1))
    else:
        segs.setColor(colors[0], colors[1], colors[2], colors[3])
    for i in range(len(verts)-1):
        segs.moveTo(verts[i][0], verts[i][1], verts[i][2])
        segs.drawTo(verts[i+1][0], verts[i+1][1], verts[i+1][2])

    objmnp = NodePath('polygons')
    objmnp.attachNewNode(segs.create())
    objmnp.setTransparency(TransparencyAttrib.MAlpha)

    return objmnp

def genLinesegsnp(verts, colors = [], thickness = 2.0):
    """
    gen objmnp

    :param objpath:
    :return:
    """

    segs = LineSegs()
    segs.setThickness(thickness)
    if len(colors) == 0:
        segs.setColor(Vec4(.2, .2, .2, 1))
    else:
        segs.setColor(colors[0], colors[1], colors[2], colors[3])
    for i in range(len(verts)-1):
        segs.moveTo(verts[i][0], verts[i][1], verts[i][2])
        segs.drawTo(verts[i+1][0], verts[i+1][1], verts[i+1][2])

    objmnp = NodePath('linesegs')
    objmnp.attachNewNode(segs.create())
    objmnp.setTransparency(TransparencyAttrib.MAlpha)

    return objmnp

def genRotmat4(icolevel = 1, angles = [0, 45, 90, 135, 180, 255, 270, 315]):
    """
    generate panda3d rotmat4 using icospheres and rotationaangle each origin-vertex vector of the icosphere

    :param icolevel:
    :param angles:
    :return:

    """

    mat4list = []
    icos = trimesh.creation.icosphere(icolevel)
    for vert in icos.vertices:
        xvec = Vec3(vert[0], vert[1], vert[2])
        yvec = xvec.cross(Vec3(0,0,1))
        zvec = xvec.cross(yvec)
        newobjmat3 = Mat3()
        newobjmat3.setRow(0, xvec)
        newobjmat3.setRow(1, yvec)
        newobjmat3.setRow(2, zvec)
        newobjmat4 = Mat4(newobjmat3)
        for angle in angles:
            tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(0))
            tmppandamat4 = newobjmat4 * tmppandamat4
            mat4list.append(tmppandamat4)

    return mat4list

def trimesh2Panda(trimesh, name = "auto"):
    """
    cvt trimesh models to panda models

    :param trimesh:
    :return:


    """

    return packpandanp_fn(trimesh.vertices, trimesh.face_normals, trimesh.faces)

if __name__=="__main__":

    # show in panda3d
    import random
    import math
    from panda3d.core import *
    import pandaplotutils.pandactrl as pandactrl

    base = pandactrl.World(camp=[0,0,3000], lookatp=[0,0,0])

    verts = []
    for i in range(-500, 500, 5):
        for j in range(-500, 500,5 ):
            verts.append([i,j,random.gauss(0, math.sqrt(i*i+j*j))/10])
    verts = np.array(verts)
    pntsnp = genPntsnp(verts, pntsize = 10)

    base.run()