import trimesh
from panda3d.core import Mat4
import pandaplotutils.pandageom as pg

class FloatingPoses(object):

    def __init__(self, objpath0, objpath1):
        """
        load two objects

        :param objpath0:
        :param objpath1:

        author: weiwei
        date: 20180613
        """

        self.objtrimesh0 = trimesh.load_mesh(objpath0)
        self.objnp0 = pg.packpandanp_fn(self.objtrimesh0.vertices, self.objtrimesh0.face_normals, self.objtrimesh0.faces)
        self.objtrimesh1 = trimesh.load_mesh(objpath1)
        self.objnp1 = pg.packpandanp_fn(self.objtrimesh1.vertices, self.objtrimesh1.face_normals, self.objtrimesh1.faces)
        self.__floatingposemat40 = []
        self.__floatingposemat41 = []

    def genPandaRotmat4(self, icolevel=1, angles=[0, 45, 90, 135, 180, 225, 270, 315]):
        """
        generate panda3d rotmat4 using icospheres and rotationaangle each origin-vertex vector of the icosphere

        :param icolevel, the default value 1 = 42vertices
        :param angles, 8 directions by default
        :return: a list of [pandarotmat4, ...] size of the inner list is size of the angles

        author: weiwei
        date: 20170221, tsukuba
        """

        self.icos = trimesh.creation.icosphere(icolevel)

        self.__mat40list = []
        initmat40 = self.objnp0.getMat()
        for vert in self.icos.vertices:
            self.__mat40list.append([])
            self.objnp0.lookAt(vert[0], vert[1], vert[2])
            ytozmat4 = Mat4.rotateMat(-90, self.objnp0.getMat().getRow3(0))
            newobjmat4 = self.objnp0.getMat() * ytozmat4
            for angle in angles:
                tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(2))
                tmppandamat4 = newobjmat4 * tmppandamat4
                self.__mat40list[-1].append(tmppandamat4)
            self.objnp0.setMat(initmat40)
        self.__floatingposemat40 = [e for l in self.__mat40list for e in l]

        self.__mat41list = []
        initmat41 = self.objnp1.getMat()
        for vert in self.icos.vertices:
            self.__mat41list.append([])
            self.objnp1.lookAt(vert[0], vert[1], vert[2])
            ytozmat4 = Mat4.rotateMat(-90, self.objnp1.getMat().getRow3(0))
            newobjmat4 = self.objnp1.getMat() * ytozmat4
            for angle in angles:
                tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(2))
                tmppandamat4 = newobjmat4 * tmppandamat4
                self.__mat41list[-1].append(tmppandamat4)
            self.objnp1.setMat(initmat41)
        self.__floatingposemat41 = [e for l in self.__mat41list for e in l]

        return self.__floatingposemat40, self.__floatingposemat41