from rtree import index
import os
import numpy as np

class RtreeKD(object):
    """
    12D rtree
    """

    def __init__(self, dimension, savefilename = ''):
        p = index.Property()
        p.dimension = dimension
        p.storage= index.RT_Memory
        p.idx_extension = "idx"
        p.dat_extension = "dat"
        # delete previously saved idx files
        # it means two rtrees should not be used at the same time
        dir = os.path.dirname(__file__)+"/tmp/"
        if os.path.isfile(dir+savefilename+str(p.dimension)+"d_index."+p.idx_extension):
            os.remove(dir+savefilename+str(p.dimension)+"d_index."+p.idx_extension)
        if os.path.isfile(dir+savefilename+str(p.dimension)+"d_index."+p.dat_extension):
            os.remove(dir+savefilename+str(p.dimension)+"d_index."+p.dat_extension)
        self.__idxkd = index.Index(dir+savefilename+str(p.dimension)+"d_index", properties = p)
        self.__npoints = 0
        self.__dimension = dimension

    def insert(self, point):
        """
        The dimension of a point must be equal to dimension of the tree

        :param point: a kd list
        :return:

        author: weiwei
        date: 20180520
        """

        if isinstance(point, np.ndarray):
            point = point.tolist()
        self.__idxkd.insert(self.__npoints, point+point, {'id': self.__npoints, 'point': point})
        self.__npoints += 1

    def nearest(self, point):
        """
        The dimension of a point must be equal to dimension of the tree

        :param point: a kd list
        :return:

        author: weiwei
        date: 20180520
        """

        if isinstance(point, np.ndarray):
            point = point.tolist()
        nearestvalue = list(self.__idxkd.nearest(point+point, 1, objects='raw'))[0]
        nid = nearestvalue['id']
        point = nearestvalue['point']
        return nid, point

if __name__ == '__main__':
    rt = RtreeKD(6)
    points0 = [0,0,0,0,0,0]
    points1 = [1,1,0,0,0,0]
    points2 = [1,1,0,0,0,0]
    # rt = RtreeKD(3)
    # points0 = [0,0,0]
    # points1 = [1,1,1]
    # points2 = [0,1,0]
    rt.insert(points0)
    # rt.insert(points1)
    print(rt.nearest(points2)) #rt.nearest(points2)