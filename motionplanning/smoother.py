import numpy as np
import random
import math

class Smoother(object):
    
    def __init__(self):
        pass

    def __linecdchecker(self, start, goal):
        """

        :param start:
        :param goal:
        :return:


        """

        nps = np.array(start).reshape(-1,1)
        npg = np.array(goal).reshape(-1,1)
        nele = math.ceil((abs(npg-nps)/self.__expanddis).max())
        ratio = np.linspace(0, 1, nele, endpoint=False)

        jointslist = (nps+(npg-nps)*ratio).T.tolist()
        for joints in jointslist:
            iscollided = self.__iscollidedcallback(joints, self.__obstaclelist, self.__robot, self.__cdchecker)
            if iscollided:
                return False, []
        return True, jointslist

    def pathsmoothing(self, path, planner, maxiter):
        """
        the path and planner are necessary parameters
        the following member variables of planner will be used for smoothing
        1. jointlimits
        2. iscollidedcallback
        3. cdchecker
        4. robot
        5. expanddis
        6. obstaclelist

        :param path:
        :param planner:
        :return:


        """

        self.__jointlimits = planner.jointlimits
        self.__iscollidedcallback = planner.iscollidedcallback
        self.__cdchecker = planner.cdchecker
        self.__robot = planner.robot
        self.__expanddis = planner.expanddis
        self.__obstaclelist = planner.obstaclelist

        pathlength = len(path)
        if pathlength <= 3:
            return path
        for i in range(maxiter):
            pickpoint0 = random.randint(0, pathlength-3)
            pickpoint1 = random.randint(pickpoint0+1, pathlength-1)
            result, addpath = self.__linecdchecker(path[pickpoint0], path[pickpoint1])
            if result:
                path = path[:pickpoint0]+addpath+path[pickpoint1:]
                pathlength = len(path)

        return path
