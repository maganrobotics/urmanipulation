#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d

# rrt is much faster than rrt connect


"""

import random
import copy
import math
import numpy as np

class RRT(object):

    def __init__(self, start, goal, iscollidedfunc, jointlimits, expanddis=.5, goalsamplerate=10, maxiter=5000,
                 robot = None, cdchecker = None):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param iscollidedfunc: a function returns whether collided or not
        :param jointlimits: [[join0low, joint0high], [joint1low, joint1high], ...]
        :param expandDis: how much to expand along the vector randpoint - nearestnode
        :param goalSampleRate: bias to set randpoint to be goal
        :param maxIter:
        :param the last three are for robotsim robots


        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__iscollidedcallback = iscollidedfunc
        self.__jointlimits = jointlimits
        self.__expanddis = expanddis
        self.__goalsamplerate = goalsamplerate
        self.__maxiter = maxiter

        self.__robot = robot
        self.__cdchecker = cdchecker

        self.__nodelist = [Node(self.__start)]
        self.__obstaclelist = []

    @property
    def start(self):
        # read-only property
        return self.__start

    @property
    def end(self):
        # read-only property
        return self.__end

    @property
    def nodelist(self):
        # read-only property
        return self.__nodelist

    @property
    def jointlimits(self):
        # read-only property
        return self.__jointlimits

    @property
    def iscollidedcallback(self):
        # read-only property
        return self.__iscollidedcallback

    @property
    def cdchecker(self):
        # read-only property
        return self.__cdchecker

    @property
    def robot(self):
        # read-only property
        return self.__robot

    @property
    def expanddis(self):
        # read-only property
        return self.__expanddis

    @property
    def obstaclelist(self):
        # read-only property
        return self.__obstaclelist

    def planning(self, obstaclelist=[], animation=False):
        """
        Pathplanning

        animation: flag for animation on or off

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        self.__obstaclelist = obstaclelist
        itercount = 0

        # one sampled point is defined as [point, iscollided]
        sampledpoints = []
        self.__nodelist = [Node(self.__start)]

        while True:

            if itercount > self.__maxiter:
                print("failed to find a path")
                break

            # Random Sampling
            randpoint = []
            if random.randint(0, 100) > self.__goalsamplerate:
                for i,jntrng in enumerate(self.__jointlimits):
                    randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                randpoint = np.asarray(randpoint)
            else:
                randpoint = copy.deepcopy(self.__end)

            # Find nearest node
            nind = self.getNearestListIndex(self.__nodelist, randpoint)
            vec = randpoint-self.__nodelist[nind].point
            vec = vec/np.linalg.norm(vec)

            # expand tree
            nearestnode = self.__nodelist[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

            iscollided = self.__iscollidedcallback(newnode.point, obstaclelist,
                                                   self.__robot, self.__cdchecker)
            if iscollided:
                sampledpoints.append([newnode.point, True])
                continue

            sampledpoints.append([newnode.point, False])
            self.__nodelist.append(newnode)

            # check goal
            d = np.linalg.norm(newnode.point - self.__end)
            if d <= self.__expanddis:
                print("reaching the goal")
                break

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

            itercount += 1

        path = [self.__end.tolist()]
        lastindex = len(self.__nodelist) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelist[lastindex].parent is not None:
            node = self.__nodelist[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        return [path, sampledpoints]

    def planningcallback(self, obstaclelist = []):
        """
        decomposed planning
        designed for callbacks of panda3dplot

        :param obstaclelist:
        :return:


        """

        # Random Sampling
        randpoint = []
        if random.randint(0, 100) > self.__goalsamplerate:
            for i,jntrng in enumerate(self.__jointlimits):
                randpoint.append(random.uniform(jntrng[0], jntrng[1]))
            randpoint = np.asarray(randpoint)
        else:
            randpoint = copy.deepcopy(self.__end)

        # Find nearest node
        nind = self.getNearestListIndex(self.__nodelist, randpoint)
        vec = randpoint-self.__nodelist[nind].point
        dvec = float(np.linalg.norm(vec))
        vec = vec/dvec

        # expand tree
        nearestnode = self.__nodelist[nind]
        newnode = copy.deepcopy(nearestnode)
        newnode.point += self.__expanddis * vec
        newnode.parent = nind
        self.newpoint = newnode.point

        iscollided = self.__iscollidedcallback(newnode.point, obstaclelist, self.__robot, self.__cdchecker)
        if iscollided:
            return "collided"

        self.__nodelist.append(newnode)

        # check goal
        d = np.linalg.norm(newnode.point - self.__end)
        if d <= self.__expanddis:
            print("reaching the goal")
            return "done"

        return "continue"

    def getpathcallback(self):
        """
        decomposed planning
        designed for callbacks of panda3dplot

        :param obstaclelist:
        :return:



        """

        path = [self.__end.tolist()]
        lastindex = len(self.__nodelist) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelist[lastindex].parent is not None:
            node = self.__nodelist[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())

        return path

    def getNearestListIndex(self, nodelist, randpoint):
        dlist = [np.linalg.norm(randpoint-node.point) for node in nodelist]
        minind = dlist.index(min(dlist))
        return minind

class Node():
    """
    RRT Node
    """

    def __init__(self, point):
        """

        :param point: nd point, numpyarray


        """

        self.point = point
        self.parent = None

def iscollidedfunc(point, obstaclelist, robot = None, cdchecker = None):
    for (obpos, size) in obstaclelist:
        d = np.linalg.norm(np.asarray(obpos) - point)
        if d <= size/2.0:
            return True  # collision

    return False  # safe

def drawwspace(planner, obstaclelist, randconfiguration=None, newconfiguration = None, newconfmark = '^r'):
    """
    Draw Graph
    """
    import matplotlib.pyplot as plt
    plt.clf()
    if randconfiguration is not None:
        plt.plot(randconfiguration[0], randconfiguration[1], "^k")
    if newconfiguration is not None:
        plt.plot(newconfiguration[0], newconfiguration[1], newconfmark)
    for node in planner.nodelist:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelist[node.parent].point[0]],
                     [node.point[1], planner.nodelist[node.parent].point[1]], '-g')
    for (point, size) in obstaclelist:
        plt.plot([point[0]], [point[1]], "ok", ms=size*20)
    plt.plot(planner.start[0], planner.start[1], "xr")
    plt.plot(planner.end[0], planner.end[1], "xr")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.0001)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import motionplanning.smoother as sm

    plt.pause(5)
    smoother = sm.Smoother()

    # ====Search Path with RRT====
    obstaclelist = [
        ((5, 5), 3),
        ((3, 6), 3),
        ((3, 8), 3),
        ((3, 10), 3),
        ((7, 5), 3),
        ((9, 5), 3)
    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0.0, 0.0], goal=[5.0, 10.0], iscollidedfunc = iscollidedfunc,
              jointlimits = [[-2.0, 15.0], [-2.0, 15.0]], expanddis=1)

    import time
    tic = time.perf_counter()
    path, sampledpoints = rrt.planning(obstaclelist=obstaclelist, animation=True)
    toc = time.perf_counter()
    print (toc-tic)

    # Draw final path
    drawwspace(rrt, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-r')
    pathsm = smoother.pathsmoothing(path, rrt, 30)
    plt.plot([point[0] for point in pathsm], [point[1] for point in pathsm], '-k')
    plt.grid(True)
    # plt.pause(0.001)  # Need for Mac
    plt.show()