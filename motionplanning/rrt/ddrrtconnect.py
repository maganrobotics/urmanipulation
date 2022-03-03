#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d


"""

import gc
import random
import copy
import math
import numpy as np

class DDRRTConnect(object):

    def __init__(self, start, goal, iscollidedfunc, jointlimits, expanddis=.5,
                 starttreesamplerate=10, endtreesamplerate=100, maxiter=5000,
                 robot = None, cdchecker = None):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param iscollidedfunc: a function returns whether collided or not
        :param jointlimits: [[join0low, joint0high], [joint1low, joint1high], ...]
        :param expandDis: how much to expand along the vector randpoint - nearestnode
        :param starttreesamplerate: bias to set randpoint to be goal
        :param endtreesamplerate: bias to set randpoint to be start
        :param maxIter:

        :param the last three parameters are for robotsim robots


        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__iscollidedcallback = iscollidedfunc
        self.__jointlimits = jointlimits
        self.__expanddis = expanddis
        self.__starttreesamplerate = starttreesamplerate
        self.__endtreesamplerate = endtreesamplerate
        self.__maxiter = maxiter

        self.__robot = robot
        self.__cdchecker = cdchecker

        self.__nodeliststart = []
        self.__nodelistend = []
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
    def nodeliststart(self):
        # read-only property
        return self.__nodeliststart

    @property
    def nodelistend(self):
        # read-only property
        return self.__nodelistend

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

        # one sampled point: [point, iscollided]
        sampledpoints = []

        self.__nodeliststart = [Node(self.__start)]
        self.__nodelistend = [Node(self.__end)]

        starttreegoal = self.__end
        endtreegoal = self.__start
        while True:

            if itercount > self.__maxiter:
                print( "failed to find a path")
                break

            # if self.__starttreesamplerate < 80:
            #     self.__starttreesamplerate += itercount/float(self.__maxiter)*10
            # else:
            #     self.__starttreesamplerate = 80
            # print self.__starttreesamplerate

            # Random Sampling
            randpoint = []
            vec = []
            nind = 0
            dvec = 1
            while True:
                randpoint = []
                if random.randint(0, 100) > self.__starttreesamplerate:
                    for i,jntrng in enumerate(self.__jointlimits):
                        randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                    randpoint = np.asarray(randpoint)
                else:
                    randpoint = copy.deepcopy(starttreegoal)

                # Find nearest node
                nind = self.getNearestListIndex(self.__nodeliststart, randpoint)
                vec = randpoint-self.__nodeliststart[nind].point
                dvec = np.linalg.norm(vec)
                if dvec < self.__nodeliststart[nind].radius:
                    break
            vec = vec/dvec

            # expand tree
            nearestnode = self.__nodeliststart[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind
            newnode.radius = float('inf')

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

            if self.__iscollidedcallback(newnode.point, obstaclelist,
                                         self.__robot, self.__cdchecker):
                self.__nodeliststart[nind].radius = 3*math.sqrt((self.__expanddis**2)*self.__start.size)
                sampledpoints.append([newnode.point, True])
                bswap = False
                # if collided, try the other tree
                while True:
                    randpoint = []
                    vec = []
                    nind = 0
                    dvec = 1
                    while True:
                        randpoint = []
                        if random.randint(0, 100) > self.__endtreesamplerate:
                            for i, jntrng in enumerate(self.__jointlimits):
                                randpoint.append(random.uniform(jntrng[0], jntrng[1]))
                            randpoint = np.asarray(randpoint)
                        else:
                            randpoint = copy.deepcopy(endtreegoal)

                        # Find nearest node
                        nind = self.getNearestListIndex(self.__nodelistend, randpoint)
                        vec = randpoint - self.__nodelistend[nind].point
                        dvec = np.linalg.norm(vec)
                        if dvec < self.__nodelistend[nind].radius:
                            break
                    vec = vec / dvec

                    # expand tree
                    nearestnode = self.__nodelistend[nind]
                    newnode = copy.deepcopy(nearestnode)
                    newnode.point += self.__expanddis * vec
                    newnode.parent = nind
                    newnode.radius = float('inf')

                    if animation:
                        drawwspace(self, obstaclelist, randpoint, newnode.point, '^r')

                    if self.__iscollidedcallback(newnode.point, obstaclelist,
                                                 self.__robot, self.__cdchecker):
                        sampledpoints.append([newnode.point, True])
                        bswap = True
                        break

                    sampledpoints.append([newnode.point, False])
                    self.__nodelistend.append(newnode)
                    starttreegoal = newnode.point# check goal

                    d = np.linalg.norm(newnode.point - endtreegoal)
                    if d <= self.__expanddis:
                        # print("reaching the goal")
                        bswap = False
                        break

                    if animation:
                        drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

                if bswap:
                    continue
                else:
                    break

            sampledpoints.append([newnode.point, False])
            self.__nodeliststart.append(newnode)
            endtreegoal = newnode.point

            # check goal
            d = np.linalg.norm(newnode.point - starttreegoal)
            if d <= self.__expanddis:
                # print("reaching the goal")
                break

            if animation:
                drawwspace(self, obstaclelist, randpoint, newnode.point, '^g')

            itercount += 1

        path = []
        lastindex = len(self.__nodelistend) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelistend[lastindex].parent is not None:
            node = self.__nodelistend[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__end.tolist())
        path = path[::-1]
        lastindex = len(self.__nodeliststart) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodeliststart[lastindex].parent is not None:
            node = self.__nodeliststart[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())
        path = path[::-1]

        return [path, sampledpoints]

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

        radius is added for dynamic domain
        the algorithm follows http://msl.cs.uiuc.edu/~lavalle/papers/YerJaiSimLav05.pdf


        """

        self.point = point
        self.parent = None
        self.radius = float('inf')


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
    for node in planner.nodeliststart:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-g')
        if node.radius < float('inf'):
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-r')
    for node in planner.nodelistend:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelistend[node.parent].point[0]],
                     [node.point[1], planner.nodelistend[node.parent].point[1]], '-b')
        if node.radius < float('inf'):
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-r')
    for (point, size) in obstaclelist:
        plt.plot([point[0]], [point[1]], "ok", ms=size*20)
    plt.plot(planner.start[0], planner.start[1], "xr")
    plt.plot(planner.end[0], planner.end[1], "xr")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)

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

    rrtc = DDRRTConnect(start=[0.0, 0.0], goal=[5.0, 10.0], iscollidedfunc = iscollidedfunc,
              jointlimits = [[-2.0, 15.0], [-2.0, 15.0]])
    path, sampledpoints = rrtc.planning(obstaclelist=obstaclelist, animation=False)

    # Draw final path
    drawwspace(rrtc, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-k')
    pathsm = smoother.pathsmoothing(path, rrtc, 30)
    # plt.plot([point[0] for point in pathsm], [point[1] for point in pathsm], '-k')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()