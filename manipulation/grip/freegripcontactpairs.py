#!/usr/bin/python

import itertools
import os

import numpy as np
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import MultiPolygon
from sklearn.cluster import KMeans
from sklearn.neighbors import RadiusNeighborsClassifier

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh.sample as sample
import sys
import trimesh
from utils import robotmath
import pickle
import time

class FreegripContactpairs(object):

    def __init__(self, ompath, dotnormplan=.95, dotnoarmovlp=.95, readser=False):
        self.objtrimesh = None
        # the sampled points and their normals
        self.objsamplepnts = None
        self.objsamplenrmls = None
        # the sampled points (bad samples removed)
        self.objsamplepnts_ref = None
        self.objsamplenrmls_ref = None
        # the sampled points (bad samples removed + clustered)
        self.objsamplepnts_refcls = None
        self.objsamplenrmls_refcls = None
        # facets is used to avoid repeated computation
        self.facets = None
        # facetnormals is used to plot overlapped facets with different heights
        self.facetnormals = None
        # facet2dbdries saves the 2d boundaries of each facet
        self.facet2dbdries = None
        # the contactpairs are not index by facetids but by facetpairs [facet0, facet1]
        self.facetpairs = None
        self.gripcontactpairs = None
        self.gripcontactpairnormals = None
        self.gripcontactpairfacets = None
        self.gripcontactrotangles = None
        # for plot
        self.facetcolorarray = None
        self.counter = 0
        self.pggen = pandageom.PandaGeomGen()
        if readser is False:
            self.loadObjModel(ompath, dotnormplan=dotnormplan, dotnoarmovlp=dotnoarmovlp)
            self.saveSerialized("tmpnocp.pickle")
        else:
            self.loadSerialized("tmpnocp.pickle", ompath)

    def loadObjModel(self, ompath, dotnormplan=.95, dotnoarmovlp=.95):
        self.objtrimesh=trimesh.load_mesh(ompath)
        # oversegmentation
        tic = time.time()
        self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=dotnormplan, segangle = dotnoarmovlp)
        toc = time.time()
        print ("facet cost", toc -tic)
        # show original triangles
        # self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=1.95, segangle = 1.95)
        # self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=.95)
        # conventional approach
        # self.facets = self.objtrimesh.facets()
        # self.facetnormals = []
        # for i, faces in enumerate(self.objtrimesh.facets()):
        #     facetnormal = np.sum(self.objtrimesh.face_normals[faces], axis=0)
        #     facetnormal = facetnormal/np.linalg.norm(facetnormal)
        #     self.facetnormals.append(facetnormal)
        self.facetcolorarray = pandageom.randomColorArray(self.facets.shape[0])
        tic = time.time()
        self.sampleObjModel()
        toc = time.time()
        print ("sampling cost", toc-tic)

    def loadSerialized(self, filename, ompath):
        self.objtrimesh=trimesh.load_mesh(ompath)
        try:
            self.facets, self.facetnormals, self.facetcolorarray, self.objsamplepnts, \
            self.objsamplenrmls, self.objsamplepnts_ref, self.objsamplenrmls_ref, \
            self.objsamplepnts_refcls, self.objsamplenrmls_refcls, self.facetpairs, \
            self.gripcontactpairs, self.gripcontactpairnormals, self.gripcontactpairfacets = \
            pickle.load(open(filename, mode="rb"))
        except:
            print( str(sys.exc_info()[0])+" cannot load tmpcp.pickle")
            raise

    def saveSerialized(self, filename):
        pickle.dump([self.facets, self.facetnormals, self.facetcolorarray, self.objsamplepnts, \
         self.objsamplenrmls, self.objsamplepnts_ref, self.objsamplenrmls_ref, \
         self.objsamplepnts_refcls, self.objsamplenrmls_refcls, self.facetpairs, \
         self.gripcontactpairs, self.gripcontactpairnormals, self.gripcontactpairfacets], open(filename, mode="wb"))

    def sampleObjModel(self, numpointsoververts=5):
        """
        sample the object model
        self.objsamplepnts and self.objsamplenrmls
        are filled in this function

        :param: numpointsover verts: the number of sampled points = numpointsoververts*mesh.vertices.shape[0]
        :return: nverts: the number of verts sampled

        author: weiwei
        date: 20160623 flight to tokyo
        """

        nverts = self.objtrimesh.vertices.shape[0]
        samples, face_idx = sample.sample_surface_even_withfaceid(self.objtrimesh,
                                                       count=(1000 if nverts*numpointsoververts > 1000 \
                                                                  else nverts*numpointsoververts))
        # print nverts
        self.objsamplepnts = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, faces in enumerate(self.facets):
            for face in faces:
                sample_idx = np.where(face_idx==face)[0]
                if len(sample_idx) > 0:
                    if self.objsamplepnts[i] is not None:
                        self.objsamplepnts[i] = np.vstack((self.objsamplepnts[i], samples[sample_idx]))
                        self.objsamplenrmls[i] = np.vstack((self.objsamplenrmls[i],
                                                            [self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0]))
                    else:
                        self.objsamplepnts[i] = np.array(samples[sample_idx])
                        self.objsamplenrmls[i] = np.array([self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0])
            if self.objsamplepnts[i] is None:
                self.objsamplepnts[i] = np.empty(shape=[0,0])
                self.objsamplenrmls[i] = np.empty(shape=[0,0])
        return nverts

    def removeBadSamples(self, mindist=2, maxdist=20):
        '''
        Do the following refinement:
        (1) remove the samples who's minimum distance to facet boundary is smaller than mindist
        (2) remove the samples who's maximum distance to facet boundary is larger than mindist

        ## input
        mindist, maxdist
            as explained in the begining

        author: weiwei
        date: 20160623 flight to tokyo
        '''

        # ref = refine
        self.objsamplepnts_ref = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_ref = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.facet2dbdries = []
        for i, faces in enumerate(self.facets):
            # print "removebadsample"
            # print i,len(self.facets)
            facetp = None
            face0verts = self.objtrimesh.vertices[self.objtrimesh.faces[faces[0]]]
            facetmat = robotmath.rotmatfacet(self.facetnormals[i], face0verts[0], face0verts[1])
            # face samples
            samplepntsp =[]
            for j, apnt in enumerate(self.objsamplepnts[i]):
                apntp = np.dot(facetmat, apnt)[:2]
                samplepntsp.append(apntp)
            # face boundaries
            for j, faceidx in enumerate(faces):
                vert0 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][0]]
                vert1 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][1]]
                vert2 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][2]]
                vert0p = np.dot(facetmat, vert0)[:2]
                vert1p = np.dot(facetmat, vert1)[:2]
                vert2p = np.dot(facetmat, vert2)[:2]
                facep = Polygon([vert0p, vert1p, vert2p])
                if facetp is None:
                    facetp = facep
                else:
                    try:
                        facetp = facetp.union(facep)
                    except:
                        continue
            self.facet2dbdries.append(facetp)
            selectedele = []
            for j, apntp in enumerate(samplepntsp):
                try:
                    apntpnt = Point(apntp[0], apntp[1])
                    dbnds = []
                    dbnds.append(apntpnt.distance(facetp.exterior))
                    for fpinter in facetp.interiors:
                        dbnds.append(apntpnt.distance(fpinter))
                    dbnd = min(dbnds)
                    if dbnd < mindist or dbnd > maxdist:
                        pass
                    else:
                        selectedele.append(j)
                except:
                    pass
            self.objsamplepnts_ref[i] = np.asarray([self.objsamplepnts[i][j] for j in selectedele])
            self.objsamplenrmls_ref[i] = np.asarray([self.objsamplenrmls[i][j] for j in selectedele])
        self.facet2dbdries = np.array(self.facet2dbdries)

            # if i is 3:
            #     for j, apntp in enumerate(samplepntsp):
            #         apntpnt = Point(apntp[0], apntp[1])
            #         plt.plot(apntpnt.x, apntpnt.y, 'bo')
            #     for j, apnt in enumerate([samplepntsp[j] for j in selectedele]):
            #         plt.plot(apnt[0], apnt[1], 'ro')
            #     ftpx, ftpy = facetp.exterior.xy
            #     plt.plot(ftpx, ftpy)
            #     for fpinters in facetp.interiors:
            #         ftpxi, ftpyi = fpinters.xy
            #         plt.plot(ftpxi, ftpyi)
            #     plt.axis('equal')
            #     plt.show()
            #     pass

                # old code for concatenating in 3d space
                # boundaryedges = []
                # for faceid in faces:
                #     faceverts = self.objtrimesh.faces[faceid]
                #     try:
                #         boundaryedges.remove([faceverts[1], faceverts[0]])
                #     except:
                #         boundaryedges.append([faceverts[0], faceverts[1]])
                #     try:
                #         boundaryedges.remove([faceverts[2], faceverts[1]])
                #     except:
                #         boundaryedges.append([faceverts[1], faceverts[2]])
                #     try:
                #         boundaryedges.remove([faceverts[0], faceverts[2]])
                #     except:
                #         boundaryedges.append([faceverts[2], faceverts[0]])
                # print boundaryedges
                # print len(boundaryedges)
                # TODO: compute boundary polygons, both outsider and inner should be considered
                # assort boundaryedges
                # boundarypolygonlist = []
                # boundarypolygon = [boundaryedges[0]]
                # boundaryedgesfirstcolumn = [row[0] for row in boundaryedges]
                # for i in range(len(boundaryedges)-1):
                #     vertpivot = boundarypolygon[i][1]
                #     boundarypolygon.append(boundaryedges[boundaryedgesfirstcolumn.index(vertpivot)])
                # print boundarypolygon
                # print len(boundarypolygon)
                # return boundaryedges, boundarypolygon

    def clusterFacetSamplesKNN(self, reduceRatio=3, maxNPnts=5):
        """
        cluster the samples of each facet using k nearest neighbors
        the cluster center and their correspondent normals will be saved
        in self.objsamplepnts_refcls and self.objsamplenrmals_refcls

        :param: reduceRatio: the ratio of points to reduce
        :param: maxNPnts: the maximum number of points on a facet
        :return: None

        author: weiwei
        date: 20161129, tsukuba
        """

        self.objsamplepnts_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, facet in enumerate(self.facets):
            self.objsamplepnts_refcls[i] = np.empty(shape=(0,0))
            self.objsamplenrmls_refcls[i] = np.empty(shape=(0,0))
            X = self.objsamplepnts_ref[i]
            nX = X.shape[0]
            if nX > reduceRatio:
                kmeans = KMeans(n_clusters=maxNPnts if nX/reduceRatio>maxNPnts else nX/reduceRatio, random_state=0).fit(X)
                self.objsamplepnts_refcls[i] = kmeans.cluster_centers_
                self.objsamplenrmls_refcls[i] = np.tile(self.facetnormals[i], [self.objsamplepnts_refcls.shape[0],1])

    def clusterFacetSamplesRNN(self, reduceRadius=3):
        """
        cluster the samples of each facet using radius nearest neighbours
        the cluster center and their correspondent normals will be saved
        in self.objsamplepnts_refcls and self.objsamplenrmals_refcls

        :param: reduceRadius: the neighbors that fall inside the reduceradius will be removed
        :return: None

        author: weiwei
        date: 20161130, osaka
        """

        self.objsamplepnts_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, facet in enumerate(self.facets):
            # print "cluster"
            # print i,len(self.facets)
            self.objsamplepnts_refcls[i] = []
            self.objsamplenrmls_refcls[i] = []
            X = self.objsamplepnts_ref[i]
            nX = X.shape[0]
            if nX > 0:
                neigh = RadiusNeighborsClassifier(radius=1.0)
                neigh.fit(X, range(nX))
                neigharrays = neigh.radius_neighbors(X, radius=reduceRadius, return_distance=False)
                delset = set([])
                for j in range(nX):
                    if j not in delset:
                        self.objsamplepnts_refcls[i].append(np.array(X[j]))
                        self.objsamplenrmls_refcls[i].append(np.array(self.objsamplenrmls_ref[i][j]))
                        # if self.objsamplepnts_refcls[i].size:
                        #     self.objsamplepnts_refcls[i] = np.vstack((self.objsamplepnts_refcls[i], X[j]))
                        #     self.objsamplenrmls_refcls[i] = np.vstack((self.objsamplenrmls_refcls[i],
                        #                                                 self.objsamplenrmls_ref[i][j]))
                        # else:
                        #     self.objsamplepnts_refcls[i] = np.array([])
                        #     self.objsamplenrmls_refcls[i] = np.array([])
                        #     self.objsamplepnts_refcls[i] = np.hstack((self.objsamplepnts_refcls[i], X[j]))
                        #     self.objsamplenrmls_refcls[i] = np.hstack((self.objsamplenrmls_refcls[i],
                        #                                                 self.objsamplenrmls_ref[i][j]))
                        delset.update(neigharrays[j].tolist())
            if self.objsamplepnts_refcls[i]:
                self.objsamplepnts_refcls[i] = np.vstack(self.objsamplepnts_refcls[i])
                self.objsamplenrmls_refcls[i] = np.vstack(self.objsamplenrmls_refcls[i])
            else:
                self.objsamplepnts_refcls[i] = np.empty(shape=(0,0))
                self.objsamplenrmls_refcls[i] = np.empty(shape=(0,0))

    def planContactpairs(self, torqueresist = 50, fgrtipdist = 82, dotnormpara = -0.75):
        """
        find the grasps using parallel pairs

        :param: torqueresist the maximum allowable distance to com
        :param: fgrtipdist the maximum dist between finger tips
        :param: dotnormpara parallelity
        :return:

        author: weiwei
        date: 20161130, harada office @ osaka university
        """

        # note that pairnormals and pairfacets are duplicated for each contactpair
        # the duplication is performed on purpose for convenient access
        # also, each contactpair"s" corresponds to a facetpair
        # it is empty when no contactpair is available
        self.gripcontactpairs = []
        # gripcontactpairnormals and gripcontactpairfacets are not helpful
        # they are kept for convenience (they could be accessed using facetnormals and facetpairs)
        self.gripcontactpairnormals = []
        self.gripcontactpairfacets = []

        # for precollision detection
        # bulletworldprecc = BulletWorld()
        # # build the collision shape of objtrimesh
        # geomobj = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals,
        #                                   self.objtrimesh.faces)
        # objmesh = BulletTriangleMesh()
        # objmesh.addGeom(geomobj)
        # objmeshnode = BulletRigidBodyNode('objmesh')
        # objmeshnode.addShape(BulletTriangleMeshShape(objmesh, dynamic=True))
        # bulletworldprecc.attachRigidBody(objmeshnode)

        # for raytracing
        bulletworldray = BulletWorld()
        nfacets = self.facets.shape[0]
        self.facetpairs = list(itertools.combinations(range(nfacets), 2))
        for facetpair in self.facetpairs:
            # print "facetpair"
            # print facetpair, len(self.facetpairs)
            self.gripcontactpairs.append([])
            self.gripcontactpairnormals.append([])
            self.gripcontactpairfacets.append([])
            # if one of the facet doesnt have samples, jump to next
            if self.objsamplepnts_refcls[facetpair[0]].shape[0] is 0 or \
                            self.objsamplepnts_refcls[facetpair[1]].shape[0] is 0:
                # print "no sampled points"
                continue
            # check if the faces are opposite and parallel
            dotnorm = np.dot(self.facetnormals[facetpair[0]], self.facetnormals[facetpair[1]])
            if dotnorm < dotnormpara:
                # check if any samplepnts's projection from facetpairs[i][0] falls in the polygon of facetpairs[i][1]
                facet0pnts = self.objsamplepnts_refcls[facetpair[0]]
                facet0normal = self.facetnormals[facetpair[0]]
                facet1normal = self.facetnormals[facetpair[1]]
                # generate collision mesh
                facetmesh = BulletTriangleMesh()
                faceidsonfacet = self.facets[facetpair[1]]
                geom = pandageom.packpandageom_fn(self.objtrimesh.vertices,
                                               self.objtrimesh.face_normals[faceidsonfacet],
                                               self.objtrimesh.faces[faceidsonfacet])
                facetmesh.addGeom(geom)
                facetmeshbullnode = BulletRigidBodyNode('facet')
                facetmeshbullnode.addShape(BulletTriangleMeshShape(facetmesh, dynamic=True))
                bulletworldray.attachRigidBody(facetmeshbullnode)
                # check the projection of a ray
                for facet0pnt in facet0pnts:
                    pFrom = Point3(facet0pnt[0], facet0pnt[1], facet0pnt[2])
                    pTo = pFrom + Vec3(facet1normal[0], facet1normal[1], facet1normal[2])*9999
                    result = bulletworldray.rayTestClosest(pFrom, pTo)
                    if result.hasHit():
                        hitpos = result.getHitPos()
                        if np.linalg.norm(np.array(facet0pnt.tolist())-np.array([hitpos[0], hitpos[1], hitpos[2]])) < fgrtipdist:
                            fgrcenter = (np.array(facet0pnt.tolist())+np.array([hitpos[0], hitpos[1], hitpos[2]]))/2.0
                            # avoid large torque
                            if np.linalg.norm(self.objtrimesh.center_mass - fgrcenter) < torqueresist:
                                self.gripcontactpairs[-1].append([facet0pnt.tolist(), [hitpos[0], hitpos[1], hitpos[2]]])
                                self.gripcontactpairnormals[-1].append([[facet0normal[0], facet0normal[1], facet0normal[2]],
                                                                    [facet1normal[0], facet1normal[1], facet1normal[2]]])
                                self.gripcontactpairfacets[-1].append(facetpair)

                        # pre collision checking: it takes one finger as a cylinder and
                        # computes its collision with the object
                        ## first finger
                        # cylindernode0 = BulletRigidBodyNode('cylindernode0')
                        # cylinder0height = 50
                        # cylinder0normal = Vec3(facet0normal[0], facet0normal[1], facet0normal[2])
                        # cylindernode0.addShape(BulletCylinderShape(radius=self.preccradius,
                        #                                            height=cylinder0height,
                        #                                            up=cylinder0normal),
                        #                        TransformState.makePos(pFrom+cylinder0normal*cylinder0height*.6))
                        # bulletworldprecc.attachRigidBody(cylindernode0)
                        # ## second finger
                        # cylindernode1 = BulletRigidBodyNode('cylindernode1')
                        # cylinder1height = cylinder1height
                        # # use the inverse of facet0normal instead of facet1normal to follow hand orientation
                        # cylinder1normal = Vec3(-facet0normal[0], -facet0normal[1], -facet0normal[2])
                        # cylindernode1.addShape(BulletCylinderShape(radius=self.preccradius,
                        #                                            height=cylinder1height,
                        #                                            up=cylinder1normal),
                        #                        TransformState.makePos(pFrom+cylinder1normal*cylinder1height*.6))
                        # bulletworldprecc.attachRigidBody(cylindernode1)
                        # if bulletworldprecc.contactTestPair(cylindernode0, objmeshnode) and \
                        #     bulletworldprecc.contactTestPair(cylindernode1, objmeshnode):

                bulletworldray.removeRigidBody(facetmeshbullnode)

        # update the pairs
        availablepairids = np.where(self.gripcontactpairs)[0]
        self.facetpairs = np.array(self.facetpairs)[availablepairids]
        self.gripcontactpairs = np.array(self.gripcontactpairs)[availablepairids]
        self.gripcontactpairnormals = np.array(self.gripcontactpairnormals)[availablepairids]
        self.gripcontactpairfacets = np.array(self.gripcontactpairfacets)[availablepairids]

    def segShow(self, base, togglesamples=False, togglenormals=False,
                togglesamples_ref=False, togglenormals_ref=False,
                togglesamples_refcls=False, togglenormals_refcls=False, alpha = .1):
        """

        :param base:
        :param togglesamples:
        :param togglenormals:
        :param togglesamples_ref: toggles the sampled points that fulfills the dist requirements
        :param togglenormals_ref:
        :return:
        """

        nfacets = self.facets.shape[0]
        facetcolorarray = self.facetcolorarray

        # offsetf = facet
        plotoffsetf = .0
        # plot the segments
        print ("number of facets", len(self.facets))
        print ("average triangles", np.array([len(facet) for facet in self.facets]).mean())
        for i, facet in enumerate(self.facets):
            geom = pandageom.packpandageom_fn(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                            [self.objtrimesh.vertices.shape[0],1]),
                                           self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
            node = GeomNode('piece')
            node.addGeom(geom)
            star = NodePath('piece')
            star.attachNewNode(node)
            star.setColor(Vec4(facetcolorarray[i][0], facetcolorarray[i][1],
                               facetcolorarray[i][2], alpha))
            star.setTransparency(TransparencyAttrib.MAlpha)

            star.setTwoSided(True)
            star.reparentTo(base.render)
            # sampledpnts = samples[sample_idxes[i]]
            # for apnt in sampledpnts:
            #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
            rgbapnts0 = [1,1,1,1]
            rgbapnts1 = [.5,.5,0,1]
            rgbapnts2 = [1,0,0,1]
            if togglesamples:
                for j, apnt in enumerate(self.objsamplepnts[i]):
                    self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts0)
            if togglenormals:
                for j, apnt in enumerate(self.objsamplepnts[i]):
                    self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                        rgba=rgbapnts0, length=10)
            if togglesamples_ref:
                for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                    self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts1)
            if togglenormals_ref:
                for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                    self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                        rgba=rgbapnts1, length=10)
            if togglesamples_refcls:
                for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                    self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts2)
            if togglenormals_refcls:
                for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                    self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                        rgba=rgbapnts2, length=10)

    def segShow2(self, base, togglesamples=False, togglenormals=False,
                togglesamples_ref=False, togglenormals_ref=False,
                togglesamples_refcls=False, togglenormals_refcls=False, specificface = True):
        """

        :param base:
        :param togglesamples:
        :param togglenormals:
        :param togglesamples_ref: toggles the sampled points that fulfills the dist requirements
        :param togglenormals_ref:
        :return:
        """

        nfacets = self.facets.shape[0]
        facetcolorarray = self.facetcolorarray

        rgbapnts0 = [1, 1, 1, 1]
        rgbapnts1 = [0, 0, 1, 1]
        rgbapnts2 = [1, 0, 0, 1]

        # offsetf = facet
        plotoffsetf = .0
        faceplotted = False
        # plot the segments
        for i, facet in enumerate(self.facets):
            if not specificface:
                geom = pandageom.packpandageom_fn(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                                [self.objtrimesh.vertices.shape[0],1]),
                                               self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
                node = GeomNode('piece')
                node.addGeom(geom)
                star = NodePath('piece')
                star.attachNewNode(node)
                star.setColor(Vec4(.77, .17, 0, 1))
                star.setTransparency(TransparencyAttrib.MAlpha)

                star.setTwoSided(True)
                star.reparentTo(base.render)
                # sampledpnts = samples[sample_idxes[i]]
                # for apnt in sampledpnts:
                #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                if togglesamples:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.8, rgba=rgbapnts0)
                if togglenormals:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                            rgba=rgbapnts0, length=10)
                if togglesamples_ref:
                    for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                        self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.9, rgba=rgbapnts1)
                if togglenormals_ref:
                    for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                        self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                            rgba=rgbapnts1, length=10)
                if togglesamples_refcls:
                    for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                        self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts2)
                if togglenormals_refcls:
                    for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                        self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                            rgba=rgbapnts2, length=10)
            if specificface:
                plotoffsetf = .3
                if faceplotted:
                    continue
                else:
                    if len(self.objsamplepnts[i])>85:
                        faceplotted = True
                        geom = pandageom.packpandageom_fn(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                                        [self.objtrimesh.vertices.shape[0],1]),
                                                       self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
                        node = GeomNode('piece')
                        node.addGeom(geom)
                        star = NodePath('piece')
                        star.attachNewNode(node)
                        star.setColor(Vec4(facetcolorarray[i][0], facetcolorarray[i][1], facetcolorarray[i][2], 1))
                        star.setColor(Vec4(.7,.3,.3, 1))
                        star.setTransparency(TransparencyAttrib.MAlpha)

                        star.setTwoSided(True)
                        star.reparentTo(base.render)
                        # sampledpnts = samples[sample_idxes[i]]
                        # for apnt in sampledpnts:
                        #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                        if togglesamples:
                            for j, apnt in enumerate(self.objsamplepnts[i]):
                                self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.8, rgba=rgbapnts0)
                        if togglenormals:
                            for j, apnt in enumerate(self.objsamplepnts[i]):
                                self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                                    rgba=rgbapnts0, length=10)
                        if togglesamples_ref:
                            for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                                self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.9, rgba=rgbapnts1)
                        if togglenormals_ref:
                            for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                                self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                                    rgba=rgbapnts1, length=10)
                        if togglesamples_refcls:
                            for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                                self.pggen.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts2)
                        if togglenormals_refcls:
                            for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                                self.pggen.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                                    rgba=rgbapnts2, length=10)

    def pairShow(self, base, togglecontacts = False, togglecontactnormals = False):
        # the following sentence requires segshow to be executed first
        facetcolorarray = self.facetcolorarray
        # offsetfp = facetpair
        # plotoffsetfp = np.random.random()*50
        plotoffsetfp = 0.0
        # plot the pairs and their contacts
        # for i in range(self.counter+1, len(self.facetpairs)):
        #     if self.gripcontactpairs[i]:
        #         self.counter = i
        #         break
        # if i is len(self.facetpairs):
        #     return
        # delete the facetpair after show
        # np0 = base.render.find("**/pair0")
        # if np0:
        #     np0.removeNode()
        # np1 = base.render.find("**/pair1")
        # if np1:
        #     np1.removeNode()
        self.counter += 1
        if self.counter >= self.facetpairs.shape[0]:
            # self.counter = 0
            return
        n = 30
        if self.counter > n or self.counter <= n-1:
            # self.counter = 0
            return
        facetpair = self.facetpairs[self.counter]
        facetidx0 = facetpair[0]
        facetidx1 = facetpair[1]
        geomfacet0 = pandageom.packpandageom_fn(self.objtrimesh.vertices+
                                       np.tile(plotoffsetfp*self.facetnormals[facetidx0],
                                               [self.objtrimesh.vertices.shape[0],1]),
                                       self.objtrimesh.face_normals[self.facets[facetidx0]],
                                       self.objtrimesh.faces[self.facets[facetidx0]])
        geomfacet1 = pandageom.packpandageom_fn(self.objtrimesh.vertices+
                                       np.tile(plotoffsetfp*self.facetnormals[facetidx1],
                                               [self.objtrimesh.vertices.shape[0],1]),
                                       self.objtrimesh.face_normals[self.facets[facetidx1]],
                                       self.objtrimesh.faces[self.facets[facetidx1]])
        # show the facetpair
        node0 = GeomNode('pair0')
        node0.addGeom(geomfacet0)
        star0 = NodePath('pair0')
        star0.attachNewNode(node0)
        star0.setColor(Vec4(facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                           facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]))
        star0.setTwoSided(True)
        star0.reparentTo(base.render)
        node1 = GeomNode('pair1')
        node1.addGeom(geomfacet1)
        star1 = NodePath('pair1')
        star1.attachNewNode(node1)
        # star1.setColor(Vec4(facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
        #                    facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]))
        # set to the same color
        star1.setColor(Vec4(facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                           facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]))
        star1.setTwoSided(True)
        star1.reparentTo(base.render)
        if togglecontacts:
            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cttpnt0 = contactpair[0]
                cttpnt1 = contactpair[1]
                self.pggen.plotSphere(star0, pos=cttpnt0+plotoffsetfp*self.facetnormals[facetidx0], radius=4,
                                     rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                           facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]])
                # pandageom.plotSphere(star1, pos=cttpnt1+plotoffsetfp*self.facetnormals[facetidx1], radius=4,
                #                      rgba=[facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                #                            facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]])
                # use the same color
                self.pggen.plotSphere(star1, pos=cttpnt1+plotoffsetfp*self.facetnormals[facetidx1], radius=4,
                                     rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                           facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]])
        if togglecontactnormals:
            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cttpnt0 = contactpair[0]
                cttpnt1 = contactpair[1]
                self.pggen.plotArrow(star0, spos=cttpnt0+plotoffsetfp*self.facetnormals[facetidx0],
                                epos=cttpnt0 + plotoffsetfp*self.facetnormals[facetidx0] +
                                     self.gripcontactpairnormals[self.counter][j][0],
                                rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                      facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]], length=10)
                # pandageom.plotArrow(star1,  spos=cttpnt1+plotoffsetfp*self.facetnormals[facetidx1],
                #                 epos=cttpnt1 + plotoffsetfp*self.facetnormals[facetidx1] +
                #                      self.gripcontactpairnormals[self.counter][j][1],
                #                 rgba=[facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                #                       facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]], length=10)
                # use the same color
                self.pggen.plotArrow(star1, spos=cttpnt1+plotoffsetfp*self.facetnormals[facetidx1],
                                epos=cttpnt1 + plotoffsetfp*self.facetnormals[facetidx1] +
                                     self.gripcontactpairnormals[self.counter][j][1],
                                rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                      facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]], length=10)
                # break
        # except:
        #     print "You might need to loadmodel first!"

if __name__=='__main__':
    # ax1 = fig.add_subplot(121, projection='3d')
    #
    # mesh = trimesh.load_mesh('./circlestar.obj')
    # samples, face_idx = sample.sample_surface_even(mesh, mesh.vertices.shape[0] * 10)
    # facets, facets_area = mesh.facets(return_area=True)
    # sample_idxes = np.ndarray(shape=(facets.shape[0],),dtype=np.object)
    # for i,faces in enumerate(facets):
    #     sample_idx = np.empty([0,0], dtype=np.int)
    #     for face in faces:
    #         sample_idx = np.append(sample_idx, np.where(face_idx == face)[0])
    #     sample_idxes[i]=sample_idx
    #

    base = pandactrl.World(camp=[0,300,-400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath = os.path.join(this_dir, "objects", "tool.stl")
    # objpath = os.path.join(this_dir, "objects", "tool2.stl")
    # objpath = os.path.join(this_dir, "objects", "planefrontstay.stl")
    objpath = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    # objpath = os.path.join(this_dir, "objects", "sandpart.stl")
    freegriptst = FreegripContactpairs(objpath)
    print(len(freegriptst.objtrimesh.faces))
    # freegriptst.objtrimesh.show()

    freegriptst.removeBadSamples(mindist=2, maxdist=20)
    # freegriptst.clusterFacetSamplesKNN(reduceRatio=15, maxNPnts=5)
    freegriptst.clusterFacetSamplesRNN(reduceRadius=11)
    freegriptst.planContactpairs()
    freegriptst.segShow(base, togglesamples=False, togglenormals=False,
                        togglesamples_ref=False, togglenormals_ref=False,
                        togglesamples_refcls=False, togglenormals_refcls=False, alpha = .2)

    # objnp = pandageom.packpandanp(freegriptst.objtrimesh.vertices,
    #                               freegriptst.objtrimesh.face_normals, freegriptst.objtrimesh.faces)
    # objnp.setColor(.3,.3,.3,1)
    # objnp.reparentTo(base.render)

    # freegriptst.segShow2(base, togglesamples=False, togglenormals=False,
    #                     togglesamples_ref=False, togglenormals_ref=False,
    #                     togglesamples_refcls=True, togglenormals_refcls=False, specificface = True)
    #
    # def updateshow0(freegriptst, task):
    #     npc = base.render.findAllMatches("**/piece")
    #     for np in npc:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # def updateshow1(freegriptst, task):
    #     npc = base.render.findAllMatches("**/piece")
    #     for np in npc:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=True, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # def updateshow2(freegriptst, task):
    #     np = base.render.find("**/piece")
    #     if np:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=True, togglenormals_ref=False,
    #                         togglesamples_refcls=True, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # taskMgr.doMethodLater(10, updateshow0, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # taskMgr.doMethodLater(20, updateshow1, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # taskMgr.doMethodLater(30, updateshow2, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # base.run()

    def updateshow(task):
        freegriptst.pairShow(base, togglecontacts=True, togglecontactnormals=True)
        print( task.delayTime)
        # if abs(task.delayTime-13) < 1:
        #     task.delayTime -= 12.85
        return task.again
    taskMgr.doMethodLater(.1, updateshow, "tickTask")

    # geom = None
    # for i, faces in enumerate(freegriptst.objtrimesh.facets()):
    #     rgba = [np.random.random(),np.random.random(),np.random.random(),1]
    #     # geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices, freegriptst.objtrimesh.face_normals[faces], freegriptst.objtrimesh.faces[faces])
    #     # compute facet normal
    #     facetnormal = np.sum(freegriptst.objtrimesh.face_normals[faces], axis=0)
    #     facetnormal = facetnormal/np.linalg.norm(facetnormal)
    #     geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices +
    #                             np.tile(0 * facetnormal,
    #                                     [freegriptst.objtrimesh.vertices.shape[0], 1]),
    #                             freegriptst.objtrimesh.face_normals[faces],
    #                             freegriptst.objtrimesh.faces[faces])
    #     node = GeomNode('piece')
    #     node.addGeom(geom)
    #     star = NodePath('piece')
    #     star.attachNewNode(node)
    #     star.setColor(Vec4(rgba[0],rgba[1],rgba[2],rgba[3]))
    #     # star.setColor(Vec4(.7,.4,0,1))
    #     star.setTwoSided(True)
    #     star.reparentTo(base.render)
        # sampledpnts = samples[sample_idxes[i]]
        # for apnt in sampledpnts:
        #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
        # for j, apnt in enumerate(freegriptst.objsamplepnts[i]):
        #     pandageom.plotSphere(base, star, pos=apnt, radius=0.7, rgba=rgba)
        #     pandageom.plotArrow(base, star, spos=apnt, epos=apnt+freegriptst.objsamplenrmls[i][j], rgba=[1,0,0,1], length=5, thickness=0.1)
    # # selectedfacet = 2
    # geom = ppg.packpandageom(mesh.vertices, mesh.face_normals[facets[selectedfacet]], mesh.faces[facets[selectedfacet]])
    # node = GeomNode('piece')
    # node.addGeom(geom)
    # star = NodePath('piece')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,0,0,1))
    # star.setTwoSided(True)
    # star.reparentTo(base.render)

    # for i, face in enumerate(mesh.faces[facets[selectedfacet]]):
    #     vert = (mesh.vertices[face[0],:]+mesh.vertices[face[1],:]+mesh.vertices[face[2],:])/3
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.face_normals[facets[selectedfacet][i],:], rgba=[1,0,0,1], length = 5, thickness = 0.1)

    # for i, vert in enumerate(mesh.vertices):
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.vertex_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)


    # generator = MeshDrawer()
    # generatorNode = generator.getRoot()
    # generatorNode.reparentTo(base.render)
    # generatorNode.setDepthWrite(False)
    # generatorNode.setTransparency(True)
    # generatorNode.setTwoSided(True)
    # generatorNode.setBin("fixed", 0)
    # generatorNode.setLightOff(True)
    #
    # generator.begin(base.cam, base.render)
    # generator.segment(Vec3(0,0,0), Vec3(10,0,0), Vec4(1,1,1,1), 0.5, Vec4(0,1,0,1))
    # generator.end()
    # mesh.show()

    # for face in facets:
    #     mesh.visual.face_colors[np.asarray(face)] = [trimesh.visual.random_color()]*mesh.visual.face_colors[face].shape[0]
    # mesh.show()
    # samples = sample.sample_surface_even(mesh, mesh.vertices.shape[0]*10)
    # ax3d.plot(ax1, samples[:,0], samples[:,1], samples[:,2], 'r.')
    # ax3dequal.set_axes_equal(ax1)
    #
    # ax2 = fig.add_subplot(122, projection='3d')
    # for face in facets:
    #     rndcolor = trimesh.visual.random_color()
    #     for faceid in face:
    #         triarray = mesh.vertices[mesh.faces[faceid]]
    #         tri = art3d.Poly3DCollection([triarray])
    #         tri.set_facecolor(mesh.visual.face_colors[faceid])
    #         ax2.add_collection3d(tri)

    # ax3dequal.set_axes_equal(ax2)
    # plt.show()
    #
    # from direct.showbase.ShowBase import ShowBase
    # from panda3d.core import *
    # import plot.pandactrl as pandactrl
    # import plot.pandageom as pandageom
    #
    # geom = ppg.packpandageom(mesh.vertices, mesh.face_normals, mesh.faces)
    # node = GeomNode('star')
    # node.addGeom(geom)
    # star = NodePath('star')
    # star.attachNewNode(node)
    # star.setColor(1,0,0)
    #
    #
    # base = ShowBase()
    #
    # # for i, face in enumerate(mesh.faces):
    # #     vert = (mesh.vertices[face[0],:]+mesh.vertices[face[1],:]+mesh.vertices[face[2],:])/3
    # #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.face_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)
    #
    # # for i, vert in enumerate(mesh.vertices):
    # #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.vertex_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)
    #
    # pandactrl.setRenderEffect(base)
    # pandactrl.setLight(base)
    # pandactrl.setCam(base, 0, 100, 100, 'perspective')
    #
    # star.reparentTo(base.render)
    #
    # generator = MeshDrawer()
    # generatorNode = generator.getRoot()
    # generatorNode.reparentTo(base.render)
    # generatorNode.setDepthWrite(False)
    # generatorNode.setTransparency(True)
    # generatorNode.setTwoSided(True)
    # generatorNode.setBin("fixed", 0)
    # generatorNode.setLightOff(True)
    #
    # generator.begin(base.cam, base.render)
    # generator.segment(Vec3(0,0,0), Vec3(10,0,0), Vec4(1,1,1,1), 0.5, Vec4(0,1,0,1))
    # generator.end()
    #
    base.run()
