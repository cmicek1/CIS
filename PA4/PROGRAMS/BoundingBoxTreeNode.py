import BoundingSphere as bs
import numpy as np
import numpy.linalg as numalg
import ICPmatching as icpm

class BoundingBoxTreeNode:

    def __init__(self, triangles, nT):
        #bSpheres is list of bounding spheres, nS is number of bounding spheres in the list
        self.triangles = triangles
        self.spheres = self.findspheres(triangles)
        self.center = bs.centroid(self.spheres, nT)
        self.maxRadius = bs.findMaxRadius(self.spheres, nT)
        self.UB = bs.findMaxCoordinates(self.spheres, nT)
        self.LB = bs.findMinCoordinates(self.spheres, nT)
        self.haveSubtrees = None
        self.nSpheres = nT
        self.subtrees = [[[[], []], [[], []]], [[[], []], [[], []]]]
        self.constructSubtrees(self.nSpheres, self.UB, self.LB)

    def findspheres(self, triangles):
        spheres = []
        for i in range(len(triangles)):
            spheres.append(triangles[i].sphere)

        return spheres

    def constructSubtrees(self, nSpheres, UB, LB):
        minCount = 1
        minDiag = 0
        if(nSpheres < minCount or np.all(UB - LB) <= minDiag):
            self.haveSubtrees = 0
            return
        self.haveSubtrees = 1
        sortedThings, nnn, npn, npp, nnp, pnn, ppn, ppp, pnp, indices = splitSort(self.center, self.spheres, self.triangles)
        self.triangles = sortedThings
        self.subtrees[0][0][0] = BoundingBoxTreeNode(sortedThings[0:indices[0]], nnn)
        self.subtrees[0][1][0] = BoundingBoxTreeNode(sortedThings[indices[0]:indices[1]], npn)
        self.subtrees[0][1][1] = BoundingBoxTreeNode(sortedThings[indices[1]:indices[2]], npp)
        self.subtrees[0][0][1] = BoundingBoxTreeNode(sortedThings[indices[2]:indices[3]], nnp)
        self.subtrees[1][0][0] = BoundingBoxTreeNode(sortedThings[indices[3]:indices[4]], pnn)
        self.subtrees[1][1][0] = BoundingBoxTreeNode(sortedThings[indices[4]:indices[5]], ppn)
        self.subtrees[1][1][1] = BoundingBoxTreeNode(sortedThings[indices[5]:indices[6]], ppp)
        self.subtrees[1][0][1] = BoundingBoxTreeNode(sortedThings[indices[6]:indices[7]], pnp)

    def FindClosestPoint(self, v, bound, closest):
        dist = bound[0] + self.maxRadius
        if (v[0] > self.UB[0] + dist):
            return
        elif (v[1] > self.UB[1] + dist):
            return
        elif (v[2] > self.UB[2] + dist):
            return
        elif (v[0] < self.LB[0] - dist):
            return
        elif (v[1] < self.LB[1] - dist):
            return
        elif (v[2] < self.LB[2] - dist):
            return
        if(self.haveSubtrees):
            self.subtrees[0][0][0].FindClosestPoint(v, bound, closest)
            self.subtrees[0][1][0].FindClosestPoint(v, bound, closest)
            self.subtrees[0][1][1].FindClosestPoint(v, bound, closest)
            self.subtrees[0][0][1].FindClosestPoint(v, bound, closest)
            self.subtrees[1][0][0].FindClosestPoint(v, bound, closest)
            self.subtrees[1][1][0].FindClosestPoint(v, bound, closest)
            self.subtrees[1][1][1].FindClosestPoint(v, bound, closest)
            self.subtrees[1][0][1].FindClosestPoint(v, bound, closest)
        else:
            for i in range(self.nSpheres):
                self.updateClosest(self.triangles[i], v, bound, closest)


    def updateClosest(self, triangle, v, bound, closest):
        dist = numalg.norm(v - triangle.sphere.c)
        if (dist - triangle.sphere.r > bound):
            return
        cp = icpm.minPointonTriangle(v, triangle.corners.data[0], triangle.corners.data[1], triangle.corners.data[2])
        dist = numalg.norm(v - cp)
        if dist < bound:
            bound = dist
            closest = cp

def splitSort(splitpoint, spheres, triangles):
    #nnn, npn, npp, nnp, pnn, ppn, ppp, pnp = 0
    nnn = 0
    npn = 0
    npp = 0
    nnp = 0
    pnn = 0
    ppn = 0
    ppp = 0
    pnp = 0
    indices = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    sortedThings = []
    for i in range(len(spheres)):
        center = spheres[i].c
        if (center[0] < splitpoint[0] and center[1] < splitpoint[1] and center[2] < splitpoint[2]):
            sortedThings.insert(indices[0], triangles[i])
            indices += 1
            nnn += 1
        elif (center[0] < splitpoint[0] and center[1] > splitpoint[1] and center[2] < splitpoint[2]):
            sortedThings.insert(indices[1], triangles[i])
            indices[1:8] += 1
            npn += 1
        elif (center[0] < splitpoint[0] and center[1] > splitpoint[1] and center[2] > splitpoint[2]):
            sortedThings.insert(indices[2], triangles[i])
            indices[2:8] += 1
            npp += 1
        elif (center[0] < splitpoint[0] and center[1] < splitpoint[1] and center[2] > splitpoint[2]):
            sortedThings.insert(indices[3], triangles[i])
            indices[3:8] += 1
            nnp += 1
        elif (center[0] > splitpoint[0] and center[1] < splitpoint[1] and center[2] < splitpoint[2]):
            sortedThings.insert(indices[4], triangles[i])
            indices[4:8] += 1
            pnn += 1
        elif (center[0] > splitpoint[0] and center[1] > splitpoint[1] and center[2] < splitpoint[2]):
            sortedThings.insert(indices[5], triangles[i])
            indices[5:8] += 1
            ppn += 1
        elif (center[0] > splitpoint[0] and center[1] > splitpoint[1] and center[2] > splitpoint[2]):
            sortedThings.insert(indices[6], triangles[i])
            indices[6:8] += 1
            ppp += 1
        elif (center[0] > splitpoint[0] and center[1] < splitpoint[1] and center[2] > splitpoint[2]):
            sortedThings.insert(indices[7], triangles[i])
            indices[7:8] += 1
            pnp += 1
    return sortedThings, nnn, npn, npp, nnp, pnn, ppn, ppp, pnp, indices



