import BoundingSphere as bs
import numpy as np

class BoundingBoxTreeNode:

    def __init__(self, bSpheres, nS):
        #bSpheres is list of bounding spheres, nS is number of bounding spheres in the list
        self.spheres = bSpheres
        self.center = bs.centroid(bSpheres, nS)
        self.maxRadius = bs.findMaxRadius(bSpheres, nS)
        self.UB = bs.findMaxCoordinates(bSpheres, nS)
        self.LB = bs.findMinCoordinates(bSpheres, nS)
        self.haveSubtrees = None
        self.nSpheres = nS
        self.subtrees = [[[[], []], [[], []]], [[[], []], [[], []]]]
        self.constructSubtrees(self.nSpheres, self.UB, self.LB)

    def constructSubtrees(self, nSpheres, UB, LB):
        minCount = 1
        minDiag = 0
        if(nSpheres < minCount or np.all(UB - LB) <= minDiag):
            self.haveSubtrees = 0
            return
        self.haveSubtrees = 1
        sortedspheres, nnn, npn, npp, nnp, pnn, ppn, ppp, pnp, indices = splitSort(self.center, self.spheres)
        self.spheres = sortedspheres
        self.subtrees[0][0][0] = BoundingBoxTreeNode(sortedspheres[0:indices[0]], nnn)
        self.subtrees[0][1][0] = BoundingBoxTreeNode(sortedspheres[indices[0]:indices[1]], npn)
        self.subtrees[0][1][1] = BoundingBoxTreeNode(sortedspheres[indices[1]:indices[2]], npp)
        self.subtrees[0][0][1] = BoundingBoxTreeNode(sortedspheres[indices[2]:indices[3]], nnp)
        self.subtrees[1][0][0] = BoundingBoxTreeNode(sortedspheres[indices[3]:indices[4]], pnn)
        self.subtrees[1][1][0] = BoundingBoxTreeNode(sortedspheres[indices[4]:indices[5]], ppn)
        self.subtrees[1][1][1] = BoundingBoxTreeNode(sortedspheres[indices[5]:indices[6]], ppp)
        self.subtrees[1][0][1] = BoundingBoxTreeNode(sortedspheres[indices[6]:indices[7]], pnp)

    def findClosestPoint(self, v, bound, closest):
        dist = bound + self.maxRadius
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
            self.subtrees[0][0][0].findClosestPoint(v, bound, closest)
            self.subtrees[0][1][0].findClosestPoint(v, bound, closest)
            self.subtrees[0][1][1].findClosestPoint(v, bound, closest)
            self.subtrees[0][0][1].findClosestPoint(v, bound, closest)
            self.subtrees[1][0][0].findClosestPoint(v, bound, closest)
            self.subtrees[1][1][0].findClosestPoint(v, bound, closest)
            self.subtrees[1][1][1].findClosestPoint(v, bound, closest)
            self.subtrees[1][0][1].findClosestPoint(v, bound, closest)
        else:
            for i in range(self.nSpheres):
                updateClosest(self.spheres[i], i, v, bound, closest)


def updateClosest(sphere, i, v, bound, closest):
    dist = np.norm(v - sphere.c)
    if (dist - sphere.r > bound):
        return
    cp = icpm.findClosestPoint()
    return None

def splitSort(splitpoint, spheres):
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
    sortedspheres = []
    for i in range(len(spheres)):
        center = spheres[i].c
        if (center[0] < splitpoint[0] and center[1] < splitpoint[1] and center[2] < splitpoint[2]):
            sortedspheres.insert(indices[0], spheres[i])
            indices += 1
            nnn += 1
        elif (center[0] < splitpoint[0] and center[1] > splitpoint[1] and center[2] < splitpoint[2]):
            sortedspheres.insert(indices[1], spheres[i])
            indices[1:8] += 1
            npn += 1
        elif (center[0] < splitpoint[0] and center[1] > splitpoint[1] and center[2] > splitpoint[2]):
            sortedspheres.insert(indices[2], spheres[i])
            indices[2:8] += 1
            npp += 1
        elif (center[0] < splitpoint[0] and center[1] < splitpoint[1] and center[2] > splitpoint[2]):
            sortedspheres.insert(indices[3], spheres[i])
            indices[3:8] += 1
            nnp += 1
        elif (center[0] > splitpoint[0] and center[1] < splitpoint[1] and center[2] < splitpoint[2]):
            sortedspheres.insert(indices[4], spheres[i])
            indices[4:8] += 1
            pnn += 1
        elif (center[0] > splitpoint[0] and center[1] > splitpoint[1] and center[2] < splitpoint[2]):
            sortedspheres.insert(indices[5], spheres[i])
            indices[5:8] += 1
            ppn += 1
        elif (center[0] > splitpoint[0] and center[1] > splitpoint[1] and center[2] > splitpoint[2]):
            sortedspheres.insert(indices[6], spheres[i])
            indices[6:8] += 1
            ppp += 1
        elif (center[0] > splitpoint[0] and center[1] < splitpoint[1] and center[2] > splitpoint[2]):
            sortedspheres.insert(indices[7], spheres[i])
            indices[7:8] += 1
            pnp += 1
    return sortedspheres, nnn, npn, npp, nnp, pnn, ppn, ppp, pnp, indices



