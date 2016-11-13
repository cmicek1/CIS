import numpy as np
import PointCloud as pc
import Frame as fr

def findTipB(aFrames, bFrames, aTip):

    for i in range(len(aFrames)):
        reg = bFrames[i].register(aFrames[i])
        d_k = reg.r.dot(aTip.data) + reg.p
        print d_k

def findClosestPoint(p, vCoords, vInd):
    """
    Finds the closest point to p on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param p:
    :param vCoords:
    :param vInd:
    :return:
    """
