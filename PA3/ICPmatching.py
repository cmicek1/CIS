import numpy as np
import PointCloud as pc
import Frame as fr

def findTipB(aFrames, bFrames, aTip):

    d_ks = np.zeros([3, len(aFrames)])

    for i in range(len(aFrames)):
        reg = aFrames[i].register(bFrames[i])
        d_k = reg.r.dot(aTip.data) + reg.p
        for j in range(0, 3):
            d_ks[j][i] = d_k[j]

    return d_ks


def findClosestPoint(s_i, vCoords, vInd):
    """
    Finds the closest point to p on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param p:
    :param vCoords:
    :param vInd:
    :return:
    """
    #find closest point on each triangle
    #find point with smallest distance from given point p
    #output that point

    c_ij = []
    print vInd
    for i in range(len(vInd)):
        r = vCoords(vInd[i][0])
        p = vCoords(vInd[i][1])