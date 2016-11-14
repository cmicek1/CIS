import numpy as np
import numpy.linalg as numalg

def findTipB(aFrames, bFrames, ledA, tipA, ledB, tipB):

    d_ks = np.zeros([3, len(aFrames)])

    for i in range(len(aFrames)):
        regA = ledA.register(aFrames[i])
        regB = ledB.register(bFrames[i])
        reg = regB.inv.compose(regA)
        d_k = reg.r.dot(tipA.data) + reg.p
        for j in range(0, 3):
            d_ks[j][i] = d_k[j]

    return d_ks

def computeSamplePoints(d_k, freg):

    for i in range(len(d_k)):
        d_k[:, i] = freg.r.dot(d_k[:, i]).T + freg.p

    return d_k

def findClosestPoint(s_i, vCoords, vInd):
    """
    Finds the closest point to s_i on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param p:
    :param vCoords:
    :param vInd:
    :return:
    """
    c_ij = np.zeros([3, np.shape(vInd)[1]])

    for i in range(np.shape(vInd)[1]):
        p = vCoords[:, int(vInd[0][i])]
        q = vCoords[:, int(vInd[1][i])]
        r = vCoords[:, int(vInd[2][i])]
        A = np.zeros([3, 2])

        for j in range(0, 3):
            A[j][0] = q[j] - p[j]
            A[j][1] = r[j] - p[j]

        soln = numalg.lstsq(A, s_i - p)
        l = soln[0][0]
        u = soln[0][1]

        c = p + l * (q - p) + u * (r - p)
        c_star = np.zeros(3)

        if ((l > 0) and (u > 0) and (l + u < 0)):
            c_star = c
        elif (l < 0):
            c_star = projectOnSegment(c, r, p)
        elif (u < 0):
            c_star = projectOnSegment(c, p, q)
        elif (l + u > 1):
            c_star = projectOnSegment(c, q, r)

        c_ij[:, i] = c_star[:]

    dist = numalg.norm(s_i - c_ij[:, 0])
    minPoint = c_ij[:, 0]
    for i in range(np.shape(c_ij)[1]):
        d = numalg.norm(s_i - c_ij[:, i])
        if (d < dist):
            dist = d
            minPoint = c_ij[:, i]

    return minPoint

def projectOnSegment(c, p, q):

    l = ((c-p).dot(q-p))/((q-p).dot(q-p))
    lambda_star = max(0, min(l, 1))
    c_star = p + lambda_star*(q-p)
    return c_star


def ICPmatch(s_i, vCoords, vInd):

    c_ij = np.zeros([3, np.shape(s_i)[1]])
    for i in range(np.shape(s_i)[1]):
        c = findClosestPoint(s_i[:,i], vCoords, vInd)
        c_ij[:, i] = c[:]

    return c_ij

def calcDifference(c_kPoints, d_kPoints):
    dist = np.zeros(np.shape(c_kPoints)[1])
    for i in range(np.shape(c_kPoints)[1]):
        dist[i] = numalg.norm(d_kPoints[:, i] - c_kPoints[:, i])

    return dist