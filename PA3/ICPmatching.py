import numpy as np
import numpy.linalg as numalg
import PointCloud as pc
import Frame as fr

def findTipB(aFrames, ledA, tipA):

    d_ks = np.zeros([3, len(aFrames)])

    for i in range(len(aFrames)):
        regA = ledA.register(aFrames[i])
        d_k = regA.r.dot(tipA.data) + regA.p
        for j in range(0, 3):
            d_ks[j][i] = d_k[j]

    return d_ks

def computeSamplePoints(d_k, freg):

    for i in range(len(d_k)):
        print freg.r.dot(d_k[:,i])
        d_k[:, i] = freg.r.dot(d_k[:, i]).T + freg.p

    return d_k

def findClosestPoint(s_i, vCoords, vInd):
    """
    Finds the closest point to p on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param p:
    :param vCoords:
    :param vInd:
    :return:
    """

 #   for i in range(np.shape(vInd)[1]):
    for i in range(1):
        p = vCoords[:, int(vInd[0][i])]
        q = vCoords[:, int(vInd[1][i])]
        r = vCoords[:, int(vInd[2][i])]
        A = np.zeros([3, 2])
        for i in range(0, 3):
            A[i][0] = q[i] - p[i]
            A[i][1] = r[i] - p[i]

        soln = numalg.lstsq(A, s_i - p)
        l = soln[0][0]
        u = soln[0][1]
        lambda_prime = calcLambdaPrime(l, u, p, q, r)


def calcLambdaPrime(l, u, p, q, r):
    print p
    print q
    print q-p
    print l*(q-p)
    c = p + l*(q-p) + u*(r-p)
    print c


def ICPmatch(s_i, vCoords, vInd):
    c_ij = np.zeros([3, len(s_i)])
    for i in range(1):
        c = np.zeros([3, 1])
        c = findClosestPoint(s_i[:,i], vCoords, vInd)
