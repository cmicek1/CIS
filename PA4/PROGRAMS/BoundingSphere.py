import numpy as np
import numpy.linalg as numalg


class BoundingSphere:
    """
    Class representing a bounding sphere around a triangle.
    """
    def __init__(self, c, r):
        """
        Initialize sphere with given center and radius.

        :param c: The center of the bounding sphere
        :param r: The radius of the bounding sphere

        :type c: numpy.array([numpy.float64]), 3x1
        :type r: numpy.float64
        """
        self.c = c
        self.r = r


def calcCenterandRadius(a, b, c):
    """
    Calculates the center and radius of a bounding sphere around triangle with vertices a, b, c.
    :param a: First vertex of triangle
    :param b: Second vertex of triangle
    :param c: Third vertex of triangle

    :type a: np.array([np.float64]), 3 x 1
    :type b: np.array([np.float64]), 3 x 1
    :type c: np.array([np.float64]), 3 x 1

    :return: center of bounding sphere
    :return: radius of bounding sphere
    :rtype: np.array([np.float64]), 3 x 1
    :rtype: np.float64
    """
    f = (a + b)/2
    u = a - f
    v = c - f
    d = np.cross(np.cross(u, v), u)
    l = max(0, ((v.dot(v) - u.dot(u))/(2*d).dot(v-u)))
    q = f + l*d
    p = numalg.norm(q - a)
    return q, p


def createBS(vCoords, vInd):
    """
    Creates a list of bounding spheres for array of triangles defined with vertex coordinates.
    :param vCoords: Coordinates of vertices on surface
    :param vInd: Indices of vertices for each triangle on surface

    :type vCoords: np.array([np.float64]), 3 x N
    :type vInd: np.array([np.float64]), 3 x M

    :return: A list of bounding spheres around each triangle defined with vertex indices in vInd
    :rtype: []
    """
    bs = []
    for i in range(np.shape(vInd)[1]):
        a = vCoords[:, int(vInd[0][i])]
        b = vCoords[:, int(vInd[1][i])]
        c = vCoords[:, int(vInd[2][i])]
        q, p = calcCenterandRadius(a, b, c)

        bs.append(BoundingSphere(q, p))

    return bs

def centroid(BSList, nS):
    c = np.zeros([3, 1])
    for i in range(nS):
        c[0] += BSList[i].c[0]
        c[1] += BSList[i].c[1]
        c[2] += BSList[i].c[2]
    c = c/nS
    return c

def findMaxCoordinates(BSList, ns):
    max = np.zeros([3, 1])
    for i in range(ns):
        thisSphere = BSList[i]
        if (thisSphere.c[0] > max[0] and thisSphere.c[1] > max[1] and thisSphere.c[2] > max[2]):
            max[0] = thisSphere.c[0]
            max[1] = thisSphere.c[1]
            max[2] = thisSphere.c[2]
    return max

def findMinCoordinates(BSList, ns):
    min = np.zeros([3, 1])
    for i in range(ns):
        thisSphere = BSList[i]
        if (thisSphere.c[0] < min[0] and thisSphere.c[1] < min[1] and thisSphere.c[2] < min[2]):
            min[0] = thisSphere.c[0]
            min[1] = thisSphere.c[1]
            min[2] = thisSphere.c[2]
    return min

def findMaxRadius(BSList, ns):
    max = 0
    for i in range(ns):
        if (BSList[i].r > max):
            max = BSList[i].r
    return max

