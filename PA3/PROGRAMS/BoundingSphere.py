import numpy as np
import numpy.linalg as numalg

class BoundingSphere:

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
    Calculates the center of a bounding sphere around triangle with vertices a, b, c.
    :param a:
    :param b:
    :param c:
    :return:
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
    bs = []
    for i in range (np.shape(vInd)[1]):
        a = vCoords[:, int(vInd[0][i])]
        b = vCoords[:, int(vInd[1][i])]
        c = vCoords[:, int(vInd[2][i])]
        q, p = calcCenterandRadius(a, b, c)

        bs.append(BoundingSphere(q, p))

    return bs

