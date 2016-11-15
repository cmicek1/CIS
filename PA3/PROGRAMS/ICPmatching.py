import numpy as np
import numpy.linalg as numalg
import BoundingSphere as bs

def findTipB(aFrames, bFrames, ledA, tipA, ledB):
    """
    Finds the positions of the tip of pointer A with respect to the B rigid body for each sample frame.
    :param aFrames: Sample frames of A pointer
    :param bFrames: Sample frames of B rigid body
    :param ledA: Position of LEDs on A pointer in calibration frame
    :param tipA: Position of tip with respect to A pointer
    :param ledB: Position of LEDs on B rigid body in calibration frame
    :param tipB: Position of attachment of B to bone in B coordinate system

    :return: d_ks: Tip positions with respect to B rigid body in each data frame
    """

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
    """
    Transforms tip points with given transformation.
    :param d_k: Array containing positions of tip with respect to rigid body B
    :param freg: Frame transformation

    :return: d_k: Transformed array of points
    """
    for i in range(len(d_k)):
        d_k[:, i] = freg.r.dot(d_k[:, i]).T + freg.p

    return d_k


def findClosestPoint(s_i, vCoords, vInd, spheres):
    """
    Finds the closest point to s_i on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param s_i: Given point
    :param vCoords: Coordinates of vertices
    :param vInd: Indices of vertex coordinates for each triangle

    :return: minPoint: The closest point to s_i on the surface mesh
    """
    p = vCoords[:, int(vInd[0][0])]
    q = vCoords[:, int(vInd[1][0])]
    r = vCoords[:, int(vInd[2][0])]

    minPoint = minPointonTriangle(s_i, p, q, r)
    dist = numalg.norm(s_i - minPoint)

    for i in range(1, np.shape(vInd)[1]):
        if(numalg.norm(s_i - spheres[i].c) - spheres[i].r < numalg.norm(s_i - minPoint)):
            #check carefully
            p = vCoords[:, int(vInd[0][i])]
            q = vCoords[:, int(vInd[1][i])]
            r = vCoords[:, int(vInd[2][i])]

            c_star = minPointonTriangle(s_i, p, q, r)

            d = numalg.norm(s_i - c_star)

            if (d < dist):
                dist = d
                minPoint = c_star

    return minPoint

def minPointonTriangle(s_i, p, q, r):
    A = np.zeros([3, 2])

    for j in range(0, 3):
        A[j][0] = q[j] - p[j]
        A[j][1] = r[j] - p[j]

    soln = numalg.lstsq(A, s_i - p)
    l = soln[0][0]
    u = soln[0][1]

    c = p + l * (q - p) + u * (r - p)
    c_star = np.zeros(3)

    if l > 0 and u > 0 and l + u < 1:
        c_star = c
    elif l < 0:
        c_star = projectOnSegment(c, r, p)
    elif u < 0:
        c_star = projectOnSegment(c, p, q)
    elif l + u > 1:
        c_star = projectOnSegment(c, q, r)

    return c_star


def projectOnSegment(c, p, q):
    """
    Projects point c onto line segment with endpoints p and q.
    :param c: Point to project onto line segment
    :param p: Endpoint of line segment
    :param q: Opposite endpoint of line segment

    :return: c_star: the projection of c onto line segment p_q
    """
    l = np.float64(((c-p).dot(q-p))) / np.float64(((q-p).dot(q-p)))
    lambda_star = max(0, min(l, 1))
    c_star = p + lambda_star*(q-p)
    return c_star


def ICPmatch(s_i, vCoords, vInd):
    """
    Finds the closest point on the surface for each point in an array of points.
    :param s_i: Array of points
    :param vCoords: Coordinates of each vertex
    :param vInd: Indices of vertices for each triangle

    :return: c_ij closest point on surface to each point in s_i
    """
    spheres = bs.createBS(vCoords, vInd)
    c_ij = np.zeros([3, np.shape(s_i)[1]])
    for i in range(np.shape(s_i)[1]):
        c = findClosestPoint(s_i[:,i], vCoords, vInd, spheres)
        c_ij[:, i] = c[:]

    return c_ij


def calcDifference(c_kPoints, d_kPoints):
    """
    Calculates the difference between two sets of points
    :param c_kPoints: First set of points
    :param d_kPoints: Second set of points

    :return: Array with distances between each pair of points
    """
    dist = np.zeros(np.shape(c_kPoints)[1])
    for i in range(np.shape(c_kPoints)[1]):
        dist[i] = numalg.norm(d_kPoints[:, i] - c_kPoints[:, i])

    return dist

def findClosestPointLinear(s_i, vCoords, vInd):
    """
    Finds the closest point to s_i on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param s_i: Given point
    :param vCoords: Coordinates of vertices
    :param vInd: Indices of vertex coordinates for each triangle

    :return: minPoint: The closest point to s_i on the surface mesh
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

        if l > 0 and u > 0 and l + u < 1:
            c_star = c
        elif l < 0:
            c_star = projectOnSegment(c, r, p)
        elif u < 0:
            c_star = projectOnSegment(c, p, q)
        elif l + u > 1:
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