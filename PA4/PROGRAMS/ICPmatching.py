import numpy as np
import numpy.linalg as numalg
import PointCloud as pc

def findTipB(aFrames, bFrames, ledA, tipA, ledB):
    """
    Finds the positions of the tip of pointer A with respect to the B rigid body for each sample frame.
    :param aFrames: Sample frames of A pointer
    :param bFrames: Sample frames of B rigid body
    :param ledA: Position of LEDs on A pointer in calibration frame
    :param tipA: Position of tip with respect to A pointer
    :param ledB: Position of LEDs on B rigid body in calibration frame
    :param tipB: Position of attachment of B to bone in B coordinate system

    :type aFrames: [pc.PointCloud]
    :type bFrames: [pc.PointCloud]
    :type ledA: pc.PointCloud
    :type tipA: pc.PointCloud
    :type ledB: pc.PointCloud
    :type tipB: pc.PointCloud

    :return: d_ks: Tip positions with respect to B rigid body in each data frame
    :rtype: pc.PointCloud
    """

    d_ks = np.zeros([3, len(aFrames)])

    for i in range(len(aFrames)):
        regA = ledA.register(aFrames[i])
        regB = ledB.register(bFrames[i])
        reg = regB.inv.compose(regA)
        d_k = reg.r.dot(tipA.data) + reg.p
        for j in range(0, 3):
            d_ks[j][i] = d_k[j]

    return pc.PointCloud(d_ks)


def findClosestPoint(s_i, vCoords, vInd, spheres):
    """
    Finds the closest point to s_i on a mesh defined by vCoords (coordinates of vertices) and vInd (vertex indices for
    each triangle.
    :param s_i: Given point
    :param vCoords: Coordinates of vertices
    :param vInd: Indices of vertex coordinates for each triangle
    :param spheres: list of bounding spheres around each triangle

    :type s_i: np.array([np.float64]) 3 x 1
    :type vCoords: np.array([np.float64]) 3 x N
    :type vInd: np.array([np.float64]) 3 x M
    :type spheres: [bs.BoundingSphere]

    :return: The closest point to s_i on the surface mesh
    :rtype: np.array([np.float64]) 3 x 1
    """
    p = vCoords[:, int(vInd[0][0])]
    q = vCoords[:, int(vInd[1][0])]
    r = vCoords[:, int(vInd[2][0])]

    minPoint = minPointonTriangle(s_i, p, q, r)
    dist = numalg.norm(s_i - minPoint)

    for i in range(1, np.shape(vInd)[1]):
        if numalg.norm(s_i - spheres[i].c) - spheres[i].r < numalg.norm(s_i - minPoint):
            # check carefully
            p = vCoords[:, int(vInd[0][i])]
            q = vCoords[:, int(vInd[1][i])]
            r = vCoords[:, int(vInd[2][i])]

            c_star = minPointonTriangle(s_i, p, q, r)

            d = numalg.norm(s_i - c_star)

            if d < dist:
                dist = d
                minPoint = c_star
                print spheres[i].c
                print spheres[i].r

    return minPoint


def minPointonTriangle(s_i, p, q, r):
    """
    Returns the closest point to s_i on a triangle defined by vertices p, q, r
    :param s_i: point to search for closest point to
    :param p: first vertex of triangle
    :param q: second vertex of triangle
    :param r: third vertex of triangle

    :type s_i: np.array([np.float64]) 3 x 1
    :type p: np.array([np.float64]) 3 x 1
    :type q: np.array([np.float64]) 3 x 1
    :type r: np.array([np.float64]) 3 x 1

    :return: Point on triangle with minimum distance to s_i
    :rtype: np.array([np.float64]) 3 x 1
    """
    A = np.zeros([3, 2])

    for j in range(0, 3):
        A[j][0] = q[j] - p[j]
        A[j][1] = r[j] - p[j]

    soln = numalg.lstsq(A, s_i - p)
    l = soln[0][0]
    u = soln[0][1]

    c = p + l * (q - p) + u * (r - p)
    c_star = np.zeros(3)

    if l >= 0 and u >= 0 and l + u <= 1:
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

    :type c: np.array([np.float64]) 3 x 1
    :type p: np.array([np.float64]) 3 x 1
    :type q: np.array([np.float64]) 3 x 1

    :return: the projection of c onto line segment p_q
    :rtype: np.array([np.float64]) 3 x 1
    """
    l = np.float64(((c-p).dot(q-p))) / np.float64(((q-p).dot(q-p)))
    lambda_star = max(0, min(l, 1))
    c_star = p + lambda_star*(q-p)
    return c_star


def ICPmatch(s_i, vCoords, vInd, spheres=None, tree=None, oldpts=None, linear=False, usetree=True):
    """
    Finds the closest point on a given surface for each point in a given PointCloud
    :param s_i: PointCloud of points to find closest point
    :param vCoords: Coordinates of each vertex on surface
    :param vInd: Indices of vertices for each triangle on surface
    :param spheres: List of bounding spheres around triangles on surface.
    :param tree: tree data strucutre to search from (optional)
    :param oldpts: old closest points (optional)
    :param linear: true if linear search should be performed
    :param usetree: true if tree search should be used

    :type s_i: PointCloud.PointCloud
    :type vCoords: np.array([np.float64]) 3 x N
    :type vInd: np.array([np.float64]) 3 x M
    :type spheres: [bs.BoundingSphere]
    :type tree: tree.CovTreeNode
    :type oldpts: pc.PointCloud
    :type linear: bool
    :type usetree: bool

    :return: closest point on surface to each point in s_i
    :rtype: pc.PointCloud
    """
    c_ij = np.zeros([3, np.shape(s_i.data)[1]])
    old = None
    closest_pts = None
    if usetree:
        old = oldpts
        closest_pts = old
    for i in range(np.shape(s_i.data)[1]):
        if linear:
            c = findClosestPointLinear(s_i.data[:, i], vCoords, vInd)
            c_ij[:, i] = c[:]
        elif usetree:
            old_i = closest_pts.data[:, i]
            dist = np.linalg.norm(old_i - s_i.data[:, i])
            closest = [old_i]
            bound = [dist]
            tree.FindClosestPoint(s_i.data[:, i], bound, closest)
            c_ij[:, i] = closest[0][:]
        else:
            c = findClosestPoint(s_i.data[:, i], vCoords, vInd, spheres)
            c_ij[:, i] = c[:]
    return pc.PointCloud(c_ij)


def calcDifference(c_kPoints, d_kPoints):
    """
    Calculates the difference between two sets of points
    :param c_kPoints: First set of points
    :param d_kPoints: Second set of points

    :type c_kPoints: pc.PointCloud
    :type d_kPoints: pc.PointCloud

    :return: Array with distances between each pair of points
    rtype: np.array([np.float64]) 1 x N
    """
    dist = np.zeros(np.shape(c_kPoints.data)[1])
    for i in range(np.shape(c_kPoints.data)[1]):
        dist[i] = numalg.norm(d_kPoints.data[:, i] - c_kPoints.data[:, i])

    return dist


def findClosestPointLinear(s_i, vCoords, vInd):
    """
    Performs linear search through all triangles to find the closest point on the surface to s_i.
    NOTE: our solution does not use this method, however, we included it for testing the bounding spheres method
    and for performance comparison.
    :param s_i: Given point
    :param vCoords: Coordinates of vertices
    :param vInd: Indices of vertex coordinates for each triangle

    :type s_i: np.array([np.float64]) 3 x 1
    :type vCoords: np.array([np.float64]) 3 x N
    :type vInd: np.array([np.float64]) 3 x M

    :return: The closest point to s_i on the surface mesh
    :rtype: np.array([np.float64]) 3 x 1
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

        if l >= 0 and u >= 0 and l + u <= 1:
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