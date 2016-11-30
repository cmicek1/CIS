import math
import collections
import numpy as np
import ICPmatching as icpm
import ICPfilereading as icpf
import Frame as fr
import PointCloud as pc
import Triangle as tr
import CovTreeNode as ctn


def completeICP(meshfile, bodyA, bodyB, sampleData):
    """
    :param meshfile: path to file that defines surface mesh
    :param bodyA: path to file that defines rigid body A
    :param bodyB: path to file that defines rigid body B
    :param sampleData: path to file that contains frames of sample data

    :type meshfile: str
    :type bodyA: str
    :type bodyB: str
    :type sampleData: str
    """

    vCoords, vIndices = icpf.meshDef(meshfile)

    nledA, ledA, tipA = icpf.bodyDef(bodyA)
    nledB, ledB, tipB = icpf.bodyDef(bodyB)

    aFrames, bFrames = icpf.readSample(sampleData, nledA, nledB)

    d_kPoints = icpm.findTipB(aFrames, bFrames, ledA, tipA, ledB)

    c_kPoints, F_reg = iterativeFramePointFinder(vCoords, vIndices, d_kPoints)

    s_k = d_kPoints.transform(F_reg)
    dist = icpm.calcDifference(s_k, c_kPoints)

    return s_k, c_kPoints, dist


def iterativeFramePointFinder(vCoords, vIndices, d_kPoints):
    """
    Finds registration transformation Freg between rigid body B and bone through iterative closest point finding.
    :param vCoords: coordinates of all vertices on mesh
    :param vIndices: indices of vertices for each triangle on mesh
    :param d_kPoints: starting positions of tip of rigid body A

    :type vCoords: np.array([np.float64]) 3 x N
    :type vIndices: np.array([np.float64]) 3 x M
    :type d_kPoints: pc.PointCloud

    :return c_kPoints: Transformed tip positions in bone coordinate system
    :return F_reg: Registration frame between bone and rigid body B

    :rtype c_kPoints: pc.PointCloud
    :rtype F_reg: fr.Frame
    """
    F_reg = fr.Frame(np.identity(3), np.zeros([3, 1]))

    nIters = 0

    triangles = []

    for i in range(vIndices.shape[1]):
        t = tr.Triangle(pc.PointCloud(vCoords[:, vIndices[:, i]]))
        triangles.append(t)

    triangles = np.array(triangles)

    print('Building tree...')
    tree = ctn.CovTreeNode(triangles, vIndices.shape[1])

    old_pts = None
    c_kPoints = None
    prev_error = collections.deque(maxlen=4)
    prev_error.append(0)
    thresh = 1

    print('\nStarting ICP:')
    while nIters < 40:

        s_i = d_kPoints.transform(F_reg)

        if nIters == 0:
            old_pts = pc.PointCloud(s_i.data + 1)

        c_kPoints = icpm.ICPmatch(s_i, vCoords, vIndices, tree=tree, oldpts=old_pts, usetree=True)

        old_pts = pc.PointCloud(s_i.data + np.random.random(s_i.data.shape) * thresh)

        deltaF_reg = s_i.register(c_kPoints)

        F_regNew = deltaF_reg.compose(F_reg)

        if isClose(.00000001, F_reg, F_regNew, prev_error):
            return c_kPoints, F_reg

        if len(prev_error) == prev_error.maxlen and not prev_error[0] == 0 and nIters % 5 == 0:
            if prev_error[3] > prev_error[0]:
                if thresh == 1:
                    thresh = 0
                else:
                    thresh = 1

        print('Iteration: ' + str(nIters) + ',   error = ' + str(prev_error[-1]))

        F_reg = F_regNew

        nIters += 1

    return c_kPoints, F_reg


def isClose(tolerance, F_reg, F_regNew, prev_error):
    """
    Tests if two frame transformations are within a given tolerance.
    :param tolerance: Maximum sum of squared differences between F_reg and F_regNew
    :param F_reg: First frame transformation
    :param F_regNew: Second frame transformation
    :param prev_error: Deque of previous errors, to determine whether to use noise to help convergence

    :type tolerance: np.float64
    :type F_reg: fr.Frame
    :type F_regNew: fr.Frame
    :type prev_error: collections.deque

    :return: True if difference is below tolerance, false otherwise

    :rtype: bool
    """
    err = 0
    for i in range(0, 3):
        err += math.pow(F_reg.p[i] - F_regNew.p[i], 2)
        for j in range(0, 3):
            err += math.pow(F_reg.r[i][j] - F_regNew.r[i][j], 2)

    if err < tolerance or abs(err - prev_error[0]) < tolerance:
        return True

    prev_error.append(err)
    return False

