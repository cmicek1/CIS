import numpy as np
import ICPmatching as icpm
import ICPfilereading as icpf
import Frame as fr
import math
import PointCloud as pc
import BoundingSphere as bs
import Triangle as tr
import CovTreeNode as ctn
import BoundingBoxTreeNode as bb

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

    F_reg = fr.Frame(np.identity(3), np.zeros([3, 1]))

    nIters = 0

    triangles = []

    for i in range(vIndices.shape[1]):
        t = tr.Triangle(pc.PointCloud(vCoords[:, vIndices[:, i]]))
        triangles.append(t)

    triangles = np.array(triangles)

    tree = ctn.CovTreeNode(triangles, vIndices.shape[1])

    spheres = bs.createBS(vCoords, vIndices)

 #   bbtree = bb.BoundingBoxTreeNode(triangles, len(triangles))

    old_pts = None
    c_kPoints = None

    while (nIters < 40):

        s_i = d_kPoints.transform(F_reg)

        if nIters == 0:
            old_pts = pc.PointCloud(s_i.data + 1)
            # c_kPoints = icpm.ICPmatch(s_i, vCoords, vIndices, spheres=spheres, oldpts=None, usetree=False, linear=True)

        c_kPoints = icpm.ICPmatch(s_i, vCoords, vIndices, spheres=spheres, tree=tree, oldpts=old_pts, usetree=True)

        # What the actual fuck
        old_pts = pc.PointCloud(s_i.data + np.random.random(s_i.data.shape))

        deltaF_reg = s_i.register(c_kPoints)

        F_regNew = deltaF_reg.compose(F_reg)

        if isClose(.0000001, F_reg, F_regNew):
            return c_kPoints, F_reg

        F_reg = F_regNew

        nIters += 1

    return c_kPoints, F_reg


def isClose(tolerance, F_reg, F_regNew):
    err = 0
    for i in range(0, 3):
        err += math.pow(F_reg.p[i] - F_regNew.p[i], 2)
        for j in range(0, 3):
            err += math.pow(F_reg.r[i][j] - F_regNew.r[i][j], 2)

    print err
    if err < tolerance:
        return True
    return False

