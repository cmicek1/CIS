import numpy as np
import ICPmatching as icpm
import ICPfilereading as icpf
import Frame as fr
import PointCloud as pc

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

    c_kPoints = iterativeFramePointFinder(vCoords, vIndices, d_kPoints)

    dist = icpm.calcDifference(c_kPoints, d_kPoints)

    return d_kPoints, c_kPoints, dist


def iterativeFramePointFinder(vCoords, vIndices, d_kPoints):

    F_reg = fr.Frame(np.identity(3), np.zeros([3, 1]))

    nIters = 0

    while ((not isClose()) and (nIters < 1)):

        s_i = d_kPoints.transform(F_reg)

        c_kPoints = icpm.ICPmatch(s_i, vCoords, vIndices)

        deltaF_reg = s_i.register(c_kPoints)

        F_reg = deltaF_reg.compose(F_reg)

        nIters += 1

    return c_kPoints

def isClose():
    return False