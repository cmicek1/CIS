import ICPfilereading as icpf
import ICPmatching as icp
import Frame as fr
import numpy as np

def main(meshfile, bodyA, bodyB, sampleData):
    """
    Super simple driver, @chris please make this better, I'm just using it for testing for now.
    :param meshfile:
    :param bodyA:
    :param bodyB:
    :return:
    """

    vCoords, vIndices = icpf.meshDef(meshfile)

    nledA, ledA, tipA = icpf.bodyDef(bodyA)
    nledB, ledB, tipB = icpf.bodyDef(bodyB)

    aFrames, bFrames = icpf.readSample(sampleData, nledA, nledB)

    d_kPoints = icp.findTipB(aFrames, ledA, tipA)

    I = fr.Frame(np.identity(3), np.zeros(3))

    s_i = icp.computeSamplePoints(d_kPoints, I)

    c_kPoints = icp.ICPmatch(s_i, vCoords, vIndices)