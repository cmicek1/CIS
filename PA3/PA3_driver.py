#should include main method

import ICPfilereading as icpf
import ICPmatching as icp

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

    d_kPoints = icp.findTipB(aFrames, bFrames, tipA)

    #in problem 4 we need to do something else to d_kPoints before finding closest points on triangles (s_i)

    c_kPoints = icp.findClosestPoint(d_kPoints, vCoords, VIndices)