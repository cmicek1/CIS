#should include main method

import ICPmatching as icp

def pa3(meshfile, bodyA, bodyB):
    """
    Super simple driver, @chris please make this better, I'm just using it for testing for now.
    :param meshfile:
    :param bodyA:
    :param bodyB:
    :return:
    """

    vCoords, vIndices = icp.meshDef(meshfile)
    print vCoords
    print vIndices

    ledA, tipA = icp.bodyDef(bodyA)
    print ledA.data
    print tipA.data

    ledB, tipB = icp.bodyDef(bodyB)
    print ledB.data
    print tipB.data
