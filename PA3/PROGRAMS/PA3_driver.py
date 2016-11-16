import ICPfilereading as icpf
import ICPmatching as icp
import Frame as fr
import numpy as np
import sys, os
import time
import testICP as test

def main():
    """
    Main method, takes command line arguments to either run tests or run program with a given data set (x-ddddd).
    """

    # Add 'test' command line option
    if str(sys.argv[1]) == 'test':
        if len(sys.argv) == 3:
            tolerance = float(sys.argv[2])
            # run tests with given tolerance
            test.testFindTipB(tolerance)
            test.testProjectOnSegment(tolerance)
            test.testFindClosestPoint(tolerance)
        else:
            # run tests with no given tolerance
            test.testFindTipB()
            test.testProjectOnSegment()
            test.testFindClosestPoint()
        sys.exit(0)

    # Parse arguments for regular execution
    directory = sys.argv[1]
    dataset = sys.argv[2]

    surface = os.getcwd() + directory + '/Problem3MeshFile.sur'
    bodyA = os.getcwd() + directory + '/Problem3-BodyA.txt'
    bodyB = os.getcwd() + directory + '/Problem3-BodyB.txt'
    testData = os.getcwd() + directory + '/PA3-' + dataset + '-SampleReadingsTest.txt'
    os.chdir("..")
    outname = os.getcwd() + '/OUTPUT/PA3-' + dataset + '-Output.txt'

    tofile(surface, bodyA, bodyB, testData, outname)

def tofile(meshfile, bodyA, bodyB, sampleData, outfile):
    """
    :param meshfile: path to file that defines surface mesh
    :param bodyA: path to file that defines rigid body A
    :param bodyB: path to file that defines rigid body B
    :param sampleData: path to file that contains frames of sample data
    :param outfile: path to file to write output to

    :type meshfile: str
    :type bodyA: str
    :type bodyB: str
    :type sampleData: str
    :type outfile: str
    """

    vCoords, vIndices = icpf.meshDef(meshfile)

    nledA, ledA, tipA = icpf.bodyDef(bodyA)
    nledB, ledB, tipB = icpf.bodyDef(bodyB)

    aFrames, bFrames = icpf.readSample(sampleData, nledA, nledB)

    d_kPoints = icp.findTipB(aFrames, bFrames, ledA, tipA, ledB)

    s_i = icp.computeSamplePoints(d_kPoints, fr.Frame(np.identity(3), np.zeros(3)))

    c_kPoints = icp.ICPmatch(s_i, vCoords, vIndices)

    dist = icp.calcDifference(c_kPoints, d_kPoints)

    writefile(d_kPoints, c_kPoints, dist, outfile)


def writefile(d_k, c_k, dist, outfile):
    """
    :param d_k: Position of tip in each frame
    :param c_k: Closest point on surface to tip in each frame
    :param dist: list of distances between tip and surface
    :param outfile: path to file to write output to

    :type d_k: pc.PointCloud
    :type c_k: pc.PointCloud
    :type dist: np.array([np.float64]) 1 x N
    :type outfile: str
    """
    f = open(outfile, 'w')
    h, t = os.path.split(outfile)

    d_k = d_k.data
    c_k = c_k.data

    f.write('{0} {1}\n'.format(np.shape(d_k)[1], t))

    for i in range(np.shape(d_k)[1]):
        f.write('{0:>8}{1:>9}{2:>9}'.format(format(d_k[0][i], '.2f'), format(d_k[1][i], '.2f'), format(d_k[2][i], '.2f')))
        f.write('{0:>13}{1:>9}{2:>9}'.format(format(c_k[0][i], '.2f'), format(c_k[1][i], '.2f'), format(c_k[2][i], '.2f')))
        f.write('{0:>10}\n'.format(format(dist[i], '.3f')))

    f.close()

if __name__ == '__main__':
    main()
