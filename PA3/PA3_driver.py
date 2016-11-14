import ICPfilereading as icpf
import ICPmatching as icp
import Frame as fr
import numpy as np
import numpy.linalg as numalg
import sys, os

def main(meshfile, bodyA, bodyB, sampleData, outfile):
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

    dist = np.zeros(np.shape(c_kPoints)[1])
    for i in range(np.shape(c_kPoints)[1]):
        dist[i] = numalg.norm(d_kPoints[:,i] - c_kPoints[:,i])

    writefile(d_kPoints, c_kPoints, dist, outfile)


def writefile(d_k, c_k, dist, outfile):
    f = open(outfile, 'w')
    h, t = os.path.split(outfile)
    f.write('{0} {1}\n'.format(np.shape(d_k)[1], t))

    for i in range(np.shape(d_k)[1]):
        f.write('{0:>8},{1:>9},{2:>9}'.format(format(d_k[0][i], '.2f'), format(d_k[1][i], '.2f'), format(d_k[2][i], '.2f')))
        f.write('{0:>13},{1:>9},{2:>9}'.format(format(c_k[0][i], '.2f'), format(c_k[1][i], '.2f'), format(c_k[2][i], '.2f')))
        f.write('{0:>10}\n'.format(format(dist[i], '.3f')))



    f.close()