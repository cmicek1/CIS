import numpy as np
import PointCloud as pc

def readSample(fpath, nA, nB):
    """
    Reads frames of positions of led markers on bodies A and B and creates 2 lists of pointClouds.
    :param fpath: Filepath containing sample readings
    :param nA: number of LED markers on A
    :param nB: number of LED markers on B

    :type fpath: str
    :type nA: int
    :type nB: int

    :return: PointClouds representing positions of LEDs on A in each frame
    :return: PointClouds representing positions of LEDs on B in each frame

    :rtype: [pc.PointCloud]
    :rtype: [pc.PointCloud]
    """
    f = open(fpath, 'r')

    line1 = f.readline()
    nMarkers = int(line1.split()[0].replace(',',''))
    nSamples = int(line1.split()[1].replace(',',''))

    aFrames = []
    bFrames = []

    for i in range(nSamples):
        aCloud = np.zeros([3, nA])
        bCloud = np.zeros([3, nB])
        for j in range(nA):
            point = f.readline().split()
            for k in range(0, 3):
                aCloud[k][j] = np.float64(point[k].replace(',',''))
        for j in range(nB):
            point = f.readline().split()
            for k in range(0, 3):
                bCloud[k][j] = np.float64(point[k].replace(',', ''))
        for j in range(nMarkers - nA - nB):
            f.readline()

        aFrames.append(pc.PointCloud(aCloud))
        bFrames.append(pc.PointCloud(bCloud))

    return aFrames, bFrames


def bodyDef(fpath):
    """
    Used with input files ProblemX-BodyY.txt to return a PointCloud with the marker positions and another with the
    tip position.
    :param fpath: path to fine containing definition of rigid body
    :type fpath: str

    :return: The number of LED markers on the rigid body
    :return: PointCloud with the positions of the LEDs
    :return: PointCloud with position of the tip

    :rtype: int
    :rtype: pc.PointCloud
    :rtype: pc.PointCloud
    """
    f = open(fpath, 'r')

    nMarkers = int(f.readline().split()[0])

    pcArray = np.zeros([3, nMarkers])

    for i in range(nMarkers):
        led = f.readline().split()
        for j in range(0, 3):
            pcArray[j][i] = np.float64(led[j])

    tip = np.zeros([3, 1])
    tipPos = f.readline().split()
    for j in range(0, 3):
        tip[j] = np.float64(tipPos[j])

    ledPC = pc.PointCloud(pcArray)
    tip = pc.PointCloud(tip)

    return nMarkers, ledPC, tip

def meshDef(fpath):
    """
    Extract two arrays with information about triangles from an input file.
    :param fpath: The file path to the input data.
    :type fpath: str

    :return: An array that holds the coordinates of each vertex
    :return: An array that holds the indices of the coordinates of the three vertices for each triangle

    :rtype: np.array(float64) 3 x nNvertices
    :rtype: np.array(int) 3 x nTriangles
    """

    f = open(fpath, 'r')

    nVertices = int(f.readline())

    vCoords = np.zeros([3, nVertices])

    for i in range(nVertices):
        vertex = f.readline().split()
        for j in range(0, 3):
            vCoords[j][i] = np.float64(vertex[j])

    nTriangles = int(f.readline())

    vIndices = np.zeros([3, nTriangles], dtype=int)

    for i in range(nTriangles):
        triangle = f.readline().split()
        for j in range(0, 3):
            vIndices[j][i] = int(triangle[j])

    return vCoords, vIndices