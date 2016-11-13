import numpy as np
import PointCloud as pc

def bodyDef(fpath):
    """
    Used with input files ProblemX-BodyY.txt to return a PointCloud with the marker positions and another with the
    tip position.
    :param fpath:
    :return:
    """
    f = open(fpath, 'r')

    nMarkers = int(f.readline().split()[0])

    pcArray = np.zeros([3, nMarkers])

    for i in range(nMarkers):
        led = f.readline().split()
        print led
        for j in range(0, 3):
            pcArray[j][i] = np.float64(led[j])

    tip = np.zeros([3, 1])
    tipPos = f.readline().split()
    for j in range(0, 3):
        tip[j] = np.float64(tipPos[j])

    ledPC = pc.PointCloud(pcArray)
    tip = pc.PointCloud(tip)

    return ledPC, tip

def meshDef(fpath):
    """
    Extract two arrays with information about triangles from an input file.
    :param fpath: The file path to the input data.
    :type fpath: str

    :return: vCoords: An array that holds the coordinates of each vertex
    :return: vIndices: An array that holds the indices of the coordinates of the three vertices for each triangle

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

    vIndices = np.zeros([3, nTriangles])

    for i in range(nTriangles):
        triangle = f.readline().split()
        for j in range(0, 3):
            vIndices[j][i] = int(triangle[j])

    return vCoords, vIndices