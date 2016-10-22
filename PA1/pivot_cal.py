import numpy as np
import scipy.linalg as scialg
import registration as reg


def pivot(G):
    #G is an array of arrays of points clouds, each with g points, representing different poses of the probe

    #step one: use the first set of point clouds to define the mean frame to perform the transformation

    print G

    g_first = G[0]

    print g_first

    G_0 = np.mean(g_first, axis=1, keepdims=True) #midpoint of observed points in first frame

    print G_0

    G_j = g_first - G_0

    print G_j

    R_I = np.zeros([6, 3*np.shape(G)[0]]) #matrix [Rn | -I] for least squares problem
    p_lstsq = np.zeros([3, np.shape(G)[0]]) #matrix [pn] for least squares problem

    #set rotational side of matrix for least squares problem

    #set identity side of matrix for least squares problem
    row_index = 0

    for n in range(np.shape(G)[0]):
        R_I[row_index][3] = -1
        R_I[row_index+1][4] = -1
        R_I[row_index+2][5] = -1
        row_index += 3

    print R_I



