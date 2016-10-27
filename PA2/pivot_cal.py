import numpy as np
import PointCloud as pc


def pivot(G, nframe, debug=False):
    """
    Performs pivot calibration given a list of PointClouds per frame of EM calibration data, and calculates the
    pointer tip location in pointer and EM coordinates

    :param G: A list of PointCloud lists, each list containing points from each frame of EM calibration data
    :param nframe: Which index of the lists in G to expect EM tracking data
    :param debug: Optional debug flag; when checked also returns a list of all Frame transformations from pointer to
                  EM coordinates calculated

    :type G: [PointCloud.PointCloud][]
    :type nframe: int
    :type debug: bool

    :return: A tuple containing (p_cal, p_piv[, Fs]), where
             p_cal is p_tip, the pointer tip in pointer coordinates;
             p_piv is P_dimple, the pointer tip in EM tracker coordinates;
             Fs is a list of Frame transformations mapping each frame of pointer points to EM tracker points

    :rtype: (numpy.array([numpy.float64]), numpy.array([numpy.float64]) [, [Frame.Frame]])
    """
    # G is a list of point clouds, each with g points, representing different poses of the probe

    g_first = G[0][nframe].data

    G_0 = np.mean(g_first, axis=1, keepdims=True)  # midpoint of observed points in first frame

    G_j = g_first - G_0

    n_frames = len(G)

    R_I = np.zeros([3 * n_frames, 6])  # matrix [Rn | -I] for least squares problem

    p_lstsq = np.zeros([n_frames * 3])  # matrix [pn] for least squares problem

    Fs = []

    # set rotational side of matrix for least squares problem
    for k in range(n_frames):
        F = pc.PointCloud(G_j).register(G[k][nframe])
        if debug:
            Fs.append(F)
        R, p = F.r, F.p
        for i in range(0, 3):
            R_I[k * 3][i] = R[0][i]
            R_I[k * 3 + 1][i] = R[1][i]
            R_I[k * 3 + 2][i] = R[2][i]
            p_lstsq[k * 3 + i] = -p[i]

    # set identity side of matrix for least squares problem
    for n in range(n_frames):
        R_I[n * 3][3] = -1
        R_I[n * 3 + 1][4] = -1
        R_I[n * 3 + 2][5] = -1

    # solve as an Ax=B problem (R_I * [pcal ppiv] = p_leastsq)
    p_soln = np.linalg.lstsq(R_I, p_lstsq)

    p_cal = np.array(p_soln[0][0:3])
    p_piv = np.array(p_soln[0][3:6])
    if debug:
        return p_cal, p_piv, Fs
    return p_cal, p_piv
