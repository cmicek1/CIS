import PointCloud as pc
import pivot_cal as piv
import PA2_Prob1 as p1
import numpy as np
import numpy.linalg
import scipy.misc as spmisc
import math


def distcal(calbody_file, calreadings_file):

    tracker_frames = pc.fromfile(calreadings_file)

    c = []
    for k in range(len(tracker_frames)):
        c.append(tracker_frames[k][2])

    c_exp = p1.c_expected(calbody_file, calreadings_file)

    pPerFrame = np.shape(c[0].data)[1]    #points per frame
    nFrames = np.shape(c)[0]

    print pPerFrame
    print nFrames

    q_min, q_max, q_star_min, q_star_max = calc_q(c, c_exp, 0)
    u_s_star = normalize(pPerFrame, c_exp, 0, q_star_min, q_star_max)
    u_s = normalize(pPerFrame, c, 0, q_min, q_max)

    F_mat = f_matrix(u_s, 5)

    coeff_mat = solve_fcu(F_mat, u_s_star)

    corrC = correctdistortion(coeff_mat, c, c_exp)

    pivval = piv.pivotcal(corrC, 0)

    print pivval


def correctdistortion(coeffMat, c, c_exp):

    pPerFrame = np.shape(c[0].data)[1]  # points per frame
    nFrames = np.shape(c)[0]

    U =[]

    for p in range(nFrames):
        U.append(pc.PointCloud(c[p].data))

    for k in range(nFrames):
        q_min, q_max, q_star_min, q_star_max = calc_q(c, c_exp, k)
        U[k].data = (f_matrix(normalize(pPerFrame, c, k, q_min, q_max), 5).dot(coeffMat)).T
        for p in range(pPerFrame):
            for i in range(0, 3):
                U[k].data[i][p] = (U[k].data[:,p].dot((q_star_max - q_star_min)) + q_star_min)[i]

    print U[0].data, "\n"
    print c_exp[0].data, "\n"
    print c[0].data, "\n"

    return U
    #print U


def normalize(pPerFrame, c, frame, q_min, q_max):

    u_s = np.zeros([pPerFrame, 3])

    for k in range(pPerFrame):
        for i in range(0, 3):
            u_s[k][i] = (c[frame].data[i][k] - q_min[i])/(q_max[i] - q_min[i])

    return u_s


def solve_fcu(F, U):

    C = np.zeros([np.shape(F)[1], 3])

    C = np.linalg.lstsq(F, U)

    return C[0]


def calc_q(c, c_exp, frame):

    q_min = np.zeros(3)
    q_max = np.zeros(3)
    q_star_min = np.zeros(3)
    q_star_max = np.zeros(3)

    for i in range(0, 3):
        q_min[i] = min(c[frame].data[i])
        q_max[i] = max(c[frame].data[i])
        q_star_min[i] = min(c_exp[frame].data[i])
        q_star_max[i] = max(c_exp[frame].data[i])

    return q_min, q_max, q_star_min, q_star_max


def bernstein(N, k, u):
    B = spmisc.comb(N, k, exact=True) * math.pow(1 - u, N - k) * math.pow(u, k)
    return B


def f_ijk(N, i, j, k, u_x, u_y, u_z):
    return bernstein(N, i, u_x) * bernstein(N, j, u_y) * bernstein(N, k, u_z)


def f_matrix(u, deg):
    #deg is degree of berenstein polynomial, u is normalized distorted data

    nPoints = np.shape(u)[0]

    f_mat = np.zeros([nPoints, math.pow(deg + 1, 3)])

    for n in range(nPoints):
        c = 0
        for i in range(0, deg + 1):
            for j in range(0, deg + 1):
                for k in range(0, deg + 1):
                    f_mat[n][c] = f_ijk(deg, i, j, k, u[n][0], u[n][1], u[n][2])
                    c += 1

    return f_mat