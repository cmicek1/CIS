import PointCloud as pc
import pivot_cal as piv
import PA2_Prob1 as p1
import numpy as np
import scipy.misc as spmisc
import math
import Frame as fr


def distcal(calbody_file, calreadings_file, empivot_file):

    tracker_frames = pc.fromfile(calreadings_file)

    print len(tracker_frames)
    print np.shape(tracker_frames[0][2].data)
    #TODO: add things to concatanate all frames of C together to calculate the stuff

    c = []
    for k in range(len(tracker_frames)):
        c.append(tracker_frames[k][2])

    c_exp = p1.c_expected(calbody_file, calreadings_file)

    pPerFrame = np.shape(c[0].data)[1]    #points per frame
    nFrames = np.shape(c)[0]

    q_min, q_max, q_star_min, q_star_max = calc_q(c, c_exp, 0)
    u_s_star = normalize(pPerFrame, c_exp, 0, q_star_min, q_star_max)
    u_s = normalize(pPerFrame, c, 0, q_min, q_max)

    F_mat = f_matrix(u_s, 5)

    coeff_mat = solve_fcu(F_mat, u_s_star)

    print coeff_mat

    EMcorrect = correct(empivot_file, coeff_mat, q_min, q_max, q_star_min, q_star_max)


def correct(inputs, coeffs, q_min, q_max, q_star_min, q_star_max):

    inputcloud = pc.fromfile(inputs)
    print inputcloud




# def correctdistortion(coeffMat, q, q_min, q_max):

#
# #code below tries to correct distortion in an entire set of point clouds but i'm pretty sure it doesn't work ...
# #    pPerFrame = np.shape(c[0].data)[1]  # points per frame
# #    nFrames = np.shape(c)[0]
#
# #    U =[]
#
# #    for p in range(nFrames):
# #        U.append([c[p]])
#
# #    for k in range(nFrames):
# #        q_min, q_max, q_star_min, q_star_max = calc_q(c, c_exp, k)
# #        U[k][0].data = np.array((f_matrix(normalize(pPerFrame, c, k, q_min, q_max), 5).dot(coeffMat)))
# #        for i in range(pPerFrame):
# #            U[k][0].data[i] = U[k][0].data[i].dot(q_star_max - q_star_min) + q_star_min
# #        U[k][0].data = U[k][0].data.T
#
# #    return U


def normalize(pPerFrame, c, frame, q_min, q_max):

    u_s = np.zeros([pPerFrame, 3])

    for k in range(pPerFrame):
        for i in range(0, 3):
            u_s[k][i] = (c[frame].data[i][k] - q_min[i])/(q_max[i] - q_min[i])

    return u_s


def normalize_vector(c, q_min, q_max):

    u_s = (c - q_min)/(q_min - q_max)

    return u_s


def solve_fcu(F, U):

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