import PointCloud as pc
import Frame as fr
import PA2_Prob1 as p1
import numpy as np
import scipy.misc as spmisc
import scipy.linalg as scialg
import math

def distCal(calbody_file, calreadings_file):

    tracker_frames = pc.fromfile(calreadings_file)

    c = []
    for k in range(len(tracker_frames)):
        c.append(tracker_frames[k][2])

    c_exp = p1.c_expected(calbody_file, calreadings_file)

    pPerFrame = np.shape(c[0].data)[1]    #points per frame

    q_min, q_max, q_star_min, q_star_max = calc_q(c, c_exp)

    u_s_star = np.zeros([pPerFrame, 3])
    u_s = np.zeros([pPerFrame, 3])

    for k in range(pPerFrame):
        for i in range(0, 3):
            u_s_star[k][i] = (c[0].data[i][k] - q_min[i])/(q_max[i] - q_min[i])
            u_s[k][i] = (c_exp[0].data[i][k] - q_star_min[i]) / (q_star_max[i] - q_star_min[i])

    F_mat = f_matrix(u_s, 5)

    #at this point want F_mat * Coeff mat = u_s_star
    #i don't know how to do this but i'm gonna try with svd

   # H = F_mat.T.dot(u_s_star)

    #u, s, v_t = scialg.svd(H)

    #u = u.T
    #v_t = v_t.T

    #correction = np.identity(v_t.shape[1])
    #correction[-1, -1] = scialg.det(v_t.dot(u))

    #coeffMat = v_t.dot(correction.dot(u))

    #print coeff_mat

def calc_q(c, c_exp):

    q_min = q_max = q_star_min = q_star_max = np.zeros(3)

    for i in range(0, 3):
        q_min[i] = min(c[0].data[i])
        q_max[i] = max(c[0].data[i])
        q_star_min[i] = min(c_exp[0].data[i])
        q_star_max[i] = max(c_exp[0].data[i])

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