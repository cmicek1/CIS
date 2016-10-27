import PointCloud as pc
import pivot_cal as piv
import PA2_Prob1 as p1
import numpy as np
import scipy.misc as spmisc
import math


def distcal(calbody_file, calreadings_file, empivot_file):
    """
    Calculates the coefficient matrix of the distortion correction file and applies it to a set of EM pivot calibration
    frames to return the corrected pivot calibration.

    :param calbody_file: File name/path for the calibration object data file
    :param calreadings_file: File name/path for the readings from the trackers
    :param empivot_file: File name/path for EM pivot poses

    :type calbody_file: str
    :type emfiducialss: str
    :type empivot_file: str

    :return: pivotanswer: the corrected pivot calibration
    :return: coeff_mat: matrix of the coefficients for dewarping a data set
    :return: q_min: vector of minimum value for each coordinate in experimental data set
    :return: q_max: vector of maximum value for each coordinate in experimental data set
    :return: q_star_min: vector of minimim value for each coordinate in expected data set
    :return: q_star_max: vector of maximum value for each coordinate in expected data set

    :rtype coeffs: numpy.array([numpy.float64][]) degree**3 x 3
    :rtype q_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :rtype q_max: numpy.array(numpy.float64) shape (3,) or (, 3)
    :rtype q_star_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :rtype q_star_max: numpy.array(numpy.float64) shape (3,) or (, 3)
    """

    tracker_frames = pc.fromfile(calreadings_file)

    c = []
    for k in range(len(tracker_frames)):
        c.append(tracker_frames[k][2])

    c_exp = p1.c_expected(calbody_file, calreadings_file)

    pPerFrame = np.shape(c[0].data)[1]    #points per frame
    nFrames = np.shape(c)[0]

    concatc = c[0].data
    concatc_exp = c_exp[0].data
    for i in range(1, nFrames):
        concatc = np.concatenate((concatc, c[i].data), axis=1)
        concatc_exp = np.concatenate((concatc_exp, c_exp[i].data), axis=1)

    q_min, q_max, q_star_min, q_star_max = calc_q(concatc, concatc_exp)
    u_s_star = normalize(pPerFrame*nFrames, concatc_exp, q_star_min, q_star_max)
    u_s = normalize(pPerFrame*nFrames, concatc, q_min, q_max)

    F_mat = f_matrix(u_s, 5)

    coeff_mat = solve_fcu(F_mat, u_s_star)

    EMcorrect = correct(empivot_file, coeff_mat, q_min, q_max, q_star_min, q_star_max)

    pivotanswer = piv.pivot(EMcorrect, 0)

    return pivotanswer, coeff_mat, q_min, q_max, q_star_min, q_star_max


def correct(inputs, coeffs, q_min, q_max, q_star_min, q_star_max):
    """
    Performs dewarping on PointClouds extracted from a given input file.

    :param inputs: file with point clouds to be dewarped
    :param coeffs: coefficient matrix for distortion correction
    :param q_min: vector of minimum value for each coordinate in experimental data set
    :param q_max: vector of maximum value for each coordinate in experimental data set
    :param q_star_min: vector of minimim value for each coordinate in expected data set
    :param q_star_max: vector of maximum value for each coordinate in expected data set

    :type inputs: str
    :type coeffs: numpy.array([numpy.float64][]) degree**3 x 3
    :type q_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_max: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_max: numpy.array(numpy.float64) shape (3,) or (, 3)

    :return: outputcloud: dewarped set of point clouds extracted from input file
    :rtype [PointCloud.PointCloud]

    """
    inputcloud = pc.fromfile(inputs)

    outputcloud = []

    for p in range(len(inputcloud)):
        outputcloud.append(inputcloud[p])

    points = np.shape(inputcloud[0][0].data)[1]

    for k in range(len(inputcloud)):
        outputcloud[k][0].data = f_matrix(normalize(points, inputcloud[k][0].data, q_min, q_max), 5).dot(coeffs)
        for i in range(points):
            for j in range(3):
                outputcloud[k][0].data[i, j] = (outputcloud[k][0].data[i, j])*(q_star_max[j] - q_star_min[j]) + \
                                               q_star_min[j]

        outputcloud[k][0].data = outputcloud[k][0].data.T

    return outputcloud


def normalize(pPerFrame, c, q_min, q_max):
    """

    :param pPerFrame: Points per frame of data
    :param c: matrix of data to normalize
    :param q_min: vector of minimum value for each coordinate in experimental data set
    :param q_max: vector of maximum value for each coordinate in experimental data set

    :type pPerFrame: Integer
    :type c: numpy.array(numpy.float64) shape (pPerFrame, 3)
    :type q_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_max: numpy.array(numpy.float64) shape (3,) or (, 3)

    :return: u_s: normalized matrix of data
    :type u_s: numpy.array(numpy.float64) shape (pPerFrame, 3)

    """

    u_s = np.zeros([pPerFrame, 3])

    for k in range(pPerFrame):
        for i in range(0, 3):
            u_s[k][i] = (c[i][k] - q_min[i])/(q_max[i] - q_min[i])

    return u_s


def normalize_vector(c, q_min, q_max):

    u_s = (c - q_min)/(q_min - q_max)

    return u_s


def solve_fcu(F, U):

    C = np.linalg.lstsq(F, U)

    return C[0]


def calc_q(c, c_exp):

    q_min = np.zeros(3)
    q_max = np.zeros(3)
    q_star_min = np.zeros(3)
    q_star_max = np.zeros(3)

    for i in range(0, 3):
        q_min[i] = min(c[i])
        q_max[i] = max(c[i])
        q_star_min[i] = min(c_exp[i])
        q_star_max[i] = max(c_exp[i])

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