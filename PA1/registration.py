import sys
import numpy as np
import scipy.linalg as scialg
import scipy.optimize as sciopt
import frame_transformations as ft


def register(a, b):
    # NOTE: a, b are arrays of column vectors
    a_bar = np.mean(a, axis=1, keepdims=True)
    b_bar = np.mean(b, axis=1, keepdims=True)

    a_tilde = a - a_bar
    b_tilde = b - b_bar

    # Method using SVD to directly solve for R

    # H = a.dot(b.T)
    #
    # u, s, v_t = scialg.svd(H)
    #
    # u = u.T
    # v_t = v_t.T
    #
    # correction = np.identity(v_t.shape[1])
    # correction[-1, -1] = scialg.det(v_t.dot(u))
    #
    # r = v_t.dot(correction.dot(u))

    r = _lsq(a_tilde, b_tilde)

    p = b_bar - r.dot(a_bar)

    return r, p


def _lsq(a_tilde, b_tilde):

    r = b_tilde.T.dot(scialg.pinv2(a_tilde.T))
    r, q = np.linalg.qr(r)

    if (scialg.det(r) < .9999999) or (scialg.det(r) > 1.000001):
        print scialg.det(r)
        raise ValueError('Error: Invalid rotation')

    epsilon = 1e-6

    start = True

    new_r = None

    while start:
        b_u = np.empty([len(b_tilde), 3])
        for i in range(len(b_tilde)):
            b_u[i] = (np.dot(scialg.pinv(r), b_tilde[i]))

        alpha_guess = np.array([0, 0, 0])

        alpha = sciopt.minimize(_f, alpha_guess, args = (a_tilde, b_u), options={'disp': False})
        #solution is in alpha.x

        delta_r = np.identity(3) + ft.skew(alpha.x)

        new_r = r.dot(delta_r)
        print new_r

        start = scialg.norm(new_r - r, 2) > epsilon

        r = new_r

    return r


def _f(x, *args):
    a_tilde = args[0]
    b_u = args[1]
    error = 0

    for i in range(len(a_tilde)):
        eadd = (a_tilde[i] - b_u[i] + np.cross(x, a_tilde[i]))
        error = error + eadd.dot(eadd)

    return error
