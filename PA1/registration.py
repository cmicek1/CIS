import sys
import numpy as np
import scipy.linalg as scialg
import scipy.optimize as sciopt
import frame_transformations as ft


def register(a, b):
    # NOTE: a, b are arrays of column vectors
    a_bar = np.mean(a, axis=0).T
    b_bar = np.mean(b, axis=0).T

    a_tilde = np.empty([len(a), 3])
    print a_tilde

    b_tilde = np.array([])

    for i in range(len(a)):
        a_tilde[i] = a[i] - a_bar
        #b_tilde.put((b[i] - b_bar), i)

    print a_tilde
    print b_tilde

    r = _lsq(a_tilde, b_tilde)

    p = b_bar - r.dot(a_bar)

    return r, p


def _lsq(a_tilde, b_tilde):

    #r = b_tilde.dot(scialg.pinv2(a_tilde))

    #r = r.dot(scialg.inv(scialg.sqrtm(r.T.dot(r))))

    r = (scialg.pinv(a_tilde)).dot(b_tilde)
    r = r.T

    print r

    print scialg.det(r)

    if scialg.det(r) != 1:
        raise ValueError('Error: Invalid rotation')

    epsilon = 1e-6

    start = True

    new_r = None

    while start:
        b_u = []
        for vec in range(b_tilde.shape[1]):
            b_u.append(np.dot(scialg.pinv(r), b_tilde[:, vec]))

        alpha = sciopt.leastsq(_f, np.array([0, 0, 0]), (a_tilde, b_u))
        delta_r = np.identity(3) + ft.skew(alpha)

        new_r = r.dot(delta_r)

        start = scialg.norm(new_r - r, 2) > epsilon

        r = new_r

    return r


def _f(x, *args):

    a_tilde = args[0]
    b_u = args[1]

    error = a_tilde - b_u + np.cross(x, a_tilde)
    return error
