import numpy as np
import scipy.linalg as scialg
import scipy.optimize as sciopt
import frame_transformations as ft


def register(a, b):
    # NOTE: a, b are arrays of column vectors
    a_bar = np.mean(a, axis=1)
    b_bar = np.mean(b, axis=1)

    a_tilde = a - a_bar
    b_tilde = b - b_bar

    r = _lsq(a_tilde, b_tilde)

    p = b_bar - r.dot(a_bar)

    return r, p


def _lsq(a_tilde, b_tilde):

    r = b_tilde.dot(a_tilde.T.dot(a_tilde.T.dot(np.invert(a_tilde))))

    r = r.dot(scialg.inv(scialg.sqrtm(r.T.dot(r))))

    try:
        if scialg.det(r) != 1:
            raise ValueError
    except ValueError:
        print('Invalid rotation')

    epsilon = 1e-6

    start = True

    new_r = None

    while start:
        b_u = []
        for point in b_tilde:
            b_u.append(np.dot(np.invert(r), point))

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
