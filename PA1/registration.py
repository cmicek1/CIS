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

    sigma = np.sum(np.apply_along_axis(scialg.norm, 0, a_tilde)) / np.sum(np.apply_along_axis(scialg.norm, 0, b_tilde))

    H = (sigma * a).dot(b.T)

    u, s, v_t = scialg.svd(H)

    u = u.T
    v_t = v_t.T

    correction = np.identity(v_t.shape[1])
    correction[-1, -1] = scialg.det(v_t.dot(u))

    r = v_t.dot(correction.dot(u))

    p = b_bar - r.dot(a_bar)

    return r, p, sigma
