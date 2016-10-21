import numpy as np
import scipy.linalg as scialg

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def transform(R, p, v):
    return R.dot(v) + p

def inv_transform(R, p, v):
    R = scialg.inv(R)
    p = -1 * scialg.inv(R).dot(p)
    return R.dot(v) + p