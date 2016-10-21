import numpy as np
import scipy.linalg as scialg
import registration as reg

def pivot(g):
    #g is a set of pivot calibration points
    G_0 = np.mean(g, axis=1, keepdims=True)

    print G_0

    G_j = g - G_0

    print G_j

    R, p = reg.register(g, G_j)

    print R
    print p




