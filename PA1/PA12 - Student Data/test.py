import numpy as np
import PointCloud as pc
import Frame


def test_reg():



def _rotation(angles):
    theta = angles[0]
    phi = angles[1]
    gamma = angles[2]

    rx = np.array([[1, 0, 0],
                   [0, np.cos(theta), -np.sin(theta)],
                   [0, np.sin(theta), np.cos(theta)]])

    ry = np.array([[np.cos(phi), 0, ]])