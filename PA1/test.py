import numpy as np
import PointCloud as pc


def test_reg(tolerance=None):
    print('Testing registration...')
    angles = np.random.uniform(0, 2 * np.pi, (3,))
    a = np.random.uniform(0, 10, (3, 10))
    print('\na =')
    print(a)
    r = _rotation(angles)
    print('\nR =')
    print(r)
    p = np.random.uniform(0, 10, (3, 1))
    print('\np =')
    print(p)
    if not tolerance:
        tolerance = 1e-4
    print('\ntolerance = ' + str(tolerance))
    b = r.dot(a) + p
    print('\nb =')
    print(b)

    print('\nCalculating F_AB...')
    f = pc.PointCloud(a).register(pc.PointCloud(b))
    print('Done!')

    print('\nR_AB =')
    print(f.r)

    print('\np_AB =')
    print(f.p)

    print('\nIs calculated R within tolerance?')
    assert np.all(np.abs(r - f.r) <= tolerance)
    print(np.all(np.abs(r - f.r) <= tolerance))

    print('\nIs calculated p within tolerance?')
    assert np.all(np.abs(p - f.p) <= tolerance)
    print(np.all(np.abs(p - f.p) <= tolerance))

    print('\nRegistration tests passed!')


def _rotation(angles):
    theta = angles[0]
    phi = angles[1]
    gamma = angles[2]

    rx = np.array([[1, 0, 0],
                   [0, np.cos(theta), -np.sin(theta)],
                   [0, np.sin(theta), np.cos(theta)]])

    ry = np.array([[np.cos(phi), 0, np.sin(phi)],
                   [0, 1, 0],
                   [-np.sin(phi), 0, np.cos(phi)]])

    rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                   [np.sin(gamma), np.cos(gamma), 0],
                   [0, 0, 1]])

    return rx.dot(ry.dot(rz))