import numpy as np
import scipy.linalg as scialg
import PointCloud as pc
import pivot_cal as piv
import distortion as d


def test_reg(tolerance=1e4):
    """
    Tests registration of a random point cloud using a random rotation and translation. If the difference between the
    transformation components and the originals are withing tolerance, the registration is considered accurate and
    the tests pass.

    :param tolerance: The amount of allowed error between the generated and calculated transformation components
    :type tolerance: float

    :return: None
    """
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

    print('\nIs it a rotation (det = 1)?')
    assert np.abs(scialg.det(f.r) - scialg.det(r)) <= tolerance
    print(np.abs(scialg.det(f.r) - scialg.det(r)) <= tolerance)

    print('\nIs calculated p within tolerance?')
    assert np.all(np.abs(p - f.p) <= tolerance)
    print(np.all(np.abs(p - f.p) <= tolerance))

    print('\nRegistration tests passed!')


def test_pivot_cal(empivot, tolerance=1e-2):
    """
    Tests whether pivot calibration is correct by using the fact that all frame transformations F_G[k] from pointer to
    EM coordinates will map p_tip to p_dimple.

    :param empivot: The name/path of an empivot.txt file, containing frames of EM tracker data during pivot calibration
    :param tolerance: Error tolerance between calculated and predicted values. Default is 1e-2.

    :type empivot: str
    :type tolerance: float

    :return: None
    """
    print('\nTesting pivot calibration...')
    print('\nExtracting marker points:')
    G = pc.fromfile(empivot)
    for i in range(len(G)):
        print('\n')
        print(G[i][0].data)

    p_ans = piv.pivot(G, 0, True)

    print('\np_tip:')
    print(p_ans[0])

    print('\np_dimple:')
    print(p_ans[1])

    print('\nIf calibration is correct, all frame transformations F_G[k] from pointer to EM coordinates will map')
    print('p_tip to p_dimple. Assert that this is true:')

    print('\ntolerance = {}'.format(tolerance))

    for i in range(len(G)):
        passes = np.all(np.abs(
            p_ans[1].reshape((3, 1)) - pc.PointCloud(p_ans[0].reshape((3, 1))).transform(p_ans[2][i]).data) <=
                        tolerance)
        assert passes
        print('\nFrame: {} of {}: {}'.format(i + 1, len(G), True))

    print('\nCalibration tests passed!')


def test_f():
    """
    Sanity check for Bernstein polynomial basis matrix, F. Checks if shape is as expected, and that the calculation
    returns the expected basis for an input of ones.

    :return: None
    """
    print('Test Bernstein polynomial matrix F for distortion correction.')

    print('\nIf input is ones, last column should be 1, with all else 0:')
    u = np.ones((3, 10))
    print('\nu (3 x 10):')
    print(u)

    F = d.f_matrix(u, 5)
    print('\nF (5th degree):')
    print(F)

    print('\nFirst check size (should be N x 6**3)')
    expected = (3, 6**3)
    print('Shape expected: {}'.format(expected))
    print('Shape received: {}'.format(F.shape))
    print('Match?')
    assert expected == F.shape
    print(expected == F.shape)

    print('\nIs last column ones?')
    passes = np.all(F[:, -1] == 1)
    assert passes
    print(passes)

    print('\nIs the rest of F zero?')
    passes = np.all(F[:, 0:F.shape[1] - 1] == 0)
    assert passes
    print(passes)

    print('F tests pass!')


def _rotation(angles):
    """
    Helper method for generating a 3d rotation matrix

    :param angles: The angles to rotate with respect to the x, y, and z axes
    :type angles: numpy.array(numpy.float64) - shape (3, ) or (, 3) are acceptable

    :return: A 3 x 3 rotation matrix
    :rtype: numpy.array([numpy.float64][]) 3 x 3
    """
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
