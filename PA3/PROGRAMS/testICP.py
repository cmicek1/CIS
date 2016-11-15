import numpy as np
import PointCloud as pc
import ICPmatching as icpm


def testFindTipB(tolerance=1e-4):
    """
    Test finding the tip position of body B from the tip position of the A body. If the generated and calculated
    positions are within tolerance of one another, the test passes.

    :param tolerance: The minimum error between created and computed values for the test to pass.

    :type tolerance: float

    :return: None
    """

    print('Testing registration of pointer tip wrt body B...')

    print('Generate random A_tip, and A_k, B_k points')
    angles1 = np.random.uniform(0, 2 * np.pi, (3,))
    angles2 = np.random.uniform(0, 2 * np.pi, (3,))

    a_tip = np.random.uniform(0, 10, (3, 1))

    a_led = np.random.uniform(0, 10, (3, 10))
    b_led = np.random.uniform(0, 10, (3, 10))

    r1 = _rotation(angles1)
    r2 = _rotation(angles2)

    p1 = np.random.uniform(0, 10, (3, 1))
    p2 = np.random.uniform(0, 10, (3, 1))

    a_frames = r1.dot(a_led) + p1
    b_frames = r2.dot(b_led) + p2

    print('Generate random transformations to a_k and b_k, and generate d_k for one frame')
    print('\nd_k =')
    d_k = np.linalg.inv(r2).dot(r1).dot(a_tip) + np.linalg.inv(r2).dot(p1 - p2)
    print(d_k)

    print('\nCalculate d_k using function:\ntest_d_k =')
    test_d_k = icpm.findTipB([pc.PointCloud(a_frames)], [pc.PointCloud(b_frames)], pc.PointCloud(a_led),
                             pc.PointCloud(a_tip), pc.PointCloud(b_led))
    print(test_d_k)

    print('\nAssert difference between created and calculated d_k is less than tolerance = {}'.format(tolerance))
    print('\nIs calculated d_k within tolerance?')
    assert np.all(np.abs(d_k - test_d_k.data) <= tolerance)
    print(np.all(np.abs(d_k - test_d_k.data) <= tolerance))
    print('Find d_k test passed!')


def testProjectOnSegment(tolerance=1e-4):
    """
    Test projection of a point onto a line segment.

    :param tolerance: Minimum error between created and calculated result for tests to pass

    :type tolerance: float

    :return: None
    """
    print('\nTesting projection of a point onto a line segment...')
    print('\nSimple case: Project on vertical line segment')
    print('\nDefine line segment from p to q:\np =')
    p = np.array([0, 10, 0])
    print(p)
    q = np.array([0, -10, 0])
    print('\nq =')
    print(q)
    print('\nNow make a test c to project:')
    print('\nCase 1: c is to the left or right of the segment in the plane:')
    print('c =')
    c = np.array([-2, 7, 0])
    print(c)
    c_exp = np.array([0, 7, 0])
    print('Expected c =')
    print(c_exp)
    print('Calculated c =')
    c_star = icpm.projectOnSegment(c, p, q)
    print(c_star)
    print('\nAre c_exp and c_calc within tolerance?')
    assert np.all(np.abs(c_exp - c_star) <= tolerance)
    print(np.all(np.abs(c_exp - c_star) <= tolerance))

    print('\nCase 2: c is on the line defined by the line segment, but not contained by it:')
    print('c =')
    c = np.array([0, 12, 0])
    print(c)
    c_exp = np.array([0, 10, 0])
    print('Expected c =')
    print(c_exp)
    print('Calculated c =')
    c_star = icpm.projectOnSegment(c, p, q)
    print(c_star)
    print('\nAre c_exp and c_calc within tolerance?')
    assert np.all(np.abs(c_exp - c_star) <= tolerance)
    print(np.all(np.abs(c_exp - c_star) <= tolerance))

    print('\nCase 3: c is on the line segment:')
    print('c =')
    c = np.array([0, 1, 0])
    print(c)
    c_exp = np.array([0, 1, 0])
    print('Expected c =')
    print(c_exp)
    print('Calculated c =')
    c_star = icpm.projectOnSegment(c, p, q)
    print(c_star)
    print('\nAre c_exp and c_calc within tolerance?')
    assert np.all(np.abs(c_exp - c_star) <= tolerance)
    print(np.all(np.abs(c_exp - c_star) <= tolerance))

    print('\nCase 4: c is out of the plane of the segment, past the endpoints of the segment:')
    print('c =')
    c = np.array([1, -13, 5])
    print(c)
    c_exp = np.array([0, -10, 0])
    print('Expected c =')
    print(c_exp)
    print('Calculated c =')
    c_star = icpm.projectOnSegment(c, p, q)
    print(c_star)
    print('\nAre c_exp and c_calc within tolerance?')
    assert np.all(np.abs(c_exp - c_star) <= tolerance)
    print(np.all(np.abs(c_exp - c_star) <= tolerance))

    print('\nCase 5: c is out of the plane of the segment, but within the bounds of the endpoints of the segment:')
    print('c =')
    c = np.array([1, 6, 5])
    print(c)
    c_exp = np.array([0, 6, 0])
    print('Expected c =')
    print(c_exp)
    print('Calculated c =')
    c_star = icpm.projectOnSegment(c, p, q)
    print(c_star)
    print('\nAre c_exp and c_calc within tolerance?')
    assert np.all(np.abs(c_exp - c_star) <= tolerance)
    print(np.all(np.abs(c_exp - c_star) <= tolerance))


def testFindClosestPoint(tolerance=1e-4):
    """
    Tests finding closest point on a mesh of triangles

    :param tolerance: Max allowed difference between result and prediction.

    :type tolerance: float

    :return: None
    """
    print('\nTesting FindClosestPoint...')
    print('\nSimple case: Closest point to a single triangle')
    v_coords = np.array([[0, 0, 4],
                        [1, 3, 2],
                        [0, 0, 0]], dtype=np.float64)
    print('\nVertices:')
    print(v_coords)
    v_inds = np.array([[0],
                       [1],
                       [2]], dtype=int)

    print('\nPoint to match: s =')
    s = np.array([2, 2, 2])
    print(s)

    print('Expected point: c =')
    c = np.array([2, 2, 0])
    print(c)

    print('Calculated point: c_calc =')
    c_calc = icpm.findClosestPointLinear(s, v_coords, v_inds)
    print(c_calc)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c - c_calc) <= tolerance)
    print(np.all(np.abs(c - c_calc) <= tolerance))


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
