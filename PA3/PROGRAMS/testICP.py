import numpy as np
import PointCloud as pc
import ICPmatching as icpm
import BoundingSphere as bs


def testFindTipB(tolerance=1e-4):
    """
    Tests finding the tip position of body B from the tip position of the A body. If the generated and calculated
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
    Tests projection of a point onto a line segment.

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

    print('\nProjection tests passed!')


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

    print('\Case 1: Point in region over triangle, out of plane')
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

    print('\nCase 2: Point outside region over triangle, out of plane')
    print('\nPoint to match: s =')
    s = np.array([5, 2, 6])
    print(s)

    print('Expected point: c =')
    c = np.array([4, 2, 0])
    print(c)

    print('Calculated point: c_calc =')
    c_calc = icpm.findClosestPointLinear(s, v_coords, v_inds)
    print(c_calc)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c - c_calc) <= tolerance)
    print(np.all(np.abs(c - c_calc) <= tolerance))

    print('\nCase 3: Point inside triangle, in plane')
    print('\nPoint to match: s =')
    s = np.array([2.5, 2, 0])
    print(s)

    print('Expected point: c =')
    c = np.array([2.5, 2, 0])
    print(c)

    print('Calculated point: c_calc =')
    c_calc = icpm.findClosestPointLinear(s, v_coords, v_inds)
    print(c_calc)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c - c_calc) <= tolerance)
    print(np.all(np.abs(c - c_calc) <= tolerance))

    print('\nCase 4: Point outside triangle, in plane')
    print('\nPoint to match: s =')
    s = np.array([-2, 1.5, 0])
    print(s)

    print('Expected point: c =')
    c = np.array([0, 1.5, 0])
    print(c)

    print('Calculated point: c_calc =')
    c_calc = icpm.findClosestPointLinear(s, v_coords, v_inds)
    print(c_calc)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c - c_calc) <= tolerance)
    print(np.all(np.abs(c - c_calc) <= tolerance))


def testICPMatchLinear(tolerance=1e-4):
    """
    Tests ICP using a series of points queried against a generated mesh. Uses a brute-force linear search.

    :param tolerance: Maximum tolerance between expected and calculated results

    :type tolerance: float

    :return: None
    """
    print('\nComplicated case: Closest points to a mesh of triangles, using slow linear search')
    v_coords = np.array([[0, 2, 0],
                    [1, 2, 3],
                    [0, 0, 0]], dtype=np.float64)
    to_add = np.array([[4, 4, 4],
                       [0, 0, 0],
                       [0, 0, 0]], dtype=np.float64)
    for i in range(2):
        v_coords = np.hstack((v_coords, v_coords[:, (-3, -2, -1)] + to_add))
    print('\nVertices:')
    print(v_coords)
    tri_inds = np.array([[0, 0, 1, 1],
                         [1, 1, 2, 3],
                         [2, 3, 5, 5]], dtype=int)
    tri_inds = np.hstack((tri_inds, tri_inds + 3 * np.ones((3, 4), dtype=int), np.array([[6], [7], [8]],dtype=int)))

    print('\nTriangle Indices:')
    print(tri_inds)

    s = v_coords.copy()
    s[-1, :] += 4

    print('\nPoints to search for:')
    print(s)

    print('\nExpected points:')
    print(v_coords)

    print('\nCalculated points:')
    c_calc = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds, True)
    print(c_calc.data)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(v_coords - c_calc.data) <= tolerance)
    print(np.all(np.abs(v_coords - c_calc.data) <= tolerance))

    print('\nTest again, with new points')
    print('\nPoints to search for:')
    s = np.zeros((3, 11))
    s[0, :] = np.arange(0, 11)
    s[1, :] = 2
    print(s)

    print('\nExpected points:')
    print(s)

    print('\nCalculated points:')
    c_calc = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds, True)
    print(c_calc.data)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(s - c_calc.data) <= tolerance)
    print(np.all(np.abs(s - c_calc.data) <= tolerance))

    print('\nLinear ICP tests passed!')


def testMakeSphere(tolerance=1e-6):
    """
    Tests the proper creation of bounding spheres using known triangles.

    :param tolerance: Maximum tolerance between expected and calculated results

    :type tolerance: float

    :return: None
    """
    print('\nTest creation of Bounding Spheres')
    print('\nCreate equilateral triangle with vertices =')
    verts = np.array([[0, 1, 2],
                      [0, np.sqrt(3), 0],
                      [0, 0, 0]])
    print(verts)
    inds = np.array([[0],
                     [1],
                     [2]])

    print('\nMake BoundingSphere from triangle.')
    sp = bs.createBS(verts, inds)

    print('Test if specs are correct')
    print('Expected center:')
    c_exp = np.array([1, np.sqrt(3) / 3, 0])
    print(c_exp)
    print('Calculated center:')
    print(sp[0].c)
    print('Expected radius:')
    r_exp = np.sqrt(3) * 2 / 3
    print(r_exp)
    print('Calculated radius:')
    print(sp[0].r)

    print('\nCenter within tolerance?')
    assert np.all(np.abs(c_exp - sp[0].c) <= tolerance)
    print(np.all(np.abs(c_exp - sp[0].c) <= tolerance))

    print('\nRadius within tolerance?')
    assert np.all(np.abs(r_exp - sp[0].r) <= tolerance)
    print(np.all(np.abs(r_exp - sp[0].r) <= tolerance))

    print('\nCreate a right triangle with vertices =')
    verts = np.array([[0, 2, 0],
                      [0, 0, 2],
                      [0, 0, 0]], dtype=np.float64)
    print(verts)
    inds = np.array([[0],
                     [1],
                     [2]])

    print('\nMake BoundingSphere from triangle.')
    sp = bs.createBS(verts, inds)

    print('Test if specs are correct')
    print('Expected center:')
    c_exp = np.array([1, 1, 0])
    print(c_exp)
    print('Calculated center:')
    print(sp[0].c)
    print('Expected radius:')
    r_exp = np.sqrt(2)
    print(r_exp)
    print('Calculated radius:')
    print(sp[0].r)

    print('\nCenter within tolerance?')
    assert np.all(np.abs(c_exp - sp[0].c) <= tolerance)
    print(np.all(np.abs(c_exp - sp[0].c) <= tolerance))

    print('\nRadius within tolerance?')
    assert np.all(np.abs(r_exp - sp[0].r) <= tolerance)
    print(np.all(np.abs(r_exp - sp[0].r) <= tolerance))


def testICPSpherical(tolerance=1e-4):
    """
    Tests ICP using a series of points queried against a generated mesh. Compares method using bounding spheres
    against a brute-force linear search.

    :param tolerance: Maximum tolerance between expected and calculated results

    :type tolerance: float

    :return: None
    """
    print('\nLinear case works; compare with bounding spheres:')
    v_coords = np.array([[0, 2, 0],
                         [1, 2, 3],
                         [0, 0, 0]], dtype=np.float64)
    to_add = np.array([[4, 4, 4],
                       [0, 0, 0],
                       [0, 0, 0]], dtype=np.float64)
    for i in range(2):
        v_coords = np.hstack((v_coords, v_coords[:, (-3, -2, -1)] + to_add))
    print('\nVertices:')
    print(v_coords)
    tri_inds = np.array([[0, 0, 1, 1],
                         [1, 1, 2, 3],
                         [2, 3, 5, 5]], dtype=int)
    tri_inds = np.hstack((tri_inds, tri_inds + 3 * np.ones((3, 4), dtype=int), np.array([[6], [7], [8]], dtype=int)))

    print('\nTriangle Indices:')
    print(tri_inds)

    s = v_coords.copy()
    s[-1, :] += 4

    print('\nPoints to search for:')
    print(s)

    print('\nExpected points:')
    print(v_coords)

    print('\nCalculated points (linear):')
    c_calc1 = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds, True)
    print(c_calc1.data)

    print('\nCalculated points (spheres):')
    c_calc2 = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds)
    print(c_calc2.data)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c_calc1.data - c_calc2.data) <= tolerance)
    print(np.all(np.abs(c_calc1.data - c_calc2.data) <= tolerance))

    print('\nPoints to search for:')
    s = np.zeros((3, 11))
    s[0, :] = np.arange(0, 11)
    s[1, :] = 2
    print(s)

    print('\nExpected points:')
    print(s)

    print('\nCalculated points (linear):')
    c_calc1 = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds, True)
    print(c_calc1.data)

    print('\nCalculated points (spheres):')
    c_calc2 = icpm.ICPmatch(pc.PointCloud(s), v_coords, tri_inds)
    print(c_calc2.data)

    print('\nMatch within tolerance?')
    assert np.all(np.abs(c_calc1.data - c_calc2.data) <= tolerance)
    print(np.all(np.abs(c_calc1.data - c_calc2.data) <= tolerance))

    print('\nBounding Sphere ICP tests passed!')


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
