import numpy as np
import scipy.linalg as scialg
import Frame as fr
import PointCloud as pc


class CovTreeNode:
    """
    Class for node in a covariance tree.
    """
    def __init__(self, triangles, num_tri):
        """
        Initializes a covariance tree data structure for a given surface.
        :param triangles: list of Triangles on the surface
        :param num_tri: number of Triangles on the surface

        :type triangles: np.array([Triangle])
        :type num_tri: integer
        """
        self.triangle_list = triangles
        self.num_tri = num_tri
        self.bounds = None
        self.frame = self._FindCovFrame(self.num_tri)
        self.has_subtrees = False
        self.subtrees = [None, None]
        self._ConstructSubtrees()
        self._FindBoundingBox(self.num_tri)

    def UpdateClosest(self, t, v, bound, closest):
        """
        Finds the closest point to a vector v on a given triangle t, or decides not to search closesly if the point
        is outside the bounding sphere of t.
        :param t: Triangle to find closest point on
        :param v: Point to find closest point to
        :param bound: distance from current closest point
        :param closest: Current closest point in bounding box

        :type t: Triangle
        :type v: np.array(np.float64) 3 X 1
        :type bound: np.float64
        :type closest: np.array(np.float64) 3 X 1

        :return bound: the new distance to the new closest point, or old distance if no new closest point found
        :return closest: the current closest point

        :rtype bound: np.float64
        :rtype closest: np.array(np.float64) 3 X 1
        """
        if np.linalg.norm(v - t.sphere.c) - t.sphere.r > bound[0]:
            return bound, closest
        # Here closest is a one-element list
        cp = t.ClosestPointTo(v)
        dist = np.linalg.norm(cp - v)
        if dist < bound[0]:
            bound[0] = dist
            closest[0] = cp
        return bound, closest


    def FindClosestPoint(self, v, bound, closest):
        """
        Finds the closest point to v on or below this node.
        :param v: the point to find the closest point to
        :param bound: distance between this point and the current closest point
        :param closest: current closest point or estimate

        :type v: np.array(np.float64) 3 X 1
        :type bound: integer
        :type closest: np.array(np.float64) 3 X 1

        :return: None if no closest point found

        """
        temp = (self.frame.inv.r.dot(v.reshape((3, 1))) + self.frame.inv.p).flatten()
        # Note: To pass by reference, closest should be a mutable type (e.g. a list)
        if np.any(temp.reshape((3, 1)) < (self.bounds[0] - bound[0])) or (
                np.any(temp.reshape((3, 1)) > (self.bounds[1] + bound[0]))):
            return

        if self.has_subtrees:
            self.subtrees[0].FindClosestPoint(v, bound, closest)
            self.subtrees[1].FindClosestPoint(v, bound, closest)

        else:
            for i in range(self.num_tri):
                bound, closest = self.UpdateClosest(self.triangle_list[i], v, bound, closest)


    def _FindCovFrame(self, *args):
        """
        Finds the frame of this covariance tree node.
        :param args: Either just the number of triangles, or the number of triangles and a list of triangles.
        :type args: integer and [Triangle]

        :return: The frame of this covariance tree node
        :rtype: fr.Frame
        """
        if len(args) == 1:
            points = None
            num_triangles = args[0]
            for i in range(num_triangles):
                if i == 0:
                    points = self.triangle_list[i].corners.data.tolist()
                else:
                    for p in self.triangle_list[i].corners.data.tolist():
                        points.append(p)

            points = np.array(points).squeeze().T

            return self._FindCovFrame(points, num_triangles * 3)

        elif len(args) == 2:
            points = args[0][:, 0:args[1]]
            c = np.mean(points, axis=1, keepdims=True)
            u = points - c

            A = u.dot(u.T)

            evals, evecs = np.linalg.eig(A)

            inds = evals.argsort()[::-1]

            max_evec = evecs[inds[0]]

            coeffs = np.zeros((3, 9))
            coeffs[(0, 1, 2), (0, 3, 6)] = 1.0

            r = np.linalg.lstsq(coeffs, max_evec)[0].reshape((3, 3))
            u, s, v = np.linalg.svd(r)

            correction = np.eye(3)
            correction[-1, -1] = scialg.det(v.T.dot(u.T))

            r = u.dot(correction.dot(v))

            return fr.Frame(r, c)


    def _FindBoundingBox(self, n):
        """
        Finds the upper and lower bounds of a bounding box around this covariance tree.
        :param n: Number of triangles in the covariance tree.
        :type n: integer

        :return: upper and lower bounds of bounding box around this covariance tree
        :rtype: []
        """
        LB = pc.PointCloud(self.triangle_list[0].SortPoint()).transform(self.frame.inv).data
        bounds = [LB, LB]
        for k in range(n):
            bounds = self.triangle_list[k].EnlargeBounds(self.frame, bounds)
        self.bounds = bounds
        return bounds


    def _SplitSort(self, num):
        """
        Splits and sorts triangles based on their x coordinate.
        :param num: Number of triangles in this covariance tree.
        :type num: integer

        :return: index in list of triangles where split occurs
        :rtype: integer
        """
        points = []
        for k in range(num):
            points.append(pc.PointCloud(self.triangle_list[k].SortPoint()).transform(self.frame.inv).data.tolist())
        points = np.array(points).squeeze().T
        inds = np.argsort(points[0, :])
        points = points[:, inds]
        self.triangle_list = self.triangle_list[inds]

        possible_splits = np.any(np.diff(np.signbit(points[0, :])))
        if possible_splits:
            return np.where(np.diff(np.signbit(points[0, :])))[0][0]


    def _ConstructSubtrees(self):
        """
        Constructs subtrees based on list of triangles in this covariance tree.

        :return: None
        """
        if len(self.triangle_list) <= 1:
            return

        splitpoint = self._SplitSort(self.num_tri)
        if splitpoint is not None:
            if splitpoint == 0:
                return

            elif splitpoint == self.num_tri:
                return

            else:
                self.has_subtrees = True
                self.subtrees[0] = CovTreeNode(self.triangle_list[0:splitpoint], splitpoint)
                self.subtrees[1] = CovTreeNode(self.triangle_list[splitpoint:self.num_tri], self.num_tri - splitpoint)
