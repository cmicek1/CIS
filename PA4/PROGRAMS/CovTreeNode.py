import numpy as np
import scipy.linalg as scialg
import Frame as fr
import PointCloud as pc


class CovTreeNode:
    def __init__(self, triangles, num_tri):
        # Data type should be numpy object array
        self.triangle_list = triangles
        self.num_tri = num_tri
        self.bounds = None
        self.frame = self._FindCovFrame(self.num_tri)
        self.has_subtrees = False
        self.subtrees = [None, None]
        self._ConstructSubtrees()
        self._FindBoundingBox(self.num_tri)

    def UpdateClosest(self, t, v, bound, closest):
        cp = t.ClosestPointTo(v)
        dist = np.linalg.norm(cp - v)
        if dist < bound:
            bound = dist
            closest = cp
        return bound, closest

    def FindClosestPoint(self, v, bound, closest):
        if np.any(v < (self.bounds[0] - bound)) or np.any(v > (self.bounds[1] + bound)):
            return closest

        if self.has_subtrees:
            self.subtrees[0].FindClosestPoint(v, bound, closest)
            self.subtrees[1].FindClosestPoint(v, bound, closest)

        else:
            for i in range(self.num_tri):
                bound, closest = self.UpdateClosest(self.triangle_list[i], v, bound, closest)
        return closest

    def _FindCovFrame(self, *args):
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
        LB = pc.PointCloud(self.triangle_list[0].SortPoint()).transform(self.frame.inv).data
        bounds = [LB, LB]
        for k in range(n):
            bounds = self.triangle_list[k].EnlargeBounds(self.frame, bounds)
        self.bounds = bounds
        return bounds

    def _SplitSort(self, num):
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
