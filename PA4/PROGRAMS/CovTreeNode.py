import numpy as np
import Frame as fr
import PointCloud as pc


class CovTreeNode:
    def __init__(self, triangles, num_tri):
        self.triangle_list = np.array(triangles)
        self.num_tri = num_tri
        self.frame = self._FindCovFrame()
        self.has_subtrees = False
        self.subtrees = [None, None]
        self._ConstructSubtrees()

    def _FindCovFrame(self, *args):
        if len(args) == 1:
            points = []
            num_points = args[0]
            for i in range(num_points):
                points.append(self.triangle_list[i].data.tolist())

            points = np.array(points).squeeze().T

            return self._FindCovFrame(points, num_points)

        elif len(args) == 2:
            points = args[0][:, 0:args[1] + 1]
            c = np.mean(points, axis=1, keepdims=True)
            u = points - c

            A = u.dot(u.T)

            evals, evecs = np.linalg.eig(A)

            inds = evals.argsort()[::-1]

            max_evec = evecs[inds[0]]

            coeffs = np.zeros((3, 9))
            coeffs[[0, 0], [1, 3], [2, 6]] = 1.0

            r = np.linalg.lstsq(coeffs, max_evec)

            return fr.Frame(r, c)

    def _FindBoundingBox(self, n):
        LB = pc.PointCloud(self.triangle_list[0].SortPoint()).transform(self.frame.inv)
        bounds = [LB, LB]
        for k in range(n):
            bounds = self.triangle_list[k].EnlargeBounds(self.frame, bounds)

        return bounds

    def _SplitSort(self, num):
        points = []
        for k in range(num):
            points.append(pc.PointCloud(self.triangle_list[k].SortPoint()).transform(self.frame.inv).data.tolist())
        points = np.array(points).squeeze().T
        inds = np.argsort(points[0, :])
        points = points[inds]
        self.triangle_list = self.triangle_list[inds]

        return np.where(np.diff(np.signbit(points[0, :])))[0]

    def _ConstructSubtrees(self):
        if len(self.triangle_list) <= 1:
            return

        self.has_subtrees = True
        splitpoint = self._SplitSort(self.num_tri)
        self.subtrees[0] = CovTreeNode(self.triangle_list[0], splitpoint)
        self.subtrees[1] = CovTreeNode(self.triangle_list[splitpoint], self.num_tri - splitpoint)
