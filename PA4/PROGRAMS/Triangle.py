import numpy as np
import BoundingSphere as bs
import ICPmatching as icpm


class Triangle:
    def __init__(self, corners):
        self.corners = corners
        data = self.corners.data
        p, q, r = data[:, 0], data[:, 1], data[:, 2]
        q, r = bs.calcCenterandRadius(p, q, r)
        self.sphere = bs.BoundingSphere(q, r)

    def SortPoint(self):
        return np.mean(self.corners.data, axis=1, keepdims=True)

    def ClosestPointTo(self, v):
        data = self.corners.data
        p, q, r = data[:, 0], data[:, 1], data[:, 2]
        return icpm.minPointonTriangle(v, p, q, r)

    def EnlargeBounds(self, frame, bounds):
        FiC = self.corners.transform(frame.inv)

        for i in range(3):
            bounds[0] = np.amin(np.hstack((bounds[0], FiC.data[:, i].reshape((3, 1)))), axis=1, keepdims=True)
            bounds[1] = np.amax(np.hstack((bounds[1], FiC.data[:, i].reshape((3, 1)))), axis=1, keepdims=True)
        # for i in range(3):
        #     bounds[0][0] = min(bounds[0][0], FiC.data[i][0])
        #     bounds[0][1] = min(bounds[0][1], FiC.data[i][1])
        #     bounds[0][2] = min(bounds[0][2], FiC.data[i][2])
        #     bounds[1][0] = max(bounds[1][0], FiC.data[i][0])
        #     bounds[1][1] = max(bounds[1][1], FiC.data[i][1])
        #     bounds[1][2] = max(bounds[1][2], FiC.data[i][2])

        return bounds

    def BoundingBox(self, frame):
        return self.EnlargeBounds(frame, [np.array([[np.inf], [np.inf], [np.inf]]),
                                          np.array([[-np.inf], [-np.inf], [-np.inf]])])

    def MayBeInBounds(self, frame, bounds):
        FiC = frame.inv.transform(self.corners)
        for i in range(3):
            if np.all(FiC.data[:, i] >= bounds[0]) and np.all(FiC.data[:, i] <= bounds[1]):
                return True
        return False
