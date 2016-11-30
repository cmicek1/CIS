import numpy as np
import BoundingSphere as bs
import ICPmatching as icpm


class Triangle:
    """
    Class for a Triangle thing with a bounding sphere.
    """
    def __init__(self, corners):
        """
        Initializes the triangle with a point cloud of corners.
        :param corners: Coordinates of the three corners of the triangle.
        :type corners: pc.Pointcloud
        """
        self.corners = corners
        data = self.corners.data
        p, q, r = data[:, 0], data[:, 1], data[:, 2]
        q, r = bs.calcCenterandRadius(p, q, r)
        self.sphere = bs.BoundingSphere(q, r)


    def SortPoint(self):
        """
        Calculates the "sort point" for a triangle aka the mean of the corners.

        :return: the sort point
        :rtype: np.array(np.float64) 3 X 1
        """
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

        return bounds

    def BoundingBox(self, frame):
        return self.EnlargeBounds(frame, [np.array([[np.inf], [np.inf], [np.inf]]),
                                          np.array([[-np.inf], [-np.inf], [-np.inf]])])

    def MayBeInBounds(self, frame, bounds):
        FiC = self.corners.transform(frame.inv)
        for i in range(3):
            if np.all(FiC.data[:, i] >= bounds[0]) and np.all(FiC.data[:, i] <= bounds[1]):
                return True
        return False
