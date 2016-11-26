import numpy as np


class Triangle:
    def __init__(self, corners):
        self.corners = corners

    def SortPoint(self):
        return np.mean(self.corners.data, axis=1, keepdims=True)

    def EnlargeBounds(self, frame, bounds):
        FiC = frame.inv.transform(self.corners)
        for i in range(3):
            bounds[0] = np.amin(np.hstack((bounds[0], FiC.data[:, i])), axis=1, keepdims=True)
            bounds[1] = np.amax(np.hstack((bounds[1], FiC.data[:, i])), axis=1, keepdims=True)

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