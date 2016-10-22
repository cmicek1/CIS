import numpy as np
import scipy.linalg as scialg
import pandas as pd
import Frame


class PointCloud:
    """
    Class representing a point cloud. Consists of a numpy array of column vectors representing each point in the
    cloud, and pertinent methods for frame transformations, file IO, etc.
    """
    def __init__(self, data=None):
        """
        Initializes the point cloud, either empty or with the data provided
        :param data: Numpy array of column vectors, with each column representing each point in the point cloud
        :type data: numpy.array([numpy.float64][]), M x N (usually 3 x N)
        """
        self.data = data

    def register(self, b):
        """
        Performs rigid-body registration with respect to another point cloud b, and returns the corresponding frame
        transformation.
        :param b: The point cloud being mapped to
        :type b: PointCloud

        :return: The Frame transformation F = [r, p] from current frame to b
        :rtype: Frame.Frame
        """
        # NOTE: data, b are arrays of column vectors
        a_bar = np.mean(self.data, axis=1, keepdims=True)
        b_bar = np.mean(b.data, axis=1, keepdims=True)

        a_tilde = self.data - a_bar
        b_tilde = b.data - b_bar

        # Method using SVD to directly solve for R

        H = self.data.dot(b.data.T)

        u, s, v_t = scialg.svd(H)

        u = u.T
        v_t = v_t.T

        correction = np.identity(v_t.shape[1])
        correction[-1, -1] = scialg.det(v_t.dot(u))

        r = v_t.dot(correction.dot(u))

        p = b_bar - r.dot(a_bar)

        return Frame.Frame(r, p)

    def transform(self, f):
        """
        Evaluate a frame transformation applied to the current point cloud.
        :param f: The Frame transformation f = [r, p] to transform with
        :type f: Frame.Frame

        :return: The resulting point cloud
        :rtype: PointCloud
        """
        return PointCloud(f.r.dot(self.data) + f.p)


def fromfile(fpath):
    """
    Extract a list of PointClouds from a file.
    :param fpath: The file path to the input data.
    :type fpath: str

    :return: A list of lists of PointClouds. Each internal list represents a frame, with the clouds therein
             representing different sets of known points.

    :rtype: [PointCloud][]
    """

    delims = pd.read_csv(fpath, header=None, nrows=1)
    name = delims.values[0, delims.shape[1] - 1].split('.')[0].split('-')[-1]

    nframes = {'calbody': 1, 'calreadings': delims.values[0, -2], 'empivot': delims.values[0, -2],
               'optpivot': delims.values[0, -2], 'output1': delims.values[0, -2], 'fiducials': 1, 'fiducialss': 1,
               'nav': delims.values[0, -2], 'output2': delims.values[0, -2]}

    dframe = pd.read_csv(fpath, header=None, names=['x', 'y', 'z'], skiprows=1)

    start = True

    frame_clouds = []
    inds = None
    frame_length = None

    for frame in range(nframes[name]):
        if start:
            start = False
            inds = [0]
            if nframes[name] == 1:
                offset = 0
            else:
                offset = 1
            for i in range(delims.shape[1] - 1 - offset):
                inds.append(inds[i] + delims.values[0, i])

            frame_length = inds[-1]

        clouds = []
        for points in range(len(inds) - 1):
            clouds.append(PointCloud(
                dframe.values[inds[points] + frame_length * frame:inds[points + 1] + frame_length * frame, :].T))

        frame_clouds.append(clouds)

    return frame_clouds

