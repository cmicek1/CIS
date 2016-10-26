import numpy as np
import PointCloud as pc
import distortion as d


def tip_in_CT(emnav, ptip, F_reg, coeffs, q_min, q_max, q_star_min, q_star_max):
    """
    Returns the position of the pointer tip in CT coordinates for tracker data frames.

    :param emfiducialss: The file name/path of the file with marker positions when the pointer is on the fiducials,
                         relative to the EM tracker.
    :param ptip: The coordinates of the tip of the pointer relative to the pointer coordinate system (Output of
                 pivot_cal.pivot)
    :param coeffs: Coefficient matrix for dewarping (Output of distortion.distcal)
    :param q_min: Vector of input minima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_max: Vector of input maxima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_star_min: Vector of output minima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_star_max: Vector of output maxima for the initial correction matrix creation (Output of distortion.distcal)

    :type emfiducialss: str
    :type ptip: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type coeffs: numpy.array([numpy.float64][]) degree**3 x 3
    :type q_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_max: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_max: numpy.array(numpy.float64) shape (3,) or (, 3)

    :return: A list of arrays, where each array is the position of the pointer tip in EM tracker coordinates for each
             frame
    """

    G = d.correct(emnav, coeffs, q_min, q_max, q_star_min, q_star_max)
    G_0 = np.mean(G[0][0].data, axis=1, keepdims=True)

    G_j = G[0][0].data - G_0

    CTs = pc.PointCloud()

    for frame in G:
        F = frame[0].register(pc.PointCloud(G_j))
        C = pc.PointCloud(ptip.reshape((3, 1))).transform(F).transform(F_reg)
        if CTs.data is None:
            CTs.data = C.data
        else:
            CTs = CTs.add(C)

    return CTs
