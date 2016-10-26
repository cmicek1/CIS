import numpy as np
import PointCloud as pc
import distortion as d


def find_freg(ctfiducials, Cs, coeffs, q_min, q_max, q_star_min, q_star_max):
    """
    Returns the transformation from tracker coordinates to CT coordinates.

    :param ctfiducials: The file name/path of the file with marker positions when the pointer is on the fiducials,
                        relative to the CT frame.
    :param ptip: The coordinates of the tip of the pointer relative to the pointer coordinate system (Output of
                 pivot_cal.pivot)
    :param coeffs: Coefficient matrix for dewarping (Output of distortion.distcal)
    :param q_min: Vector of input minima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_max: Vector of input maxima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_star_min: Vector of output minima for the initial correction matrix creation (Output of distortion.distcal)
    :param q_star_max: Vector of output maxima for the initial correction matrix creation (Output of distortion.distcal)

    :type ctfiducials: str
    :type ptip: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type coeffs: numpy.array([numpy.float64][]) degree**3 x 3
    :type q_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_max: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_min: numpy.array(numpy.float64) shape (3,) or (, 3)
    :type q_star_max: numpy.array(numpy.float64) shape (3,) or (, 3)

    :return: A list of arrays, where each array is the position of the pointer tip in EM tracker coordinates for each
             frame
    """

    b = d.correct(ctfiducials, coeffs, q_min, q_max, q_star_min, q_star_max)
    f_reg = Cs.register(b[0][0])

    return f_reg
