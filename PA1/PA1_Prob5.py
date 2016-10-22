import PointCloud as pc
import pivot_cal as piv

def p_dimple(empivot_file):
    """
    Computes position of dimpled pin in tracker coordinate system.

    :param calbody_file: File name/path for the calibration object data file
    :param calreadings_file: File name/path for the readings from the trackers

    :type calbody_file: str
    :type calreadings_file: str

    :return: C_expected, a list of the expected positions of EM tracking markers on the calibration object for each
             frame of data
    :rtype: [PointCloud.PointCloud]
    """
    g_clouds = pc.fromfile(empivot_file)
    print g_clouds

    p_cal, p_piv = piv.pivot(g_clouds)

    return p_piv
