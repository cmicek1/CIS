import PointCloud as pc
import pivot_cal as piv
import frame_transformations as ft

def p_dimple(optpivot_file):
    """
    Computes expected EM marker positions on calibration object, given measured positions of markers on the object
    and data from each tracker.

    :param optpivot_file: File name/path for the calibration object data file

    :type optpivot_file: str

    :return: C_expected, a list of the expected positions of EM tracking markers on the calibration object for each
             frame of data
    :rtype: [PointCloud.PointCloud]
    """
    opt_frames = pc.fromfile(optpivot_file)

    f_d = opt_frames[0][0].register(opt_frames[0][1])
    print f_d.r
    print f_d.p

    for i in range(len(opt_frames)):
        opt_frames[i][1] = opt_frames[i][1].transform(f_d)

    p_cal, p_piv = piv.pivot(opt_frames, 1)

    print p_cal
    print p_piv

    return p_piv
