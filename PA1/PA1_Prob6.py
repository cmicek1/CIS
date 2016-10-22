import PointCloud as pc
import pivot_cal as piv
import frame_transformations as ft

def p_dimple(calbody_file, calreadings_file):
    """
    Computes expected EM marker positions on calibration object, given measured positions of markers on the object
    and data from each tracker.

    :param calbody_file: File name/path for the calibration object data file
    :param calreadings_file: File name/path for the readings from the trackers

    :type calbody_file: str
    :type calreadings_file: str

    :return: C_expected, a list of the expected positions of EM tracking markers on the calibration object for each
             frame of data
    :rtype: [PointCloud.PointCloud]
    """
    tracker_frames = pc.fromfile(calreadings_file)

    object_frame = pc.fromfile(calbody_file)

    f_d = object_frame[0][0].register(tracker_frames[0][0])
    print f_d.r
    print f_d.p

    for i in range(len(tracker_frames)):
        tracker_frames[i][0] = tracker_frames[i][0].transform(f_d)

    p_cal, p_piv = piv.pivot(tracker_frames)

    return p_piv
