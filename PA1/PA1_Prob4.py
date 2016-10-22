import PointCloud as pc


def c_expected(calbody_file, calreadings_file):
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

    c_exp = []
    for frame in tracker_frames:
        f_d = object_frame[0][0].register(frame[0])
        f_a = object_frame[0][1].register(frame[1])

        c_exp.append(object_frame[0][2].transform(f_d.inv.compose(f_a)))

    for c in range(len(c_exp)):
        print c_exp[c].data

    return c_exp
