import PointCloud as pc


def find_freg(ctfiducials, Cs):
    """
    Returns the transformation from tracker coordinates to CT coordinates.

    :param ctfiducials: The file name/path of the file with positions of each fiducial pin in the CT frame
    :param Cs: A PointCloud of computed positions for the pointer tip relative to the EM tracker frame for each frame of
               data

    :type ctfiducials: str
    :type Cs: PointCloud.PointCloud

    :return: The frame transformation from tracker to CT coordinates
    :rtype: Frame.Frame
    """

    b = pc.fromfile(ctfiducials)
    f_reg = Cs.register(b[0][0])

    return f_reg
