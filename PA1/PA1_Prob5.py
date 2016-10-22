import PointCloud as pc
import pivot_cal as piv

def p_dimple(empivot_file):
    """
    Computes position of dimpled pin in tracker coordinate system.

    :param empivot_file filename/path for file with positions of probe in EM tracker coordinates

    :type calbody_file: str

    :return: p_piv, the position of the dimpled pin in tracker coordinates

    :rtype: np.array
    """
    g_clouds = pc.fromfile(empivot_file)

    p_cal, p_piv = piv.pivot(g_clouds)

    return p_piv
