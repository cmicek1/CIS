import PointCloud as pc
import Frame as fr
import PA2_Prob1 as p1
import numpy as np

def distCal(calbody_file, calreadings_file):

    tracker_frames = pc.fromfile(calreadings_file)

    c = []
    for k in range(len(tracker_frames)):
        c.append(tracker_frames[k][2])

    c_exp = p1.c_expected(calbody_file, calreadings_file)

    nFrames = len(c)

    q_min = np.zeros([nFrames, 3])
    q_max = np.zeros([nFrames, 3])
    q_star_min = np.zeros([nFrames, 3])
    q_star_max = np.zeros([nFrames, 3])

    for k in range(nFrames):
        for i in range(0, 3):
            q_min[k][i] = min(c[k].data[i])
            q_max[k][i] = max(c[k].data[i])
            q_star_min[k][i] = min(c_exp[k].data[i])
            q_star_max[k][i] = max(c_exp[k].data[i])
