import sys, os
import PA2_Prob4 as p4
import distortion as d
import PA2_Prob5 as p5
import PA2_Prob6 as p6
import test


def main():
    """
    Main method, takes command line arguments for input files.
    :return: None
    """

    # Initialize variables
    calbody = None
    calreadings = None
    empivot = None
    ctfiducials = None
    emfiducialss = None
    emnav = None

    # Add 'test' command line option
    if str(sys.argv[1]) == 'test':
        if len(sys.argv) == 3:
            tolerance = float(sys.argv[2])
            test.test_reg(tolerance)
        else:
            test.test_reg()
        sys.exit(0)

    # Parse arguments for regular execution
    for arg in sys.argv:
        if arg.split('.')[0].split('-')[-1] == 'calbody':
            calbody = arg
        if arg.split('.')[0].split('-')[-1] == 'calreadings':
            calreadings = arg
        if arg.split('.')[0].split('-')[-1] == 'empivot':
            empivot = arg
        if arg.split('.')[0].split('-')[-1] == 'fiducials':
            ctfiducials = arg
        if arg.split('.')[0].split('-')[-1] == 'fiducialss':
            emfiducialss = arg
        if arg.split('.')[0].split('-')[-1] == 'nav':
            emnav = arg

    outname = sys.argv[1].split('/')[-1].rsplit('-', 1)[0] + '-output2.txt'

    # Run code for probelms 4 - 6 and save output
    tofile(outname, calbody, calreadings, empivot, ctfiducials, emfiducialss, emnav)


def tofile(outfile, calbody, calreadings, empivot, ctfiducials, emfiducialss, emnav):
    """
    Runs methods for questions 4-6 and writes output file with solutions.
    :param outfile: File name/path for output file
    :param calbody: File name/path for the calibration object data file
    :param calreadings: File name/path for the readings from the trackers
    :param empivot: File name/path for EM pivot poses
    :param ctfiducials: The file name/path of the file with positions of each fiducial pin in the CT frame
    :param emfiducialss: The file name/path of the file with marker positions when the pointer is on the fiducials,
                         relative to the EM tracker.
    :param emnav: The file name/path of the file with marker positions when the pointer is in an arbitrary position,
                  relative to the EM tracker.

    :type outfile: str
    :type calbody: str
    :type calreadings: str
    :type empivot: str
    :type ctfiducials: str
    :type emfiducialss: str
    :type emnav: str

    :return: None
    """
    p_ans, C, qmi, qma, qmis, qmas = d.distcal(calbody, calreadings, empivot)

    Cs = p4.tip_in_EM(empivot, emfiducialss, p_ans[0], C, qmi, qma, qmis, qmas)

    F = p5.find_freg(ctfiducials, Cs)

    CT = p6.tip_in_CT(empivot, emnav, p_ans[0], F, C, qmi, qma, qmis, qmas)

    f = open(outfile, 'w')
    h, t = os.path.split(outfile)

    for i in range(CT.data.shape[1]):
        f.write('{0}, {1}\n'.format(CT.data.shape[1], t))
        f.write('  {0},   {1},   {2}\n'.format(format(CT.data[0][i], '.2f'),
                                               format(CT.data[1][i], '.2f'),
                                               format(CT.data[2][i], '.2f')))
    f.close()


if __name__ == '__main__':
    main()
