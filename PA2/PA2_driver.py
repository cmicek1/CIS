import PA2_Prob4 as p4
import distortion as d
import PA2_Prob5 as p5
import PA2_Prob6 as p6
import Frame as fr

#TODO: @chris this is just to automate running for the console but this isn't actually the driver

def runprogram(calbody, calreadings, empivot, emfiducials, ctfiducials, emnav):

    pans, C, qmi, qma, qmis, qmas = d.distcal(calbody, calreadings, empivot)

    Cs = p4.tip_in_EM(emfiducials, pans[1], C, qmi, qma, qmis, qmas)

    F = p5.find_freg(ctfiducials, Cs)

    CT = p6.tip_in_CT(emnav, pans[1], F, C, qmi, qma, qmis, qmas)

    return CT.data
