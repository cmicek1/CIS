import pandas as pd
import PA1_Prob4 as p4
import PA1_Prob5 as p5
import PA1_Prob6 as p6

def tofile(outfile, calbody, calreadings, empivot, optpivot):

    c_exp = p4.c_expected(calbody, calreadings)
    p_em = p5.p_dimple(empivot)
    p_opt = p6.p_dimple(optpivot)

    f = open(outfile, 'w')
    f.write(str(outfile))

    f.close()
