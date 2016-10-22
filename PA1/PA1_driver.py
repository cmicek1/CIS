import PA1_Prob4 as p4
import PA1_Prob5 as p5
import PA1_Prob6 as p6
import os

def tofile(outfile, calbody, calreadings, empivot, optpivot):

    c_exp = p4.c_expected(calbody, calreadings)
    p_em = p5.p_dimple(empivot)
   # p_opt = p6.p_dimple(optpivot)

    f = open(outfile, 'w')
    h, t = os.path.split(outfile)
    f.write('{0}, {1}, {2}\n'.format(len(c_exp[0].data[0]), len(c_exp), t))
    f.write('{0},   {1},   {2}\n'.format(format(p_em[0], '.2f'), format(p_em[1], '.2f'), format(p_em[2], '.2f')))
    f.write('optical output goes here \n')
    #f.write('{0},   {1},   {2}'.format(p_opt[0], p_opt[1], p_opt[2]))
    #insert loop for writing c_exp in correct format

    for i in range(len(c_exp)):
        for k in range(len(c_exp[0].data[1])):
            f.write('{0},   {1},   {2}\n'.format(format(c_exp[i].data[0][k], '.2f'), format(c_exp[i].data[1][k], '.2f'),
                                                                                            format(c_exp[i].data[2][k], '.2f')))
    f.close()
