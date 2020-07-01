import numpy as np
import re
import matplotlib.pyplot as plt
import Init
import Global_Par as gp
n = 0
tag = [0 for i in range(1000)]
posi = [0 for i in range(1000)]
with open("sbg2km40nd.mobility.tcl", 'r') as f:
    movement_list = []
    init_position_list = []
    item_list = []
    key = 0
    for line in f:
        line_list = re.split('[\s]', line)
        a = (int(line_list[3][8:-1]))
        if tag[a] == 0:
            posi[a] = [round(float(line_list[5]),2), round(float(line_list[6]), 2)]
            tag[a] = 1
            n += 1
            # a = " $node_(" +int(a)+ ") set X_ " + round(float(line_list[5]),2) +"\n"
            # b = " $node_(" + int(a) + ") set Y_ " + round(float(line_list[6]), 2) + "\n"
            # c = " $node_(" + int(a) + ") set Z_ " + 0 + "\n"

with open('800.tcl', 'a+') as f:
    for i in range(n):
        a = " $node_(" + str(i) + ") set X_ " + str(posi[i][0]) +"\n"
        b = " $node_(" + str(i) + ") set Y_ " + str(posi[i][1]) + "\n"
        c = " $node_(" + str(i) + ") set Z_ " + "0" + "\n"
        f.write(a)
        f.write(b)
        f.write(c)


    movement_matrix = np.mat(movement_list)
    init_position_matrix = np.mat(init_position_list)

print(1)