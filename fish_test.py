import math
import numpy as np
import matplotlib.pyplot as plt
import big_junction_init as bji
import junction_init as ji
import Global_Par as Gp
from matplotlib.lines import Line2D

def fish_placement(xmin, ymin, xmax, ymax):
    posi_list = []
    h = ymax - ymin
    w = xmax - xmin
    # h = (int((ymax - ymin) / (Gp.con_dis * 2)) + 1) * Gp.con_dis * 2
    # w = (int((xmax - xmin) / (Gp.con_dis * 2)) + 1) * Gp.con_dis * 2
    ## 第一层控制器相切布置 ( `д´)/
    for i in range(int((w) / (Gp.con_dis * 2)) + 1):
        for j in range(int((h) / (Gp.con_dis * 2)) + 1):
            posi_list.append([xmax - (Gp.con_dis * 2 * i), ymax - (Gp.con_dis * 2 * j)])
    ## 第二层控制器填空相切布置 \(`д´ )
    for i in range(int((w+Gp.con_dis) / (Gp.con_dis*2))):
        for j in range(int((h+Gp.con_dis) / (Gp.con_dis*2))):
            posi_list.append(
                [xmax - Gp.con_dis - Gp.con_dis * 2 * i, ymax - Gp.con_dis - Gp.con_dis *2 * j])

    return posi_list
#
a = fish_placement(0,0,2300,2300)
a = np.array(a)
fig = plt.figure(figsize=(8, 8), dpi=80)
plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=6000, edgecolors='blue')
plt.scatter(a[:, 0], a[:, 1], alpha=1, s=20, edgecolors='black')
plt.show()

