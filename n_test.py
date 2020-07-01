import Packet as Pkt
import Global_Par as Gp
import dij_test1 as dij
import networkx as nx
import junction_init as ji
import big_junction_init as bji
import math as m
import bf_test as bf
import jhmmtg as jh
import linecircle as lc
import big_jhmmtg as bjh
import tgeaa as tg
import add_test as ad
import HRLB as hr
# import big_HRLB as bhr
import time as tim
import random
# import mcds
import math
import math
import numpy as np
import matplotlib.pyplot as plt
import fish_test
import rectangle
import re
import Get_Move as Gm
import cover_test as cover
def f_cal(v_load,vd_load):
    f = [0 for i in range(len(v_load))]
    for i in range(len(vd_load)):
        f[i] = 0.7 * (1-((20 - v_load[i]))/20) + 0.3 *(10 - vd_load[i])
    return f

def uua(i,n,s,on_controller,v_c,v_c_d,vehicle_position,v_load,vd_load):
    if len(n) == 0:
        return 0,[]
    else:
        ncm = [0 for i in range(len(s))]
        change = 1
        atleast = 1
        while change == 1 and atleast == 1:
            change = 0
            atleast = 0
            for ve in range(len(s)):
                if ncm[ve] == 0:
                    fve = [0 for i in range(len(n))]
                    for nei in range(len(n)):
                        dis = math.sqrt(pow(on_controller[n[nei]][0] - vehicle_position[s[ve]][0], 2) + pow(
                            on_controller[n[nei]][1] - vehicle_position[s[ve]][1], 2))
                        fve[nei] = 0.7 * (10 - v_load[n[nei]]+1) + 0.3 *(1000 - dis)/100
                    aa = fve.index(max(fve))
                    dis = math.sqrt(pow(on_controller[n[aa]][0] - vehicle_position[s[ve]][0], 2) + pow(
                            on_controller[n[aa]][1] - vehicle_position[s[ve]][1], 2))
                    if dis <= 1250:
                        ncm[ve] = n[aa]
                        v_load[n[aa]] += 1
                        change = 1
            if ncm.count(0) > 0:
                atleast = 1
        if atleast == 0:
            return 1,ncm
        else:
            return 0,[]

def uua_1(i,n,s,vehicle_position):
    ncm = [-1 for i in range(len(s))]
    change = 1
    atleast = 1
    while change == 1 and atleast == 1:
        change = 0
        atleast = 0
        for ve in range(len(s)):
            if ncm[ve] == -1:
                fve = [0 for i in range(len(n))]
                for nei in range(len(n)):
                    dis = math.sqrt(pow(n[nei][0] - s[ve][0], 2) + pow(
                        n[nei][1] - s[ve][1], 2))
                    fve[nei] = 0.7 * (10 - 1) + 0.3 *(1000 - dis)/100
                aa = fve.index(max(fve))
                dis = math.sqrt(pow(n[nei][0] - s[ve][0], 2) + pow(
                        n[nei][1] - s[ve][1], 2))
                if dis <= 1250:
                    ncm[ve] = aa
                    change = 1
        if ncm.count(-1) > 0:
            atleast = 1
    if atleast == 0:
        return 1,ncm
    else:
        return 0,[]


def oua(x,offn,s,on_controller,controller_position,vehicle_position,v_load,vd_load):
    if len(offn) == 0:
        return []
    else:
        position = []
        for i in s:
            position.append(vehicle_position[i])
        vsg = [0 for i in range(10)]
        osg = [0 for i in range(10)]
        for ge in range(10):
            v1 = []
            v2 = []
            for j in range(int(len(position)/2)):
                v1.append(position[random.randint(0,len(position))-1])
            for i in position:
                if v1.count(i) == 0:
                    v2.append(i)
            a1,al1 = uua_1(i,offn,v1,vehicle_position)
            if a1 == 1:
                v1_load = [0 for i in range(len(v1))]
                vd1_load = [0 for i in range(len(v1))]
                g1 = len(v2)/len(position)*((0.7 * (10 - v_load[x]) + 0.3 *(1000 - vd_load[x])/100))

                for i in range(len(al1)):
                    if al1[i] < len(v1_load):
                        v1_load[al1[i]] += 1

                for i in range(len(v1)):
                    dis = math.sqrt(pow(v1[i][0] - offn[al1[i]][0], 2) + pow(
                        v1[i][1] - offn[al1[i]][1], 2))
                    if al1[i] < len(vd1_load):
                        vd1_load[al1[i]] += dis

                for i in range(len(vd1_load)):
                    if v1_load[i] == 0:
                        vd1_load[i] = 0
                    else:
                        vd1_load[i] = vd1_load[i] / v1_load[i] / 100
                f1 = f_cal(v1_load,vd1_load)
                for i in range(len(f1)):
                    g1 += v1_load[i] / len(position) * f1[i]
            else:
                g1 = 0


            a2,al2 = uua_1(i,offn,v2,vehicle_position)
            if a2 == 1:
                v2_load = [0 for i in range(len(v2))]
                vd2_load = [0 for i in range(len(v2))]
                g2 = len(v1) / len(position) * (0.7 * (10 - v_load[x]) + (0.3 * (1000 - vd_load[x])/100))
                for i in range(len(al2)):
                    if al2[i] < len(v2_load):
                        v2_load[al2[i]] += 1

                for i in range(len(v2)):
                    dis = math.sqrt(pow(v2[i][0] - offn[al2[i]][0], 2) + pow(
                        v2[i][1] - offn[al2[i]][1], 2))
                    if al2[i] < len(vd2_load):
                        vd2_load[al2[i]] += dis

                for i in range(len(vd2_load)):
                    if v2_load[i] == 0:
                        vd2_load[i] = 0
                    else:
                        vd2_load[i] = vd2_load[i] / v2_load[i] / 100

                f2 = f_cal(v2_load, vd2_load)

                for i in range(len(f2)):
                    g2 += v2_load[i] / len(position) * f2[i]
            else:
                g2 = 0

            if g1 > g2:
                vsg[ge] = g1
                osg[ge] = al1
            else:
                vsg[ge] = g2
                osg[ge] = al2
        aa = vsg.index(max(vsg))
        return osg[aa]




def nzsg(on_s,vehicle_position, controller_position,fix_controller_position):
    off = []
    on = []
    beta = 2.5
    xita = 1
    on_controller = []
    for i in fix_controller_position:
        on_controller.append(i)
    for i in range(len(on_s)-2):
        if on_s[i] == 1:
            on_controller.append(controller_position[i])

    X = np.array(on_controller)
    X = X.transpose()
    m, n = X.shape
    G = np.dot(X.T, X)
    # 把G对角线元素拎出来，列不变，行复制n遍。
    H = np.tile(np.diag(G), (n, 1))
    D = H + H.T - G * 2
    D = np.sqrt(D)
    ## 构建控制器之间的传输图
    con_g = nx.DiGraph()
    for i in range(len(on_controller)):
        for j in range(len(on_controller)):
            if D[i][j] > 1000:
                D[i][j] = 0


    xa = len(vehicle_position)
    xb = len(on_controller)
    # 联系每一个解和对应的向量，即n辆车到m个控制器的位置，a为车辆位置矩阵，b为控制器位置矩阵
    aq = np.array(vehicle_position)
    bq = np.array(on_controller)
    ## 计算asq
    a = aq ** 2
    a = a.sum(axis=1)
    aaa = a
    for i in range(xb - 1):
        aaa = np.vstack([aaa, a])
    aaa = aaa.transpose()
    ## 计算bsq
    b = bq ** 2
    b = b.sum(axis=1)
    b = b.transpose()
    bbb = b
    for i in range(xa - 1):
        bbb = np.vstack([bbb, b])
    ## 计算 a*(b^-1)
    c = aq.dot(bq.transpose())
    ## asq+bsq-2*a*(b^-1)开方即为n到m的距离矩阵
    ddd = np.sqrt(aaa + bbb - 2 * c)
    ## 在每行中挑出最小的，即为车辆的最距离控制器距离，索引通过argmin获得
    v_c_d = np.min(ddd.T, 0)
    v_c = np.argmin(ddd.T, 0)

    v_load = [0 for i in range(len(on_controller))]
    for i in v_c:
        v_load[i] += 1

    vd_load = [0 for i in range(len(v_load))]
    for i in range(len(v_c_d)):
        vd_load[v_c[i]] += v_c_d[i]
    for i in range(len(vd_load)):
        if v_load[i] == 0:
            vd_load[i] = 0
        else:
            vd_load[i] = vd_load[i] / v_load[i]/100

    f = f_cal(v_load,vd_load)

    for i in range(len(f)):
        n = []
        for j in range(len(D[i])):
            if D[i][j] != 0:
                n.append(j)
        s = []
        for j in range(len(v_c)):
            if v_c[j] == i:
                s.append(j)
        if f[i] >= beta:
            a,al = uua(i,n,s,on_controller,v_c,v_c_d,vehicle_position,v_load,vd_load)
            if a == 1:
                off.append(i)
        if f[i] <= xita:
            offn = []
            for j in range(len(controller_position)):
                if on_s[j] == 0:
                    dis = math.sqrt(pow(on_controller[i][0] - controller_position[j][0], 2) + pow(
                        on_controller[i][1] - controller_position[j][1], 2))
                    if dis < 1000:
                        offn.append(controller_position[j])
            aa = oua(i,offn,s,on_controller,controller_position,vehicle_position,v_load,vd_load)
            for i in aa:
                on.append(i)

    for i in off:
        for j in range(len(controller_position)):
            if on_controller[i] == controller_position[j]:
                on_s[j] = 0
                break

    for i in on:
        for j in range(len(controller_position)):
            if i < len(offn):
                if offn[i] == controller_position[j]:
                    on_s[j] = 1
                    break
    return on_s