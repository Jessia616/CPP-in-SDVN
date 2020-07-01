import os
import re
import math
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import big_junction_init as bji
import dij_test1 as dij
import Global_Par as Gp

alpha = 0.5
beta = 1 - alpha


class edge:
    def __init__(self, u, v, t, d):
        self.u = u  # hello请求列表
        self.v = v  # 路由请求列表
        self.t = t  # 错误请求列表
        self.d = d  # 邻接矩阵


def insort_right(a, x, lo=0, hi=None):
    """Insert item x in list a, and keep it sorted assuming a is sorted.

    If x is already in a, insert it to the right of the rightmost x.

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """

    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(a)
    while lo < hi:
        mid = (lo+hi)//2
        if x.t < a[mid].t:
            hi = mid
        else:
            lo = mid+1
    a.insert(lo, x)


def junction_judge(x, y, node_id):
    min = 999999999
    intersection_id = 0
    for key_i, i in enumerate(bji.junction_position):
        if (abs(x - i[0]) > min) or (abs(y - i[1]) > min):
            continue
        else:
            d = math.sqrt(pow(x - i[0], 2)+pow(x - i[0], 2))
            if (d<min):
                min = d
                intersection_id = key_i
    bji.junction_vehicle[intersection_id].append(node_id)
    return intersection_id


def cal_weight(node, next_node, des):
    den_in = 0.0
    h_in = 0.0
    dis_in = 0.0
    if bji.adj_martix[node][next_node] != 0:
        den_in = float(len(bji.junction_vehicle[node])+len(bji.junction_vehicle[next_node]))/float(bji.junction_distance[node][next_node])
        h_in = 0
        dis_in = bji.junction_distance[node][des] - bji.junction_distance[next_node][des]
        if dis_in < 0:
            dis_in = 0
        return den_in, h_in, dis_in
    else:
        return -1, -1, -1


def junction_reward(r, des):
    for i in range(268):
        for j in range(268):
            if bji.adj_martix[i][j] != 0:
                a, b, c = cal_weight(i, j, des)
                if a == 0:
                    r[i][j] = 0
                else:
                    r[i][j] = alpha*(a+b) + beta*c


def hidden_seq_generate(reward, source, des):
    if source > des:
        flag = 1
    else:
        flag = 0
    g = nx.DiGraph()
    for i in range(268):
        for j in range(268):
            if reward[i][j] != 0:
                g.add_edge(i, j, weight=1000-reward[i][j])
    try:
        a = nx.shortest_path(g, source=source, target=des)
    except nx.NetworkXNoPath as err:
        a = None
    else:
        a = nx.shortest_path(g, source=source, target=des)
    # a =dij.Dijkstra(g, source, des)
    print("intersection1:")
    print(a)
    if(a!=None):
        for i in range(len(a)-1):
            bji.chosen_edge[a[i]][a[i+1]] = 1
            g.remove_edge(a[i], a[i+1])
    try:
        b = nx.shortest_path(g, source=source, target=des)
    except nx.NetworkXNoPath as err:
        b = None
    else:
        b = nx.shortest_path(g, source=source, target=des)
    # b = dij.Dijkstra(g, source, des)
    print("intersection2:")
    print(b)
    return a, b


def cal_dis(i, j, node_list):
    return pow(node_list[i].position[0] - node_list[j].position[0], 2) + pow(node_list[i].position[1] - node_list[j].position[1], 2)


def hidden_to_obverse(source_vehicle, des_vehicle, node_list, hidden_seq):
    if hidden_seq is None:
        return
    for i in range(len(hidden_seq)-1):
        if i == 0:
            for nodei in bji.junction_vehicle[hidden_seq[i + 1]]:
                node = int(nodei)
                d = cal_dis(source_vehicle, node, node_list)
                if d < pow(Gp.com_dis, 2):
                    insort_right(bji.edge_list, edge(source_vehicle, node, bji.e_arrival_time[source_vehicle], d))
                    if bji.e_arrival_time[node] > (bji.e_arrival_time[source_vehicle] + d):
                        bji.e_arrival_time[node] = bji.e_arrival_time[source_vehicle] + d
        else:
            if i == len(hidden_seq)-1:
                for nodei in bji.junction_vehicle[hidden_seq[i]]:
                    node = int(nodei)
                    d = cal_dis(des_vehicle, node, node_list)
                    if d < pow(Gp.com_dis, 2):
                        insort_right(bji.edge_list, edge(node, des_vehicle, bji.e_arrival_time[node], d))
                        if bji.e_arrival_time[des_vehicle] > (bji.e_arrival_time[node] + d):
                            bji.e_arrival_time[des_vehicle] = bji.e_arrival_time[node] + d
            else:
                for s_nodei in bji.junction_vehicle[hidden_seq[i]]:
                    s_node = int(s_nodei)
                    for d_nodei in bji.junction_vehicle[hidden_seq[i+1]]:
                        d_node = int(d_nodei)
                        d = cal_dis(s_node, d_node, node_list)
                        if d < pow(Gp.com_dis, 2):
                            insort_right(bji.edge_list, edge(s_node, d_node, bji.e_arrival_time[s_node], d))
                            if bji.e_arrival_time[d_node] > (bji.e_arrival_time[s_node] + d):
                                bji.e_arrival_time[d_node] = bji.e_arrival_time[s_node] + d


def delete():
    bji.edge_list.clear()
    for i in range(600):
        bji.e_arrival_time[i] = Gp.MAX
    for i in range(268):
        bji.junction_vehicle[i].clear()
        for j in range(4):
            bji.num_segement_martix[i][j] = 0
        for j in range(268):
            bji.chosen_edge[i][j] = 0
