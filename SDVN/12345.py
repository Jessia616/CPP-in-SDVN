import math
import numpy as np
import matplotlib.pyplot as plt
import big_junction_init as bji
import junction_init as ji
import Global_Par as Gp
import linecircle as lc
import re
import Get_Move as Gm
# from matplotlib.lines import Line2D

rect_list = []
draw_list = []
time = 0


class Stack(object):
    """栈"""

    def __init__(self):
        self.items = []

    def is_empty(self):
        """判断是否为空"""
        return self.items == []

    def push(self, item):
        """加入元素"""
        self.items.append(item)

    def pop(self):
        """弹出元素"""
        return self.items.pop()

    def peek(self):
        """返回栈顶元素"""
        return self.items[self.items.__len__() - 1]

    def size(self):
        """返回栈的大小"""
        return int(self.items.__len__())


def getstack(adj, i, n, s):
    for j in range(n):
        if adj[i][j] == 1:
            s.push(j)
    return s.size()


def fish_placement(xmin, ymin, xmax, ymax, size):
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
    for i in range(int((w + Gp.con_dis) / (Gp.con_dis * 2))):
        for j in range(int((h + Gp.con_dis) / (Gp.con_dis * 2))):
            posi_list.append(
                [xmax - Gp.con_dis - Gp.con_dis * 2 * i, ymax - Gp.con_dis - Gp.con_dis * 2 * j])

    return posi_list


## new 长方形覆盖判断 ，有重叠返回1 (*ﾟ∇ﾟ) ​​​​
def isOverlap(minx0, miny0, maxx0, maxy0, minx1, miny1, maxx1, maxy1):
    if (maxx0 > minx1) and (maxx1 > minx0) and (maxy0 > miny1) and (maxy1 > miny0):
        return 1
    else:
        return 0


# # 圆的基本信息
# # 1.圆半径
# r = 250
# # 2.圆心坐标
# a, b = (0., 0.)
# # ==========================================
# # 方法一：参数方程
# plt.scatter(poi_num[:,0],poi_num[:,1],s = 23000,alpha=0.5, edgecolors= 'blue')
# plt.title('www.jb51.net')
# plt.show()


def initial_placement(intersection_position, intersection_matrix):
    x0 = 99990
    x1 = 0
    y0 = 99990
    y1 = 0
    for i in intersection_position:
        if i[0] < x0:
            x0 = i[0]
        if i[1] < y0:
            y0 = i[1]
        if i[0] > x1:
            x1 = i[0]
        if i[1] > y1:
            y1 = i[1]
    ## 多控制器初始化  ​​​​
    draw_list.clear()
    rect_list.clear()
    position_list = []
    no = 0
    ## 初始布置算法
    chushu = 0.00019 * Gp.con_dis * Gp.con_dis
    ## 在所有路口出放置控制器 (=ﾟωﾟ)=b
    for key, i in enumerate(intersection_position):
        a = [i[0], i[1]]
        position_list.append(a)
    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    num = 0
    for x, y in zip(a[:, 0], a[:, 1]):
        plt.text(x, y + 0.3, str(num), ha='center', va='bottom', fontsize=10.5)
        num += 1
    plt.show()

    ## 对于路口间路段长度过长的（>= 2*通信范围），补充控制器以实现全覆盖。(｀･ω･)
    c_n = intersection_position.__len__()
    for i in range(c_n):
        for j in range(c_n):
            if intersection_matrix[i][j] > Gp.con_dis * 2:
                ##（>= 4*通信范围）以圆相切的方式补充控制器
                if intersection_matrix[i][j] > Gp.con_dis * 4:
                    len = intersection_matrix[i][j]
                    k = (intersection_position[j][1] - intersection_position[i][1]) / \
                        (intersection_position[j][0] - intersection_position[i][0])
                    x = math.sqrt(pow(Gp.con_dis * 2, 2) / (k * k + 1))
                    ## 两路口谁长谁高 作比较，做标记
                    if intersection_position[j][0] > intersection_position[i][0]:
                        x_flag = 1
                    else:
                        x_flag = -1

                    if intersection_position[j][1] > intersection_position[i][1]:
                        y_flag = 1
                    else:
                        y_flag = -1
                    c_x = intersection_position[i][0]
                    c_y = intersection_position[i][1]
                    ## 剩余路段长度放得下时，循环添加控制器
                    while len >= 0:
                        c_x += x_flag * x
                        c_y += y_flag * k * x
                        a = [c_x, c_y]
                        position_list.append(a)
                        len -= Gp.con_dis * 2
                    c_x += x_flag * x
                    c_y += y_flag * k * x
                    a = [c_x, c_y]
                    position_list.append(a)
                    ## 在最后一个坐标与j中间放一个
                ##（<= 4*通信范围 and >= 2*通信范围）时，两路口中点放置控制器，即可全覆盖
                else:
                    a = [(intersection_position[i][0] + intersection_position[j][0]) / 2, \
                         (intersection_position[i][1] + intersection_position[j][1]) / 2]
                    position_list.append(a)
    n_best = 0
    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    plt.show()

    ## 过于拥挤时，减少多余控制器，循环：判断密集控制器，删除密集控制器，鱼鳞布置新控制器 (*´∀`) ​​​​
    for time in range(100):
        n = position_list.__len__()
        edge_c = 0
        adj = [[0 for i in range(n)] for i in range(n)]
        dis = 0

        ## 两路口间距离< 通信距离时，即两路口过于接近，在adj邻接矩阵中做标记  ​​​​
        for key_i, i in enumerate(position_list):
            for key_j, j in enumerate(position_list):
                if key_i != key_j:
                    dis = pow(i[0] - j[0], 2) + pow(i[1] - j[1], 2)
                    if dis < pow(Gp.con_dis, 2):
                        adj[key_i][key_j] = 1
                        adj[key_j][key_i] = 1
                        print([key_i], [key_j])

        delete_list = []
        add_list = []
        m = []
        ress = []
        s = Stack()

        count = 0
        n = position_list.__len__()

        ## 根据adj邻接矩阵，划分其路口图的联通子集，并将其转换为最小的长方形，以覆盖子集中所有成员 (σ`∀´) ​​​​
        for i in range(n):
            if m.count(i) >= 1:
                continue
            else:
                m.append(i)
            result = getstack(adj, i, n, s)
            if result == 0:
                ress.append([i])
                # print([i])
                count += 1
            else:
                relay = [i]
                while s.size() != 0:
                    temp = s.peek()
                    s.pop()
                    if m.count(temp) == 0:
                        relay.append(temp)
                        m.append(temp)
                        getstack(adj, temp, n, s)
                print(relay)
                ## new!! 子集中节点为2个，进行判断，如果两点过于接近，就删除两点，取其中点，否则保留两点。 (*ﾟ∇ﾟ) ​​​​
                if relay.__len__() == 2:
                    d = pow((position_list[relay[0]][0] - position_list[relay[1]][0]), 2) + pow(
                        (position_list[relay[0]][1] - position_list[relay[1]][1]), 2)
                    if d <= pow(Gp.con_dis / 2, 2):
                        add_list.append([(position_list[relay[0]][0] + position_list[relay[1]][0]) / 2,
                                         (position_list[relay[0]][1] + position_list[relay[1]][1]) / 2])
                        delete_list.append(relay[0])
                        delete_list.append(relay[1])
                    continue
                for node in relay:
                    draw_list.append(node)
                ## 绘制长方形
                min_x = 99999
                min_y = 99999
                max_x = 0
                max_y = 0
                for i in relay:
                    # delete_list.append(position_list[i])
                    if position_list[i][0] > max_x:
                        max_x = position_list[i][0]
                    if position_list[i][1] > max_y:
                        max_y = position_list[i][1]
                    if position_list[i][0] < min_x:
                        min_x = position_list[i][0]
                    if position_list[i][1] < min_y:
                        min_y = position_list[i][1]
                    ## new!!适当扩大长方形面积，边缘更好地与其他长方形适应 (*ﾟ∇ﾟ) ​​​​
                    if min_x - (Gp.con_dis / chushu) >= x0:
                        min_x -= (Gp.con_dis / chushu)
                    if min_y - (Gp.con_dis / chushu) >= y0:
                        min_y -= (Gp.con_dis / chushu)
                    if max_x + (Gp.con_dis / chushu) <= x1:
                        max_x += (Gp.con_dis / chushu)
                    if max_y + (Gp.con_dis / chushu) <= y1:
                        max_y += (Gp.con_dis / chushu)
                    # min_x -= (Gp.con_dis / chushu)
                    # min_y -= (Gp.con_dis / chushu)
                    # max_x += (Gp.con_dis / chushu)
                    # max_y += (Gp.con_dis / chushu)
                    # # min_x -= (Gp.con_dis / 12)
                    # # min_y -= (Gp.con_dis / 12)
                    # # max_x += (Gp.con_dis / 12)
                    # # max_y += (Gp.con_dis / 12)
                rect_list.append([min_x, min_y, max_x, max_y, 0])
                ## 对长方形内面积实现鱼鳞布置

        ## new!! 所有子集所属长方形进行判断，如互相重叠，即融合两个长方形 (*ﾟ∇ﾟ) ​​​​
        rett = []
        for tt in range(10):
            for ri in rect_list:
                if ri[4] != 0:
                    continue
                for rj in rect_list:
                    if ri == rj:
                        continue
                    if rj[4] != 0:
                        continue
                    if (isOverlap(ri[0], ri[1], ri[2], ri[3], rj[0], rj[1], rj[2], rj[3]) == 1):
                        ri[4] = 1
                        rj[4] = 1
                        ## 融合
                        rett.append([min(ri[0], rj[0]), min(ri[1], rj[1]), max(ri[2], rj[2]), max(ri[3], rj[3]), 0])
            for r in rect_list[::-1]:
                if r[4] == 1:
                    rect_list.remove(r)
            for r in rett:
                rect_list.append(r)
            rett.clear()
        ## new!! 根据新的子集长方形，进行鱼鳞分布 (*ﾟ∇ﾟ) ​​​​
        for rr in rect_list:
            add = fish_placement(rr[0], rr[1], rr[2], rr[3], 0)
            for i in add:
                add_list.append(i)
        ## new!! 所有长方形范围中节点全部删除 (*ﾟ∇ﾟ) ​​​​
        for r in rect_list:
            for node in position_list:
                if node[0] >= r[0] and node[0] <= r[2] and node[1] >= r[1] and node[1] <= r[3]:
                    delete_list.append(node)

        a = np.array(position_list)
        fig = plt.figure(figsize=(8, 7.82), dpi=80)
        plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, edgecolors='blue')
        plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
        for r in rect_list:
            plt.gca().add_patch(plt.Rectangle(xy=(r[0], r[1]), width=r[2] - r[0], height=r[3] - r[1],
                                              fill=False, linewidth=2))
        plt.show()
        rect_list.clear()
        print("original num:")
        print(position_list.__len__())

        ## 删除子集内所有原控制器

        for i in position_list[::-1]:
            time1 = 0
            if delete_list.count(i) >= 1:
                # for node in position_list:
                #     if time1 >= 2:
                #         break
                #     d = pow(i[0] - node[0], 2) + pow(i[1] - node[1], 2)
                #     if d <= pow(Gp.con_dis,2):
                #         time1 += 1
                # if time1 >= 2:
                position_list.remove(i)
        print("delete num:")
        print(position_list.__len__())
        ## 将鱼鳞布置控制器加入
        for iii in add_list:
            position_list.append(iii)
        print("polish num:")
        print(position_list.__len__())
        # a = np.array(position_list)
        # fig = plt.figure(figsize=(8, 7.82), dpi=80)
        # plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis*Gp.con_dis*0.024, edgecolors='blue')
        # plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
        # plt.show()
        number = position_list.__len__()
        ## 如果结果稳定，即视为最优解，不然继续循环：判断密集控制器，删除密集控制器，鱼鳞布置新控制器
        if number == n_best:
            time += 1
            if time >= 10:
                break
        else:
            n_best = number

    edge_list = []
    ## 统计所有路口间路段，起始点坐标 (〃∀〃) ​​​​
    for i in range(c_n):
        for j in range(i):
            if intersection_matrix[i][j] != 0:
                edge_list.append([intersection_position[i], intersection_position[j], 0])

    flag = 0

    ## 对每个控制器判断，自身是否覆盖到任一路段，是否自身为无用控制器，对路段自身也做判断，无控制器覆盖则做标记 (*´ω`*) ​​​​
    for con in position_list[::-1]:
        for edge in edge_list:
            if lc.Judis(edge[0][0], edge[0][1], edge[1][0], edge[1][1], con[0], con[1], Gp.con_dis) == 1:
                flag += 1
                edge[2] = 1
        ## 删除所有无效控制器
        if flag == 0:
            print(con)
            position_list.remove(con)
        flag = 0
    ## 将无控制器负责的路段 在路段中间架设控制器 (ゝ∀･)☆ ​​​​
    for edge in edge_list:
        if edge[2] == 0:
            position_list.append([(edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2])
            print(edge)

    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    plt.show()
    ## 避免过于密集，再次鱼鳞  (＾o＾)ﾉ ​​​​
    n = position_list.__len__()
    edge_c = 0
    adj = [[0 for i in range(n)] for i in range(n)]
    dis = 0
    for con in position_list:
        con.append(0)
        con.append(0)
    time = 0
    ve = 0
    flag = 0
    movement_matrix, init_position_matrix = Gm.get_position('tiexi1.tcl')
    time_topo = [[] for i in range(10000)]
    for i in movement_matrix:
        time_topo[int(i[0, 0])].append([i[0, 2], i[0, 3]])

    ## 统计sumo每一时刻数据中车辆位置数据，对于孤岛节点用controller中第四个属性做标记 (*´∀`) ​​​​
    for graph in time_topo:
        for node in graph:
            for nodei in graph:
                if node != nodei:
                    if pow(nodei[1] - node[1], 2) + pow(nodei[0] - node[0], 2) <= pow(Gp.com_dis, 2):
                        flag = 1
                        break
            if flag == 0:
                for con in position_list:
                    if pow(con[1] - node[1], 2) + pow(con[0] - node[0], 2) <= pow(Gp.con_dis, 2):
                        con[3] += 1
            flag = 0

    ## 对车辆拓扑进行长时间统计，对控制范围下车辆过于稀疏的控制器进行删除处理 (*´∀`) ​​​​
    with open("tiexi1.tcl", 'r') as f:
        item_list = []
        for line in f:
            line_list = re.split('[\s]', line)
            if line_list[0] != '':
                if (float(line_list[2]) > time):
                    time = float(line_list[2])
                if (float(line_list[3][8:-1]) > ve):
                    ve = float(line_list[3][8:-1])
                item_list.append(float(line_list[2]))
                item_list.append(float(line_list[3][8:-1]))
                item_list.append(float(line_list[5]))
                item_list.append(float(line_list[6]))
                item_list.append(float(line_list[7][0:-1]))
                print(item_list)
                ## 统计sumo每一时刻数据中车辆位置数据，对于车流量用controller中第三个属性做标记 *･ﾟ(*´ω`*)･
                for con in position_list:
                    d = pow(con[0] - item_list[2], 2) + pow(con[1] - item_list[3], 2)
                    if d < pow(Gp.con_dis, 2):
                        con[2] += 1
                item_list.clear()
                flag = 0

    ## 可能删去的偏僻控制器 *･ﾟ(*´ω`*)･
    position_list_can = []
    ## 车流量大需要固定的控制器 (*ﾟ∇ﾟ)♡ ​​​​
    position_list_fixed = []

    for con in position_list[::-1]:
        # 阈值应根据多个因素做调整，包括通信半径，数据包含时长，数据总车辆数等，车流量小且孤岛节点多的控制器删去 (　ﾟ 3ﾟ) ​​​​
        if con[2] <= (pow(Gp.con_dis, 2) * math.pi) / ((x1 - x0) * (y1 - y0)) * ve * time / 6 and con[3] > (
                (pow(Gp.con_dis, 2)) / ((x1 - x0) * (y1 - y0)) * ve * time / 14.3):
            position_list_can.append(con)
            position_list.remove(con)
        # 车流量大的固定
        if con[2] >= (pow(Gp.con_dis, 2) * math.pi) / ((x1 - x0) * (y1 - y0)) * ve * time / 3:
            position_list_fixed.append(con)
            position_list.remove(con)

    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024)
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_fixed)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='y', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_can)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='m', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    plt.show()

    ## 对车流量大的控制器进行标记，相互之间距离近的连接 (＾o＾)ﾉ
    n = position_list_fixed.__len__()
    fixed_position_adj = [[0 for i in range(n)] for i in range(n)]
    for numi, i in enumerate(position_list_fixed):
        for numj, j in enumerate(position_list_fixed):
            if pow(j[1] - i[1], 2) + pow(j[0] - i[0], 2) <= pow(Gp.con_dis * 2, 2):
                fixed_position_adj[numi][numj] = 1

    delete_list = []
    add_list = []
    m = []
    ress = []
    s = Stack()
    rect_list.clear()
    count = 0
    ## 与上文同样的步骤，联通子集，画长方形 (`ε´ ) ​​​​
    for i in range(n):
        if m.count(i) >= 1:
            continue
        else:
            m.append(i)
        result = getstack(fixed_position_adj, i, n, s)
        if result == 0:
            ress.append([i])
            # print([i])
            count += 1
        else:
            relay = [i]
            while s.size() != 0:
                temp = s.peek()
                s.pop()
                if m.count(temp) == 0:
                    relay.append(temp)
                    m.append(temp)
                    getstack(fixed_position_adj, temp, n, s)
            print(relay)
            for node in relay:
                draw_list.append(node)
            ## 绘制长方形
            min_x = 99999
            min_y = 99999
            max_x = 0
            max_y = 0
            for i in relay:
                # delete_list.append(position_list_fixed[i])
                if position_list_fixed[i][0] > max_x:
                    max_x = position_list_fixed[i][0]
                if position_list_fixed[i][1] > max_y:
                    max_y = position_list_fixed[i][1]
                if position_list_fixed[i][0] < min_x:
                    min_x = position_list_fixed[i][0]
                if position_list_fixed[i][1] < min_y:
                    min_y = position_list_fixed[i][1]
                # min_x -= (Gp.con_dis / chushu)
                # min_y -= (Gp.con_dis / chushu)
                # max_x += (Gp.con_dis / chushu)
                # max_y += (Gp.con_dis / chushu)
                # # min_x -= (Gp.con_dis / 12)
                # # min_y -= (Gp.con_dis / 12)
                # # max_x += (Gp.con_dis / 12)
                # # max_y += (Gp.con_dis / 12)
            rect_list.append([min_x, min_y, max_x, max_y, 0])
            ## 对长方形内面积实现鱼鳞布置
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024)
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_fixed)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='y', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_can)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='m', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    for r in rect_list:
        plt.gca().add_patch(plt.Rectangle(xy=(r[0], r[1]), width=r[2] - r[0], height=r[3] - r[1],
                                          fill=False, linewidth=2))
    plt.show()
    # ## new!! 所有子集所属长方形进行判断，如互相重叠，即融合两个长方形 (*ﾟ∇ﾟ) ​​​​
    # rett = []
    # for tt in range(10):
    #     for ri in rect_list:
    #         if ri[4] != 0:
    #             continue
    #         for rj in rect_list:
    #             if ri == rj:
    #                 continue
    #             if rj[4] != 0:
    #                 continue
    #             if (isOverlap(ri[0], ri[1], ri[2], ri[3], rj[0], rj[1], rj[2], rj[3]) == 1):
    #                 ri[4] = 1
    #                 rj[4] = 1
    #                 ## 融合
    #                 rett.append([min(ri[0], rj[0]), min(ri[1], rj[1]), max(ri[2], rj[2]), max(ri[3], rj[3]), 0])
    #     for r in rect_list[::-1]:
    #         if r[4] == 1:
    #             rect_list.remove(r)
    #     for r in rett:
    #         rect_list.append(r)
    #     rett.clear()
    ## new!! 根据新的子集长方形，进行鱼鳞分布 (*ﾟ∇ﾟ) ​​​​
    for rr in rect_list:
        xx = range(int(rr[0]), int(rr[2]), int(4 * Gp.con_dis))
        yy = range(int(rr[1]), int(rr[3]), int(4 * Gp.con_dis))
        ## 将长方形分割成同样大小的长方形，如果长方形内固定控制器超过一定数量,删去之中所有节点，稀疏布置 σ`∀´) ​​​​
        for i in xx:
            for j in yy:
                count = 0
                for con in position_list_fixed:
                    if i + 4 * Gp.con_dis >= con[0] >= i and j <= con[1] <= j + 4 * Gp.con_dis:
                        count += 1
                if count >= 4:
                    for con in position_list_fixed[::-1]:
                        if i + 4 * Gp.con_dis >= con[0] >= i and j <= con[1] <= j + 4 * Gp.con_dis:
                            position_list_fixed.remove(con)
                    for con in position_list[::-1]:
                        if i + 4 * Gp.con_dis >= con[0] >= i and j <= con[1] <= j + 4 * Gp.con_dis:
                            position_list.remove(con)
                    position_list_fixed.append([i + Gp.con_dis, j + Gp.con_dis, 0, 0])
                    position_list_fixed.append([i + 3 * Gp.con_dis, j + Gp.con_dis, 0, 0])
                    position_list_fixed.append([i + 3 * Gp.con_dis, j + 3 * Gp.con_dis, 0, 0])
                    position_list_fixed.append([i + Gp.con_dis, j + 3 * Gp.con_dis, 0, 0])

    a = np.array(position_list)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024)
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_fixed)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='y', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')

    a = np.array(position_list_can)
    # fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='m', edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    for r in rect_list:
        plt.gca().add_patch(plt.Rectangle(xy=(r[0], r[1]), width=r[2] - r[0], height=r[3] - r[1],
                                          fill=False, linewidth=2))
    plt.show()

    ## 对固定控制器之中较为靠近控制器对进行稀疏分布  (*ﾟ∇ﾟ) ​​​​
    for numi,i in enumerate(position_list_fixed):
        for numj,j in enumerate(position_list_fixed):
            if numi < numj:
                d = pow(i[0] - j[0], 2) + pow(i[1] - j[1], 2)
                if d <= pow(Gp.con_dis, 2):
                    add_list.append([(i[0] + j[0]) / 2, (i[1] + j[1]) / 2, 0,0])
                    delete_list.append(i)
                    delete_list.append(j)
        continue

    for i in add_list:
        position_list_fixed.append(i)
    for j in delete_list:
        position_list_fixed.remove(j)
    ## 返回所有控制器位置，算法完成 (＾o＾)ﾉ ​​​​
    return position_list, position_list_fixed


bji.inti()
ji.inti()
a = []
# a = initial_placement(ji.junction_position, ji.adj_martix)
a, b = initial_placement(bji.junction_position, bji.junction_adj_ma)
n = a.__len__()
n += b.__len__()
print(n)
# for key_i, i in enumerate(a):
#     for key_j, j in enumerate(a):
#         if key_i != key_j:
#             dis = pow(i[0] - j[0], 2) + pow(i[1] - j[1], 2)
#             if dis < pow(Gp.con_dis, 2):
#                print([key_i], [key_j])

a = np.array(a)
fig = plt.figure(figsize=(8, 7.82), dpi=80)
plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.025, edgecolors='blue')
plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
a = []

a = np.array(b)
plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=Gp.con_dis * Gp.con_dis * 0.024, c='y', edgecolors='blue')
plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
plt.show()
