#routing test
#before this run file_io
c_count = 0
v_count = 0
import random
import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import big_junction_init as bji
import big_jhmmtg as bjh
import big_HRLB as bhr
import HRLB as hr
import math
import tgeaa as tg
import networkx as nx
node_list = []
com_node_list = []
onc = []
sum = 0
flag = 1
delay_sum = 0
sim_time = 200  # int(input("sim_time:"))
cost = 0
head = []
tail = []
sshop = []
ssshop = 0
minh = 9999999
maxh = 0
with open('big_1000.tcl', 'r') as f:
    for line in f:
        print(line)
        if line[2] == 'o':
            tail.append(line)
        else:
            head.append(line)

print('1')

with open('tiexi1.tcl', 'w') as f:
    for line in head:
        f.write(line)
    for line in tail:
        f.write(' ')
        f.write(line)

# 位置文件读取
movement_matrix, init_position_matrix = Gm.get_position('tiexi1.tcl')
node_num = init_position_matrix.shape[0]
# 控制器初始化
controller = Init.init_controller(node_num)

# 位置数据处理
init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]

node_position = init_position_arranged[0]
total_route = []
shop = 0
# node_position = np.insert(node_position, 0, values=np.zeros(node_num), axis=1)
# node_position = np.column_stack((node_position, node_position[:, 2:4]))
# node_position = np.insert(node_position, 6, values=np.zeros(node_num), axis=1)

# ji.inti()
## 根据下一跳列表递归地寻找到最近控制器的路径，并检验是否可行
def v2vfor(node,hop,next,controller_position,v_c,node_list,road):
    print(node)
    road.append(node)
    ## 如果hop为0，说明无法找到路径
    if hop[node] == 0:
        return []
    ## 如果hop为1，已有控制器在该车范围内
    if hop[node] == 1:
        d = math.sqrt(pow(controller_position[v_c[node]][0] - node_list[node].position[0], 2) + pow(controller_position[v_c[node]][1] - node_list[node].position[1], 2))
        ## 距离够的话，转发成功
        if d < Gp.com_dis:
            return road
        return []
    ## 如果不为1，也不为0，根据next寻找下一跳，计算之间距离
    n = next[node]
    d = math.sqrt(pow(node_list[n].position[0] - node_list[node].position[0], 2) + pow(
        node_list[n].position[1]- node_list[node].position[1], 2))
    ## 如果距离小于最小距离，可传输，递归寻找
    if d < Gp.com_dis:
        if(len(v2vfor(n,hop,next,controller_position,v_c,node_list,road))!= 0 ):
            return road
        else:
            return []
    ## 如果距离大于最小距离，可传输失败
    else:
        return 0
def v2vfor_1(node,hop,next,controller_position,v_c,node_list):
    if hop[node] == 0:
        return 0
    if hop[node] == 1:
        d = math.sqrt(pow(controller_position[v_c[node]][0] - node_list[node].position[0], 2) + pow(controller_position[v_c[node]][1] - node_list[node].position[1], 2))
        if d < Gp.com_dis*2:
            return 1
        return 0
    n = next[node]
    d = math.sqrt(pow(node_list[n].position[0] - node_list[node].position[0], 2) + pow(
        node_list[n].position[1] - node_list[node].position[1], 2))
    if d < Gp.com_dis*2:
        a = v2vfor(n,hop,next,controller_position,v_c,node_list)
        print(node)
        if a == 1:
            return 1
        else:
            return 0
    else:
        return 0
bji.inti()
hr.grid_intiall()
# 节点初始化
node_list = (Init.init_node(node_position, controller))
effi = 0
delay = 0
std2 = 0
# 生成通信节点
# 输入循环次数
# a = np.loadtxt('com.txt')
# com_node_list.extend(a)
round = 10
for i in range(round):
    # 控制器初始化
    com_node_list.clear
    com_node_list.extend(Init.get_communication_node(node_num - 1))
    controller = Init.init_controller(node_num)
    ## 根据路口情况，初始化所有控制器初始位置
    controller.initial_placement(bji.junction_position, bji.junction_adj_ma)

    # 以秒为间隔进行
    for time in range(0, sim_time):
        print('\nTime: %d' % time)
        # with open('history.txt', 'a') as f:
        #     a = ""
        #     a += str(time)
        #     a += '\n'
        #     f.write(a)
        # 处理位置矩阵
        current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
        for value in current_move:
            for i in range(1, 4):
                node_position[int(value[0, 1]), i] = value[0, i+1]
        node_id_position = node_position[:, [1, 2, 3]]
        X = np.array(node_id_position)
        X = X.transpose()
        m, n = X.shape
        G = np.dot(X.T, X)
        # 把G对角线元素拎出来，列不变，行复制n遍。
        H = np.tile(np.diag(G), (n, 1))
        D = H + H.T - G * 2
        D = np.sqrt(D)
        ## 构建控制器之间的传输图
        node_g = nx.DiGraph()
        for i in range(len(node_id_position)):
            for j in range(len(node_id_position)):
                if i < j:
                    if D[i][j] <= 350:
                        node_g.add_edge(i, j, weight=D[i][j])
                        node_g.add_edge(j, i, weight=D[i][j])
        # print(node_id_position[44])
        # 所有节点更新位置，并发送hello至控制器
        for node in node_list:
            node.update_node_position(node_id_position)
            node.generate_hello(controller)
        jh.num_count()
        # 控制器更新网络全局情况
        controller.predict_position()
        # controller.junction_matrix_construction(node_num)
        if time % 50 == 0:
            ## 如果达到条件，对可调整控制器位置进行多目标ABC，获得开关情况
            toc,onc1 = controller.controller_place(round)
            onc.append(onc1)
            ## 计算控制器间距离
            X = np.array(controller.controller_position)
            X = X.transpose()
            m, n = X.shape
            G = np.dot(X.T, X)
            # 把G对角线元素拎出来，列不变，行复制n遍。
            H = np.tile(np.diag(G), (n, 1))
            D = H + H.T - G * 2
            D = np.sqrt(D)
            ## 构建控制器之间的传输图
            con_g = nx.DiGraph()
            for i in range(len(controller.controller_position)):
                for j in range(len(controller.controller_position)):
                    if i < j:
                        if D[i][j] <= 1000:
                            con_g.add_edge(i, j, weight=D[i][j])
                            con_g.add_edge(j, i, weight=D[i][j])
            flag = 0
        ## 统计各个车辆在当前秒中的负载
        v_load = [0 for i in range(len(node_list))]
        ## 统计各个控制器在当前秒中的负载
        c_load = [0 for i in range(len(controller.controller_position))]
        start_time = t.time()
        ## 根据算法获得，车辆控制器从属关系，车辆到最近控制器跳数，车辆到最近控制器的路径（由下一跳列表表示）
        hop, next_hop = controller.vehicle_controller()
        desv = 0
        dec = 0
        dedv = 0
        ## 统计当前秒数 所传输的包的所有路径，a为源-控制器，b为控制器-目的，c为控制间路径
        total_a_route = []
        total_b_route = []
        total_c_route = []
        total_hop = []
        ## 根据每秒产生包数做循环，发送n个包，在此循环中不计算时延，只统计传输成功与否，和跳数
        ## 时延等本秒传输全部结束后，再根据记录下的路径 统一根据各控制器和处理的负载统计
        for k in range(Gp.pps):
            a = 0
            b = 0
            c = 0
            nnn = int(random.randint(0,len(com_node_list)-1))
            s = node_list[int(com_node_list[nnn][0])]
            d = node_list[int(com_node_list[nnn][1])]

            a_road = []
            ## 获取由源节点向最近控制器传输，是否成功，a=1为成功，0为失败
            a_road = (v2vfor(s.node_id,hop,next_hop,controller.controller_position,controller.v_c,node_list,a_road))
            ## 如成功计算出来路径，a标识设为1
            if len(a_road) != 0 and a_road:
                a = 1
            ## 对路径上的相关车辆的负载加1
            for i in a_road:
                if i != s.node_id:
                    v_load[i] += 1
            ## 计算源节点向最近控制器传输时延
            print("------------")

            ## 获取源节点，与目标节点最近的控制器索引
            sc = controller.v_c[s.node_id]
            dc = controller.v_c[d.node_id]
            ## 计算控制器间传输最短路径
            try:
                nx.shortest_path(con_g, source=sc, target=dc)
            except nx.NodeNotFound as err1:
                route = None
                c = 0
            except nx.NetworkXNoPath as err2:
                route = None
                c = 0
            else:
                ## 获取路径与距离
                route = nx.shortest_path(con_g,source=sc,target=dc)
                diss = nx.dijkstra_path_length(con_g,source=sc,target=dc)
                if route:
                    ## 根据跳数与距离计算时延
                    print(route)
                    c = 1
                    ## 对路径上的相关控制器的负载加1
                    for i in route:
                        c_load[i] += 1
                    chop = len(route)-1
                else:
                    ## 如果控制器间转发失败，标识变为0
                    c = 0
            print("------------")

            ## 获取由目的节点向最近控制器传输，是否成功，b=1为成功，0为失败
            b_road = []
            ## 获取由源节点向最近控制器传输，是否成功，a=1为成功，0为失败
            b_road = (v2vfor(d.node_id, hop, next_hop, controller.controller_position, controller.v_c, node_list, b_road))
            ## 如成功计算出来路径，b标识设为1，并且将此路径反转
            if len(b_road) != 0 and b_road:
                b = 1
                b_road.reverse()
            ## 对路径上的相关车辆的负载加1
            for i in b_road:
                if i != d.node_id:
                    v_load[i] += 1
            ## a,b,c三者相乘，即可得知转发是否成功
            if a!=0 and b!=0 and c!= 0:
                sum += 1
            ## 如果三标识相乘为1，转发成功1，三段路径存入total中
            # if a * b * c == 1:
                c_count += 1
                print("------------")
                print("successful transmission")
                total_a_route.append(a_road)
                total_b_route.append(b_road)
                total_c_route.append(route)
                hop11 = hop[d.node_id] + hop[s.node_id] + chop
                if hop11 > maxh:
                    maxh = hop11
                if hop11 < minh:
                    minh = hop11
                total_hop.append(hop11)
                shop += hop11
                print('hop:')
                print(hop11)
            else:
                delay_sum += 45
                try:
                    nx.shortest_path(node_g,source=s.node_id,target=d.node_id)
                except nx.NodeNotFound as err1:
                    route = None
                except nx.NetworkXNoPath as err2:
                    route = None
                else:
                    route = nx.shortest_path(node_g,source=s.node_id,target=d.node_id)
                if route != None:
                    v_count += 1
                    for i in range(len(route) - 1):
                        ## 同样的，aa为由于当前车辆负载所产生的额外等待时间
                        aa = 1
                        ## 传输时延+额外等待时间
                        delay_1 = (224 + 0.0023629942501200486 + 30.845634559999993) / 1000 * aa
                        dis = math.sqrt(pow(node_list[i].position[0] - node_list[i + 1].position[0], 2) + pow(
                            node_list[i].position[1] - node_list[i + 1].position[1], 2))
                        delay_1 += ((dis / 1000 * 5 + dis / 1000 * 5 * 2 * 0.4169999999999999) / 1000) * aa
                        delay_1 += 0.16 * aa
                        delay_1 += 45
                    sum += 1
                    delay_sum += delay_1
                    Gp.record.append(delay_1)
                    ssshop += (len(route) - 1)
                    cost += 0.5973 * (len(route) - 1)
        ## 统计计算时间，由于hop的计算一秒只需要一次，秒发包率越高，效率越高
        end_time = t.time()
        effi += end_time - start_time

        for i in c_load:
            cost += 5 + 9.95*(2*i/20-pow(i/20,2))

        ## 开始统计本秒内所有转发成功的包的时延
        for j in range(len(total_a_route)):
            ## 遍历total，以获得各个包的三段路径
            a_road = total_a_route[j]
            b_road = total_b_route[j]
            route = total_c_route[j]

            ## 这里算由最后一个车辆到控制器的时延
            ## aa为由于当前车辆负载所产生的额外等待时间
            aa = ((v_load[(a_road[len(a_road) - 1])] - 1) / 2 + 1)
            ## 在原有时延计算基础上加上额外等待时间
            desv = (224 + 0.0023629942501200486 + 29.799999999999997) / 1000 * aa
            desv += ((controller.v_c_d[(a_road[len(a_road) - 1])] / 1000 * 5 + (
            controller.v_c_d[(a_road[len(a_road) - 1])]) / 1000 * 5 * 2 * 0.4169999999999999) / 1000) * aa
            ## 发送时延，同样算上等待时间
            desv += 0.16 * aa

            ## 这里计算的是车辆间中继时延
            if len(a_road) != 1:
                for i in range(len(a_road) - 1):
                    ## 同样的，aa为由于当前车辆负载所产生的额外等待时间
                    aa = ((v_load[(a_road[i])] - 1) / 2 + 1)
                    ## 传输时延+额外等待时间
                    desv += (224 + 0.0023629942501200486 + 30.845634559999993) / 1000 * aa
                    dis = math.sqrt(pow(node_list[i].position[0]-node_list[i+1].position[0],2)+pow(node_list[i].position[1]-node_list[i+1].position[1],2))
                    desv += ((dis / 1000 * 5 + dis / 1000 * 5 * 2 * 0.4169999999999999) / 1000) * aa
                    desv += 0.16 * aa

            ## 计算控制器件中继时延
            dec = 0
            for i in range(len(route)-1):
                ## 根据控制器负载计算额外等待时间
                aa = ((c_load[(route[i])] - 1) / 2 + 1)

                ## 如果为路径头，需要加上额外等待时间
                if i == 0:
                    dec += 20 * (aa-1)
                ## 如果不为路径头，需要加上额外的路径读取时间
                else:
                    dec += 7 * (aa-1)

                ## 传输时延+额外等待时间
                dec += (224 + 0.0023629942501200486 + 30.845634559999993) / 1000 * aa
                dis = D[i][i+1]
                dec += (dis / 1000 * 5 + dis / 1000 * 5 * 2 * 0.4169999999999999) / 1000 * aa
                dec += 0.16 * aa

            ## 对控制器-目的路径的时延计算，与源-控制器大致相同
            ## 唯一不同是，这里是计算从控制器-路径上第一辆车的时延
            aa = ((v_load[(b_road[0])] - 1) / 2 + 1)
            dedv = (224 + 0.0023629942501200486 + 29.799999999999997) / 1000 * aa
            dedv += ((controller.v_c_d[(b_road[0])] / 1000 * 5 + (
            controller.v_c_d[(b_road[0])]) / 1000 * 5 * 2 * 0.4169999999999999) / 1000) * aa
            dedv += 0.16 * aa

            if len(b_road) != 1:
                for i in range(len(b_road) - 1):
                    aa = ((v_load[(b_road[i])] - 1) / 2 + 1)
                    dedv += (224 + 0.0023629942501200486 + 30.845634559999993) / 1000 * aa
                    dis = math.sqrt(pow(node_list[i].position[0] - node_list[i + 1].position[0], 2) + pow(
                        node_list[i].position[1] - node_list[i + 1].position[1], 2))
                    dedv += ((dis / 1000 * 5 + dis / 1000 * 5 * 2 * 0.4169999999999999) / 1000) * aa
                    dedv += 0.16 * aa

            ## 三段时延相加，再上发送时延，和处理时间，该包时延计算完成(*´ω`*)(*´ω`*)(*´ω`*) ​​​​
            delay = dec + dedv + desv
            hop = total_hop[j]
            delay += hop * 0.16
            delay += 20
            delay_sum += delay
            Gp.record.append(delay)
            print(delay)
    sshop.append(shop)
    ssshop += shop
    shop = 0
    std2 += np.std(Gp.record, ddof=1)
    Gp.record.clear()
ssshop = ssshop + Gp.shop
print(cost/sim_time)

cost += 0.5973 * shop
print('\ncalculation time:\n')
print(effi/sim_time*1000/Gp.pps/round)
print('\ndelay:\n')
print(delay_sum/sum)
print('\njitter:\n')
print(std2/Gp.pps/round)
print('\ndelivery ratio:\n')
print(sum/sim_time/round/Gp.pps)
print('\naverage hops:\n')
print(ssshop/sum)
print("\n所有控制器数\n")
print(toc)
print("\n开启控制器数\n")
print(onc)
print("\ncost(w)\n")
print(cost/sim_time/round)
print(minh)
print(maxh)
print(sshop)