import numpy as np
import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.special import comb
from itertools import combinations
import copy
import Global_Par as gp
## 生成参考点
def uniformpoint(N,M):
    H1=1
    while (comb(H1+M-1,M-1)<=N):
        H1=H1+1
    H1=H1-1
    W=np.array(list(combinations(range(H1+M-1),M-1)))-np.tile(np.array(list(range(M-1))),(int(comb(H1+M-1,M-1)),1))
    W=(np.hstack((W,H1+np.zeros((W.shape[0],1))))-np.hstack((np.zeros((W.shape[0],1)),W)))/H1
    if H1<M:
        H2=0
        while(comb(H1+M-1,M-1)+comb(H2+M-1,M-1) <= N):
            H2=H2+1
        H2=H2-1
        if H2>0:
            W2=np.array(list(combinations(range(H2+M-1),M-1)))-np.tile(np.array(list(range(M-1))),(int(comb(H2+M-1,M-1)),1))
            W2=(np.hstack((W2,H2+np.zeros((W2.shape[0],1))))-np.hstack((np.zeros((W2.shape[0],1)),W2)))/H2
            W2=W2/2+1/(2*M)
            W=np.vstack((W,W2))#按列合并
    W[W<1e-6]=1e-6
    N=W.shape[0]
    return W,N

def yyec(x0,x1,y0,y1,x):
    return ((y1-y0)/(x1-x0))*x + (y1-(y1-y0)/(x1-x0)*x0)

def calculateFitness(f):
    if (f >= 0):
        fit = 1.0/(f+1)
    else:
        fit = 1.0 + abs(f + 1)
    return fit

##(*^‧^*) 第一个目标函数  能量 参数是车辆位置，需确定的控制器位置，固定开启的控制器位置，开关情况，所有开启控制器的统治车辆数量，车辆到最近控制器距离
def function1 (vehicle_position, controller_position,fix_controller_position, food,on_num,v_c_d):
    ## ｀（*^﹏^*）′发送能量 单位为 mJ
    esend = 440 * 5 * gp.ps / 6000000 * gp.pps * len(vehicle_position)
    ## ｀（*^﹏^*）′接受能量 单位为 mJ
    erecv = 260 * 5 * gp.ps / 6000000 * gp.pps * len(vehicle_position)

    ## ｀（*^﹏^*）′如果超过最大传输距离，需要中继，加一跳能量计算
    for i in v_c_d:
        if i > gp.com_dis:
            esend += 440 * 5 * gp.ps / 6000000 * gp.pps
            erecv += 260 * 5 * gp.ps / 6000000 * gp.pps

    eon = 0
    ## ｀（*^﹏^*）′控制器开启能耗，假设离自己最近的车均发送数据包
    for i in on_num:
        u = i / gp.cml
        eon += 0.05 + 9.95*(2*u - u*u)

    return esend/1000 + eon + erecv/1000

    sum = 0
    a = [2, -1, 2, -1, -3, -6, 2, 6, -5, 7, 3, 1, -8]
    for i in range(len(a)):
        sum += (food[i] * a[i])
    return sum+24

##｀（*^﹏^*）′ 第二个目标函数  负载 参数是车辆位置，需确定的控制器位置，固定开启的控制器位置，开关情况，所有开启控制器位置，所有开启控制器的统治车辆数量，车辆到最近控制器索引，车辆到最近控制器距离
def function2 (vehicle_position, controller_position,fix_controller_position, food,on_controller,on_num,v_c,v_c_d):

    ## (≥◇≤)平均功率
    lavg = gp.pps * len(vehicle_position) / len(on_controller)
    ## ( # ^.^ # ) 控制器度数计算
    w = [0 for i in range(len(on_controller))]
    ## p( ^ O ^ )q归一化
    maxd = max(on_num)
    for i in range(len(on_num)):
        w[i] = 1+(on_num[i]/maxd)
    ## (≥◇≤)计算各控制器的度数负载和总负载
    d_c = [0 for i in range(len(on_controller))]
    d_c_sum = 0
    for i in range(len(v_c)):
        d_c[v_c[i]] +=v_c_d[i]*w[v_c[i]]
        d_c_sum += v_c_d[i]*w[v_c[i]]
    ## p( ^ O ^ )q计算归一化后的度数负载方差
    summ = 0
    for i in d_c:
        summ += pow((i/d_c_sum-1/len(on_controller)),2)
    ldgr = math.sqrt(summ)
    ## (≥◇≤)计算各控制器的流负载和总流负载
    f_c = [0 for i in range(len(on_controller))]
    f_c_sum = 0
    for i in range(len(v_c)):
        f_c[v_c[i]] +=v_c_d[i]*gp.pps
        f_c_sum += v_c_d[i]*gp.pps
    ## ((o(^_ ^)o))计算归一化后的流负载方差
    summ = 0
    for i in f_c:
        summ += pow((i/f_c_sum-1/len(on_controller)),2)
    lflw = math.sqrt(summ)
    ## (≥◇≤)三者相加
    return lavg + ldgr + lflw

    sum = 0
    # a = [2, -1, 2, -1, -3, -6, 2, 6, -5, 7, 3, 1, -8]
    a = [3, 6, -2, 2, 3, -9, -8, 1, 4, -5, 1, 4, 0]
    for i in range(len(a)):
        sum += (food[i] * a[i])
    return sum+24

## (≥◇≤)csmaca时延
def tcontca(tprop):
    sum = 0
    for k in range(1,gp.c):
        sum += k*pow(gp.p,k)*(1-gp.p)
    sum += gp.c*pow(gp.p,gp.c)
    return 3/4*gp.aifs + gp.cwmin/8*((2-2*gp.p-pow(gp.p*2,gp.c))/(1-2*gp.p))*gp.tslot + sum * 2*tprop
## (≥◇≤)csmacd时延
def tcontcd(tprop):
    c = math.log(gp.cwmax/gp.cwmin,2)+1
    backoff = gp.cwmin/4*(2-2*gp.p-pow(2*gp.p,c))/(1-2*gp.p)
    sum = 0
    for k in range(1,gp.c):
        sum += k*pow(gp.p,k)*(1-gp.p)
    sum += gp.c*pow(gp.p,gp.c)
    return backoff * gp.tslot + sum * (2*tprop-gp.ifg)

## p( ^ O ^ )q迪杰斯特拉
def dijkstra(graph,src):
    length = len(graph)
    type_ = type(graph)
    if type_ == list:
        nodes = [i for i in range(length)]
    elif type_ == dict:
        nodes = graph.keys()

    visited = [src]
    path = {src:{src:[]}}
    nodes.remove(src)
    distance_graph = {src:0}
    pre = next = src

    while nodes:
        distance = float('inf')
        for v in visited:
             for d in nodes:
                new_dist = graph[src][v] + graph[v][d]
                if new_dist <= distance:
                    distance = new_dist
                    next = d
                    pre = v
                    graph[src][d] = new_dist


        path[src][next] = [i for i in path[src][pre]]
        path[src][next].append(next)

        distance_graph[next] = distance

        visited.append(next)
        nodes.remove(next)

    return distance_graph, path


## p( ^ O ^ )q第三个目标函数 时延
def function3 (vehicle_position, controller_position ,fix_controller_position,food,on_controller,on_num,v_c,v_c_d):
    ## p( ^ O ^ )q传输时延
    d_t = 0
    ## ((o(^_ ^)o))扩散时延
    d_p = 0
    ##( # ^.^ # )  排队时延
    d_q = 0
    ## (≥◇≤)竞争时延
    d_c = 0
    ttrans = 224
    tprop = 5
    tqueue = gp.ar/gp.rr*(1-pow(gp.ar/gp.rr,gp.N)-gp.N*pow(gp.ar/gp.rr,gp.N-1)*(1-gp.ar/gp.rr))/gp.ar*(1-gp.ar/gp.rr)*(1-pow(gp.ar/gp.rr,gp.N+1))
    ## p( ^ O ^ )q统计车辆向控制器发生时的四种时延
    for i in range(len(v_c)):
        if v_c_d[i] <= gp.com_dis:
            d_t += ttrans
            d_p += v_c_d[i]/1000 * tprop
            d_q += tqueue
            d_c += tcontca(v_c_d[i]/1000 * tprop)
        ## p( ^ O ^ )q如大于最大传输距离，以两条计
        else:
            d_t += ttrans*2
            d_p += v_c_d[i]*1.3 / 1000 * tprop
            d_q += tqueue*2
            d_c += tcontca(v_c_d[i]*1.3 / 1000 * tprop)


    delay_v = (d_q+d_p+d_t+d_c)/len(v_c)

    ## (≥◇≤)接下来计算控制器间传输时延
    c = math.log(gp.cwmax / gp.cwmin, 2) + 1
    backoff = gp.cwmin / 4 * (2 - 2 * gp.p - pow(2 * gp.p, c)) / (1 - 2 * gp.p)
    summm = 0
    for k in range(1, gp.c):
        summm += k * pow(gp.p, k) * (1 - gp.p)
    summm += gp.c * pow(gp.p, gp.c)
    ## (≥◇≤)统计控制器间邻接矩阵
    adj_martix = [[0 for i in range(len(on_controller))] for i in range(len(on_controller))]
    for i in range(len(on_controller)):
        for j in range(len(on_controller)):
            if i <= j:
                d = math.sqrt(pow(on_controller[i][0] - on_controller[j][0], 2) + \
                    pow(on_controller[i][1] - on_controller[j][1], 2))
                if d <= gp.com_dis*2:
                    adj_martix[i][j] = d
                    adj_martix[j][i] = d
                else:
                    adj_martix[i][j] = float('inf')
                    adj_martix[j][i] = float('inf')
    delay_c = 0
    a = np.array(on_controller)
    fig = plt.figure(figsize=(8, 7.82), dpi=80)
    plt.scatter(a[:, 0], a[:, 1], alpha=0.5, s=gp.con_dis * gp.con_dis * 0.025, edgecolors='blue')
    plt.scatter(a[:, 0], a[:, 1], alpha=1, s=1, edgecolors='black')
    plt.show()
    ##p( ^ O ^ )q 用迪杰斯特拉计算从每个控制器处到其他所有控制器的距离与跳数
    for i in range(len(on_controller)):
        d_t = 0
        d_p = 0
        d_q = 0
        d_c = 0
        ##p( ^ O ^ )q 归一化后的权重
        ww = on_num[i]/len(vehicle_position)
        ## ( # ^.^ # ) 迪杰斯特拉，获得到其他各控制器最短距离与跳数
        distance,path = dijkstra(adj_martix,i)
        for j in range(len(on_controller)):
            ##p( ^ O ^ )q 根据上述信息，更新四项时延
            hop = len(path[i][j])
            d_t += ttrans * hop
            d_q += tqueue * hop
            d_c += backoff * gp.tslot +summm*gp.ifg
            d_p += distance[j]/1000 * tprop
            d_c += summm*2*distance[j]/1000 * tprop
        ## p( ^ O ^ )q每个控制器的总时延，与自身统治车辆数有关，乘以该权重
        delay_c += (d_q+d_p+d_t+d_c) * ww
    ##(≥◇≤) 除以总控制器数
    delay_c /= len(on_controller)

    return delay_c + delay_v

    sum = 0
    #a = [2, -1, 2, -1, -3, -6, 2, 6, -5, 7, 3, 1, -8]
    a = [-3, 4, 7, -7, 2, 2, 1, -7, 5, 0, 0, -7, 3]
    for i in range(len(a)):
        sum += (food[i] * a[i])
    return sum+24

## 单目标CBA
def controller_place(position_list, position_list_fixed, Foods, function0):
    ## 选择开启控制器 群智能算法
    size = len(Foods)
    maxiter = 2000
    dim = len(Foods[0])
    times = 0
    numm = 0
    limit = size * dim*100000
    ObjVal = [0 for i in range(size)]
    Fitness = [0 for i in range(size)]
    trial = [0 for i in range(size)]
    prob = [0 for i in range(size)]
    memorize = [0 for i in range(maxiter)]
    sol = [0 for i in range(dim)]
    cand = [0 for i in range(dim)]
    for i in range(size):
        # 计算各蜜源目标函数
        ObjVal[i] = function0(Foods[i])
        Fitness[i] = calculateFitness(ObjVal[i])
        print(1)
    max_list = max(ObjVal)
    max_index = ObjVal.index(min(ObjVal))
    GlobalMin = ObjVal[max_index]
    GlobalParams = Foods[max_index]

    for i in range(maxiter):
        index = sorted(range(len(ObjVal)), key=lambda k: ObjVal[k])
        ia = i - maxiter / 10 / 1.5 * numm
        for j in range(size):
            for iii in range(len(Foods[index[j]])):
                sol[iii] = Foods[index[j]][iii]
            Param2Change = random.randint(0, dim-1)
            if j <= size * yyec(0, maxiter, 1, 0.5, ia):
                neigh = random.randint(0, size-1)
                while neigh == index[j]:
                    neigh = random.randint(0, size - 1)
                if sol[Param2Change] == 0:
                    sol[Param2Change] = 1
                else:
                    sol[Param2Change] = 0
            else:
                sol[Param2Change] = GlobalParams[Param2Change]

            ObjValSol = function0(sol)# 评价函数
            FitnessSol = calculateFitness(ObjValSol)# 适应度值计算calculateFitness(ObjValSol);

            if ObjValSol < ObjVal[index[j]]:
                print(sol)
                for iii in range(len(sol)):
                    Foods[index[j]][iii] = sol[iii]
                Fitness[index[j]] = FitnessSol
                ObjVal[index[j]] = ObjValSol
                trial[index[j]] = 0
            else:
                trial[index[j]] += 1

        max_index = ObjVal.index(min(ObjVal))
        GlobalMin = ObjVal[max_index]
        GlobalParams = Foods[max_index]

        for f in range(size):
            prob[f] = (0.9 * Fitness[f]/max(Fitness)) + 0.1

        k = 1
        t = 0
        while(t < size):
            if (random.random() < prob[k]):
                t += 1
                Param2Change = random.randint(0, dim - 1)
                neigh = random.randint(0, size - 1)
                for iii in range(len(Foods[k])):
                    sol[iii] = Foods[k][iii]
                if sol[Param2Change] == 0:
                    sol[Param2Change] = 1
                else:
                    sol[Param2Change] = 0

                ObjValSol = function0(sol)  # 评价函数
                FitnessSol = calculateFitness(ObjValSol)  # 适应度值计算calculateFitness(ObjValSol);

                if ObjValSol < ObjVal[k]:
                    print(sol)
                    for iii in range(len(sol)):
                        Foods[k][iii] = sol[iii]
                    Fitness[k] = FitnessSol
                    ObjVal[k] = ObjValSol
                    trial[k] = 0
                else:
                    trial[k] += 1
            k = k+1
            if k == size:
                k = 0

        aaa = GlobalMin
        max_index = ObjVal.index(min(ObjVal))
        GlobalMin = ObjVal[max_index]
        GlobalParams = Foods[max_index]
        if aaa == GlobalMin:
            times += 1

        max_index = ObjVal.index(min(ObjVal))
        GlobalMin = ObjVal[max_index]
        GlobalParams = Foods[max_index]
        Param2Change1 = []
        pop_size = 10
        if (times >= maxiter/10) or (i % 1000 == 0):

            for iii in range(len(Foods[max_index])):
                sol[iii] = Foods[max_index][iii]
            can = [sol for ii in range(pop_size + 1)]
            for l in range(pop_size):
                Param2Change1.append(np.random.randint(0, dim))
            for ll in range(pop_size):
                Param2Change = Param2Change1[ll]
                for iii in range(len(sol)):
                    cand[iii] = sol[iii]
                if can[ll][Param2Change] == 0:
                    can[ll][Param2Change] = 1
                else:
                    can[ll][Param2Change] = 0
            minx = 9999999
            for l in range(pop_size):
                re = function0(can[l]) # 目标函数
                if re < minx:
                    minx = re
                    mini = l
            if minx < min(ObjVal):
                if mini == pop_size:
                    trial[max_index] += 1
                print(can[mini])
                for iii in range(len(can[mini])):
                    Foods[max_index][iii] = can[mini][iii]
                Fitness[max_index] = calculateFitness(minx) # 适应度值计算
                ObjVal[max_index] = minx
                trial[max_index] = 0
            times = 0
            numm += 1

        max_index = ObjVal.index(min(ObjVal))
        GlobalMin = ObjVal[max_index]
        GlobalParams = Foods[max_index]

        # max_index = trial.index(max(trial))
        # mm = ObjVal.index(min(ObjVal))
        # if max_index != mm:
        #     if trial[max_index] > limit:
        #         sol = Foods[max_index]
        #         for ll in range(dim):
        #             sol[ll] = random.randint(0, 1)
        #         ObjValSol = function0(sol) #目标函数
        #         FitnessSol = calculateFitness(ObjValSol)# 适应度值计算
        #         Foods[max_index] = sol
        #         Fitness[max_index] = FitnessSol
        #         ObjVal[max_index] = ObjValSol

        memorize[i] = GlobalMin
    max_index = ObjVal.index(min(ObjVal))
    GlobalMin = ObjVal[max_index]
    GlobalParams = Foods[max_index]

    print(GlobalMin)
    print(GlobalParams)
    print(function0(GlobalParams))
    return memorize, GlobalParams, Foods

## 比较两个解是否支配 参数为两个三值的列表，为三个适应度值。
def compare(listed, list):
    n = 0
    for i in range(len(listed)):
        if (listed[i] < list[i]):
            return 0## 第二个解无法支配第一个解
        if listed[i] == list[i]:
            n+=1
    if n == len(list):
        return 0
    else:
        return 1 ## 第二个解支配第一个解

## 非支配排序
def Nd_sort(list):
    re = [0 for i in range(len(list))]
    a = [0 for i in range(len(list))]
    b = [[] for i in range(len(list))]
    for i in range(len(list)):
        for j in range(len(list)):
            if i!=j:
                if compare(list[i],list[j]) == 1:
                    a[i] +=1
                    b[j].append(i)

    maxfront = 6
    for i in range(1,maxfront):
        d = []
        for j in range(len(a)):
            if a[j] == 0 and re[j] == 0:
                re[j] = i
                for s in b[j]:
                    d.append(s)
        for ss in d:
            a[ss]-=1

    for i in re:
        if i == 0:
            i=maxfront
    return re

## 统计所有解中优解与劣解，参数为3*N的列表，为所有解的三个适应度值
def Pareto_Optimality(list):
    ## 优解列表
    optimal_list = []
    ## 劣解列表
    no_list = []
    b = 0
    for i in range(len(list)):
        for j in range(len(list)):
            if i!=j:
                ## 如被支配 失去测试资格 跳出循环
                if b == 1:
                    break
        ## i被支配 为劣解
        if b == 1:
            no_list.append(i)
        ## i未被任何解支配 优解
        else:
            optimal_list.append(i)
        b = 0
    ## 返回值为 优解与劣解 在源列表中的索引
    return optimal_list, no_list

## 筛选部分 统计帕累托前沿，并维护种群数量 多删少补， 参数为适应度列表，期望种群规模，所有解，维度
def Pareto_Optimality_1(list, size, food, dim):
    optimal_list = []
    re_op_list = []
    re_no_list = []
    re_list = []
    no_list = []
    b = 0
    ## 所有解后两位初始化，dim位存支配解数量，dim+1位存密度
    for i in food:
        i[dim] = 6
        i[dim+1] = 0
    ## 非支配排序计算
    front = Nd_sort(list)

    for i in range(front.__len__()):
        food[i][dim] = front[i]

    ## 解密度计算
    sol_density(food, list, dim)


    ## 根据参考点筛选
    npfront = np.array(front)
    max = np.sort(npfront)
    ind = np.argsort(npfront)
    line = max[size-1]
    len = 0
    for i in range(max.__len__()):
        if max[i] < line:
            optimal_list.append(ind[i])
            re_op_list.append(list[ind[i]])
            re_list.append(list[ind[i]])
            len += 1
        else:
            if max[i] == line:
                no_list.append(ind[i])
                re_no_list.append(list[ind[i]])
                re_list.append(list[ind[i]])
    Z,aaa = uniformpoint(size, list[1].__len__())
    len = size - len
    choose = lastselection(np.array(re_op_list),np.array(re_no_list),len,Z,re_list)
    for i in range(choose.__len__()):
        if choose[i]:
            optimal_list.append(no_list[i])
    return optimal_list

def pdist(x,y):
    x0=x.shape[0]
    y0=y.shape[0]
    xmy=np.dot(x,y.T)#x乘以y
    xm=np.array(np.sqrt(np.sum(x**2,1))).reshape(x0,1)
    ym=np.array(np.sqrt(np.sum(y**2,1))).reshape(1,y0)
    xmmym=np.dot(xm,ym)
    cos = xmy/xmmym
    return cos

## 根据参考点筛选过程
def lastselection(popfun1, popfun2, K, Z, re):
    z = []
    re = np.array(re)
    for i in range(len(re[1,:])):
        list = re[:,i]
        a = np.min(list)
        z.append(a)
    ZZ = []
    ZZ.append(z)
    Zmin = np.array(ZZ)
    # 选择最后一个front的解
    if popfun1.__len__() != 0 and popfun2.__len__() != 0:
        popfun = copy.deepcopy(np.vstack((popfun1, popfun2))) - np.tile(Zmin, (popfun1.shape[0] + popfun2.shape[0], 1))
    if popfun1.__len__() == 0:
        popfun = popfun2
    if popfun2.__len__() == 0:
        popfun = popfun1
    N, M = popfun.shape[0], popfun.shape[1]
    N1 = popfun1.shape[0]
    N2 = popfun2.shape[0]
    NZ = Z.shape[0]

    # 正则化
    extreme = np.zeros(M)
    w = np.zeros((M, M)) + 1e-6 + np.eye(M)
    for i in range(M):
        extreme[i] = np.argmin(np.max(popfun / (np.tile(w[i, :], (N, 1))), 1))

    # 计算截距
    extreme = extreme.astype(int)  # python中数据类型转换一定要用astype
    # temp = np.mat(popfun[extreme,:]).I
    temp = np.linalg.pinv(np.mat(popfun[extreme, :]))
    hyprtplane = np.array(np.dot(temp, np.ones((M, 1))))
    a = 1 / hyprtplane
    if np.sum(a == math.nan) != 0:
        a = np.max(popfun, 0)
    np.array(a).reshape(M, 1)  # 一维数组转二维数组
    # a = a.T - Zmin
    a = a.T
    popfun = popfun / (np.tile(a, (N, 1)))

    ##联系每一个解和对应向量
    # 计算每一个解最近的参考线的距离
    cos = pdist(popfun, Z)
    distance = np.tile(np.array(np.sqrt(np.sum(popfun ** 2, 1))).reshape(N, 1), (1, NZ)) * np.sqrt(1 - cos ** 2)
    # 联系每一个解和对应的向量
    d = np.min(distance.T, 0)
    pi = np.argmin(distance.T, 0)

    # 计算z关联的个数
    rho = np.zeros(NZ)
    for i in range(NZ):
        rho[i] = np.sum(pi[:N1] == i)

    # 选出剩余的K个
    choose = np.zeros(N2)
    choose = choose.astype(bool)
    zchoose = np.ones(NZ)
    zchoose = zchoose.astype(bool)
    while np.sum(choose) < K:
        # 选择最不拥挤的参考点
        temp = np.ravel(np.array(np.where(zchoose == True)))
        jmin = np.ravel(np.array(np.where(rho[temp] == np.min(rho[temp]))))
        j = temp[jmin[np.random.randint(jmin.shape[0])]]
        #        I = np.ravel(np.array(np.where(choose == False)))
        #        I = np.ravel(np.array(np.where(pi[(I+N1)] == j)))
        I = np.ravel(np.array(np.where(pi[N1:] == j)))
        I = I[choose[I] == False]
        if (I.shape[0] != 0):
            if (rho[j] == 0):
                s = np.argmin(d[N1 + I])
            else:
                s = np.random.randint(I.shape[0])
            choose[I[s]] = True
            rho[j] = rho[j] + 1
        else:
            zchoose[j] = False
    return choose




## 统计帕累托前言，与支配解数量  参数为种群信息，目标结果列表， 维度， 优解列表， 劣解列表， 优解索引列表， 劣解索引列表
## 返回 优解列表大小， 并将优解 劣解区分完成 储存在各自列表中
def pareto_front(food, list, dim):
    for i in food:
        i[dim] = 6
        i[dim + 1] = 0

    front = Nd_sort(list)

    for i in range(len(front)):
        food[i][dim] = front[i]
    sol_density(food, list, dim)

## 计算中群内当前解密度， 参数为种群信息，优解列表大小， 种群大小， 维度
## 无返回值， 所有解密度计算完成后，储存在各解dim+1索引处
def sol_density(Foods,relist, dim):
    re = np.array(relist)
    for i in range(len(re[1,:])):
        d = re[:,i]
        max = np.sort(d)
        ind = np.argsort(d)
    for j in range(len(ind)):
        if j == 0:
            Foods[ind[j]][dim+1] = Foods[ind[j]][dim+1]+2*(max[j+1]-max[j])/(max[len(ind)-1]-max[0])
        else:
            if j == len(ind)-1:
                Foods[ind[j]][dim+1] = Foods[ind[j]][dim+1] + 2 * (max[j] - max[j-1]) / (
                            max[len(ind)-1] - max[0])
            else:
                Foods[ind[j]][dim + 1] = Foods[ind[j]][dim+1] + (max[j+1] - max[j - 1]) / (
                        max[len(ind)-1] - max[0])


## 将新求出解与原解链接，并做帕累托判断，并保持种群大小 参数为新解+原解种群信息，目标结果列表，维度，原解大小（目标解大小）
## 返回处理后的新种群，在连接解中寻优，并与原种群保持同样大小
def new_old_sol_compare(Foods, re_list,dim, size,vehicle_position, controller_position,fix_controller_position):
    for num in range(len(Foods)):
        food = Foods[num]
        on_controller = []
        for i in range(len(controller_position)):
            if food[i] == 1:
                on_controller.append(controller_position[i])
        for i in fix_controller_position:
            on_controller.append(i)

        lavg = gp.pps * len(vehicle_position) / len(on_controller)

        on_num = [0 for i in range(len(on_controller))]
        v_c = [0 for i in range(len(vehicle_position))]
        v_c_d = [0 for i in range(len(vehicle_position))]
        for i in range(len(vehicle_position)):
            mind = 999999
            for j in range(len(on_controller)):
                d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0], 2) + \
                              pow(vehicle_position[i][1] - on_controller[j][1], 2))
                if d < mind:
                    mind = d
                    minj = j
            v_c[i] = minj
            v_c_d[i] = mind
            on_num[minj] += 1
        re_list[num][0] = function1(vehicle_position, controller_position,fix_controller_position,Foods[num],on_num,v_c_d)
        re_list[num][1] = function2(vehicle_position, controller_position,fix_controller_position,Foods[num],on_controller,on_num,v_c,v_c_d)
        re_list[num][2] = function3(vehicle_position, controller_position,fix_controller_position,Foods[num],on_controller,on_num,v_c,v_c_d)
        Foods[num][dim] = 6
        Foods[num][dim + 1] = 0
    Food = Pareto_Optimality_1(re_list, size, Foods, dim)
    Foods1 = []
    for i in Food:
        Foods1.append(Foods[i])
    ## 当前Foods为筛选后的新解
    return Foods1

## 雇佣蜂操作 参数种群信息，种群大小，维度，优解索引列表， 劣解索引列表， 当前最优解
## 优解邻域搜索，劣解向最优解前进
## 无返回值 种群信息将原解与更新后的解连接，以供后面的判断
def employed_bees(Foods, size, dim, GlobalParams):
    for i in range(size):
        sol = [0 for i in range(dim + 2)]
        for iii in range(len(Foods[i])):
            sol[iii] = Foods[i][iii]
        ## 随机决定更改的维度
        Param2Change = random.randint(0, dim - 1)
        ## 劣解向最优解前进
        if (Foods[i][dim]!=1):
            sol[Param2Change] = GlobalParams[Param2Change]
        else:
            ## 前沿中的解邻域搜索
            if (Foods[i][dim]==1):
                if sol[Param2Change] == 0:
                    sol[Param2Change] = 1
                else:
                    sol[Param2Change] = 0
        ## 新解融入原解中
        Foods.append(sol)

## 跟随蜂操作 参数种群信息，概率列表，种群大小，维度
## 所有解轮盘赌，进行邻域搜索
## 无返回值 种群信息将原解与更新后的解连接，以供后面的判断
def onlooker_bee(Foods, prob, size, dim):
    ## 根据轮盘赌概率，进行邻域搜索
    i = 1
    t = 0
    while (t < size):
        if (random.random() < prob[i]):
            sol = [0 for i in range(dim + 2)]
            t += 1
            Param2Change = random.randint(0, dim - 1)
            for iii in range(len(Foods[i])):
                sol[iii] = Foods[i][iii]
            ## 选中的解 在随机维度上邻域搜索
            if sol[Param2Change] == 0:
                sol[Param2Change] = 1
            else:
                sol[Param2Change] = 0
            Foods.append(sol)

        i = i + 1
        if i == size:
            i = 0

## 侦察蜂操作 参数种群信息，维度，变异种群大小
## 对密度最高的优解，随机进行变异
## 无返回值 种群信息将原解与更新后的解连接，以供后面的判断
def scout_bee(Foods, dim, pop_size):
    max = 0
    max_index = -1
    ## 找到密度值最高的解
    for i in range(len(Foods)):
        if Foods[i][dim] == 1 and Foods[i][dim+1] > max:
            max = Foods[i][dim + 1]
            max_index = i

    for i in range(pop_size):
        ## 对选中的解在随机维度上变异10次
        sol = [0 for i in range(dim + 2)]
        for iii in range(len(Foods[max_index])):
            sol[iii] = Foods[max_index][iii]
        can = []
        can = sol
        Param2Change = np.random.randint(0, dim)
        if can[Param2Change] == 0:
            can[Param2Change] = 1
        else:
            can[Param2Change] = 0
        ## 所有生成的新解融入原解中
        Foods.append(can)
    times = 0

def cal(Foods, re_list,dim,vehicle_position, controller_position,fix_controller_position):
    for num in range(len(Foods)):
        food = Foods[num]
        on_controller = []
        for i in range(len(controller_position)):
            if food[i] == 1:
                on_controller.append(controller_position[i])
        for i in fix_controller_position:
            on_controller.append(i)
        on_num = [0 for i in range(len(on_controller))]
        v_c = [0 for i in range(len(vehicle_position))]
        v_c_d = [0 for i in range(len(vehicle_position))]
        for i in range(len(vehicle_position)):
            mind = 999999
            for j in range(len(on_controller)):
                d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0], 2) + \
                              pow(vehicle_position[i][1] - on_controller[j][1], 2))
                if d < mind:
                    mind = d
                    minj = j
            v_c[i] = minj
            v_c_d[i] = mind
            on_num[minj] += 1
        re_list[num][0] = function1(vehicle_position, controller_position,fix_controller_position,Foods[num],on_num,v_c_d)
        re_list[num][1] = function2(vehicle_position, controller_position,fix_controller_position,Foods[num],on_controller,on_num,v_c,v_c_d)
        re_list[num][2] = function3(vehicle_position, controller_position,fix_controller_position,Foods[num],on_controller,on_num,v_c,v_c_d)
        Foods[num][dim] = 6
        Foods[num][dim + 1] = 0

def multiobj(dim,vehicle_position, controller_position,fix_controller_position):
    ## 种群规模
    size1 = 30
    ## 最大迭代次数
    maxiter = 1000
    ## 稳定次数
    timess = 0
    ## 上一轮解
    pre_foods = []
    ## 随机生成初始解
    Foods = [[random.randint(0, 1) for i in range(dim+2)] for i in range(size1)]

    for times in range(maxiter):
        ## 统计上一轮解
        pre_foods = Foods
        ## 统计Foods的适应度值列表
        re_list = [[0 for i in range(3)] for i in range(size1)]
        ## 优解列表
        optimal = []
        ## 劣解列表
        no = []
        optimal_list = []
        no_list = []

        cal(Foods,re_list,dim,vehicle_position, controller_position,fix_controller_position)
        ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
        pareto_front(Foods, re_list, dim)
        ## 统计解密度
        sol_density(Foods, re_list, dim)

        min = 99999
        mini = 0
        ## 选择帕累托前沿中密度最小的解为最优解
        for i in range(len(Foods)):
            if Foods[i][dim+1] < min and Foods[i][dim] == 1:
                min = Foods[i][dim+1]
                mini = i
        GlobalParams = [0 for i in range(dim+2)]
        GlobalParams = Foods[mini]


        size = len(Foods)

############################################ 雇佣蜂 ###########################################################
        employed_bees(Foods, size, dim , GlobalParams)

        re_list = [[0 for i in range(3)] for i in range(len(Foods))]

        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 新产生的解与原解 全部放入Foods进行筛选
        Foods = new_old_sol_compare(Foods, re_list, dim, size,vehicle_position, controller_position,fix_controller_position)

        re_list = [[0 for i in range(3)] for i in range(size1)]
        ## 优解列表
        optimal = []
        ## 劣解列表
        no = []
        optimal_list = []
        no_list = []
        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
        pareto_front(Foods, re_list, dim)

        ## 统计解密度
        sol_density(Foods, re_list, dim)

########################################## 跟随蜂 ###############################################################
        prob = [0 for i in range(size)]
        f = np.array(Foods)
        d = f[:,dim+1]
        max = np.sort(d)
        ind = np.argsort(d)
        leng = len(max)
        for i in range(leng):
            prob[ind[i]] = (max[i]-max[0])/(max[leng-1]-max[0])
            if f[ind[i],dim] >= 3:
                aa = 0
            else:
                aa = 1
            prob[ind[i]] = prob[ind[i]] * aa


        ## 根据轮盘赌概率，进行邻域搜索
        onlooker_bee(Foods, prob, size, dim)


        re_list = [[0 for i in range(3)] for i in range(len(Foods))]
        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 新产生的解与原解 全部放入Foods进行筛选
        Foods = new_old_sol_compare(Foods, re_list, dim, size,vehicle_position, controller_position,fix_controller_position)

        re_list = [[0 for i in range(3)] for i in range(size1)]
        ## 优解列表
        optimal = []
        ## 劣解列表
        no = []
        optimal_list = []
        no_list = []
        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
        pareto_front(Foods, re_list, dim)

        ## 统计解密度
        sol_density(Foods, re_list, dim)

########################################## 侦察蜂 ###############################################################
        ## 若种群保持不变，稳定次数加1
        if Foods == pre_foods:
            timess += 1
        ## 若种群稳定次数超过阈值 ，进行变异
        pop_size = 10
        if (timess >= maxiter / 4) or (times % 1000 == 0):
            scout_bee(Foods, dim, pop_size)
            timess = 0


        re_list = [[0 for i in range(3)] for i in range(len(Foods))]
        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 新产生的解与原解 全部放入Foods进行筛选
        Foods = new_old_sol_compare(Foods, re_list, dim, size,vehicle_position, controller_position,fix_controller_position)

        ## 重新统计适应度值 支配解数量 解密度

        re_list = [[0 for i in range(3)] for i in range(size1)]
        ## 优解列表
        optimal = []
        ## 劣解列表
        no = []
        optimal_list = []
        no_list = []
        cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
        pareto_front(Foods, re_list, dim)

        ## 统计解密度
        sol_density(Foods, re_list, dim)

        print(times)
        ############ 侦察蜂发生在后期。到后期帕雷托前沿中每个解都有重要的参考价值，将其随机变异成未知解 反而会扰乱帕累托前沿。############
        # max_index = trial.index(max(trial))
        # mm = ObjVal.index(min(ObjVal))
        # if max_index != mm:
        #     if trial[max_index] > limit:
        #         sol = Foods[max_index]
        #         for ll in range(dim):
        #             sol[ll] = random.randint(0, 1)
        #         ObjValSol = function0(sol) #目标函数
        #         FitnessSol = calculateFitness(ObjValSol)# 适应度值计算
        #         Foods[max_index] = sol
        #         Fitness[max_index] = FitnessSol
        #         ObjVal[max_index] = ObjValSol

        if times%100 == 0:
            for num in range(len(Foods)):
                food = Foods[num]
                on_controller = []
                for i in range(len(controller_position)):
                    if food[i] == 1:
                        on_controller.append(controller_position[i])
                for i in fix_controller_position:
                    on_controller.append(i)
                on_num = [0 for i in range(len(on_controller))]
                v_c = [0 for i in range(len(vehicle_position))]
                v_c_d = [0 for i in range(len(vehicle_position))]
                for i in range(len(vehicle_position)):
                    mind = 999999
                    for j in range(len(on_controller)):
                        d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0], 2) + \
                                      pow(vehicle_position[i][1] - on_controller[j][1], 2))
                        if d < mind:
                            mind = d
                            minj = j
                    v_c[i] = minj
                    v_c_d[i] = mind
                    on_num[minj] += 1
                re_list[num][0] = function1(vehicle_position, controller_position, fix_controller_position, Foods[num],
                                            on_num, v_c_d)
                re_list[num][1] = function2(vehicle_position, controller_position, fix_controller_position, Foods[num],
                                            on_controller, on_num, v_c, v_c_d)
                re_list[num][2] = function3(vehicle_position, controller_position, fix_controller_position, Foods[num],
                                            on_controller, on_num, v_c, v_c_d)



            data = np.array(re_list)

            x = data[:, 0]  # [ 0  3  6  9 12 15 18 21]
            y = data[:, 1]  # [ 1  4  7 10 13 16 19 22]
            z = data[:, 2]  # [ 2  5  8 11 14 17 20 23]

            # 绘制散点图
            fig = plt.figure()
            ax = Axes3D(fig)
            ax.scatter(x, y, z)

            # 添加坐标轴(顺序是Z, Y, X)
            ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
            ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
            ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
            plt.show()
            print(re_list)
    ## 返回所有帕累托前沿解，按需选择适合的解
    return Foods
##((o(^_ ^)o)) 未确定控制器的位置
controller = np.loadtxt('controller.txt')
##((o(^_ ^)o)) 固定控制器的位置
fixcontroller = np.loadtxt('fixcontroller.txt')
## ((o(^_ ^)o))车辆位置
vehicle = np.loadtxt('vehicle.txt')
controller = controller[:,[0,1]]
fixcontroller = fixcontroller[:,[0,1]]
# multiobj(len(controller),vehicle,controller,fixcontroller)