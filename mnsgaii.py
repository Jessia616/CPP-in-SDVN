import numpy as np
import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.special import comb
from itertools import combinations
import copy
import Global_Par as gp
import time
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

################################################################
##(*^‧^*) 第一个目标函数  各车辆到控制器的距离 求最大距离和平均距离
def function1 (v_c_d):
    ## ｀（*^﹏^*）根据总跳数 计算发送能量与接受能量之和 单位为 mJ
    v_c_d = np.array(v_c_d)
    a = v_c_d.max()
    b = v_c_d.sum()/len(v_c_d)
    return a + b
################################################################

################################################################
##｀（*^﹏^*）′ 第二个目标函数 控制器间距离，同样最大值和平均值
def function2 (on_controller):
    X = np.array(on_controller)
    X = X.transpose()
    m, n = X.shape
    G = np.dot(X.T, X)
    # 把G对角线元素拎出来，列不变，行复制n遍。
    H = np.tile(np.diag(G), (n, 1))
    D = H + H.T - G * 2
    D = np.sqrt(D)

    e = (D<1000)

    a = e.max()
    b = e.sum()/len(e)

    return a + b
################################################################

################################################################
## p( ^ O ^ )q第三个目标函数 负载，控制器所负责车辆的最大值最小值之差
def function3 (on_controller,v_c):
    dis_c = [0 for i in range(len(on_controller))]
    for i in range(len(v_c)):
        dis_c[v_c[i]] += 1
    dis_c = np.array(dis_c)
    return dis_c.max() - dis_c.min()
################################################################

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
        ## 计算所有开启控制器的位置
        on_controller = []
        for i in range(len(controller_position)):
            if food[i] == 1:
                on_controller.append(controller_position[i])
        for i in fix_controller_position:
            on_controller.append(i)

        ## 计算每辆车离自己最近的控制器，v_c是最近控制器索引，v_c_d是最近控制器的离自己的距离，on_num是每个控制器所统治的车辆数量，vhop是车辆到最近控制器所需跳数
        on_num = [0 for i in range(len(on_controller))]
        xa = len(vehicle_position)
        xb = len(on_controller)

        # 联系每一个解和对应的向量，即n辆车到m个控制器的位置，a为车辆位置矩阵，b为控制器位置矩阵
        aq = np.array(vehicle_position)
        bq = np.array(on_controller)
        ## 计算asq
        a = aq ** 2
        a = a.sum(axis=1)
        aaa = a
        for i in range(xb-1):
            aaa = np.vstack([aaa,a])
        aaa = aaa.transpose()

        ## 计算bsq
        b = bq ** 2
        b = b.sum(axis=1)
        b = b.transpose()
        bbb = b
        for i in range(xa-1):
            bbb = np.vstack([bbb,b])
        ## 计算 a*(b^-1)
        c = aq.dot(bq.transpose())
        ## asq+bsq-2*a*(b^-1)开方即为n到m的距离矩阵
        ddd = np.sqrt(aaa+bbb-2*c)
        ## 在每行中挑出最小的，即为车辆的最距离控制器距离，索引通过argmin获得
        diii = np.min(ddd.T, 0)
        pii = np.argmin(ddd.T, 0)


        # for i in range(len(vehicle_position)):
        #     mind = 999999
        #     for j in range(len(on_controller)):
        #         d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0],2) + \
        #             pow(vehicle_position[i][1] - on_controller[j][1],2))
        #         if d < mind:
        #             mind = d
        #             minj = j
        #     v_c[i] = minj
        #     v_c_d[i] = mind
        #     on_num[minj] += 1
        for i in pii:
            on_num[i]+=1
        v_c = pii
        v_c_d = diii
        ## 最短距离整除最大通信距离，即可获得车辆到最近控制器所需跳数
        vhop = np.divide(v_c_d, gp.com_dis)
        vhop = vhop.astype(np.int)
        vhop += 1
        re_list[num][0] = function1(v_c_d)
        re_list[num][1] = function2(on_controller)
        re_list[num][2] = function3(on_controller,v_c)
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
    add = []
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
        add.append(sol)
    return add

## 跟随蜂操作 参数种群信息，概率列表，种群大小，维度
## 所有解轮盘赌，进行邻域搜索
## 无返回值 种群信息将原解与更新后的解连接，以供后面的判断
def onlooker_bee(Foods, prob, size, dim):
    add = []
    ## 根据轮盘赌概率，进行邻域搜索
    i = 1
    t = 0
    while (t < size):
        if (random.random() < prob[i]):
            jt = 0
            while jt == 0:
                for j in range(size):
                    if (random.random() < prob[j]) and j != i:
                        jt += 1
                        sta = random.randint(0, int(dim/2))
                        sol = [0 for i in range(dim + 2)]
                        for iii in range(len(Foods[i])):
                            sol[iii] = Foods[i][iii]

                        for iii in range(sta):
                            a = random.randint(0, dim - 1)
                            sol[a] = Foods[j][a]
                        break
            add.append(sol)
            t += 1
        i = i + 1
        if i == size:
            i = 0
    return add

## 侦察蜂操作 参数种群信息，维度，变异种群大小
## 对密度最高的优解，随机进行变异
## 无返回值 种群信息将原解与更新后的解连接，以供后面的判断
def scout_bee(Foods, dim, pop_size):
    add = []
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
        add.append(can)
    times = 0

    return add

def cal(Foods, re_list,dim,vehicle_position, controller_position,fix_controller_position):
    for num in range(len(Foods)):
        food = Foods[num]
        ## 计算所有开启控制器的位置
        on_controller = []
        for i in range(len(controller_position)):
            if food[i] == 1:
                on_controller.append(controller_position[i])
        for i in fix_controller_position:
            on_controller.append(i)

        ## 计算每辆车离自己最近的控制器，v_c是最近控制器索引，v_c_d是最近控制器的离自己的距离，on_num是每个控制器所统治的车辆数量，vhop是车辆到最近控制器所需跳数
        on_num = [0 for i in range(len(on_controller))]
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
        diii = np.min(ddd.T, 0)
        pii = np.argmin(ddd.T, 0)

        # for i in range(len(vehicle_position)):
        #     mind = 999999
        #     for j in range(len(on_controller)):
        #         d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0],2) + \
        #             pow(vehicle_position[i][1] - on_controller[j][1],2))
        #         if d < mind:
        #             mind = d
        #             minj = j
        #     v_c[i] = minj
        #     v_c_d[i] = mind
        #     on_num[minj] += 1
        for i in pii:
            on_num[i] += 1
        v_c = pii
        v_c_d = diii
        ## 最短距离整除最大通信距离，即可获得车辆到最近控制器所需跳数
        vhop = np.divide(v_c_d, gp.com_dis)
        vhop = vhop.astype(np.int)
        vhop += 1
        re_list[num][0] = function1(v_c_d)
        re_list[num][1] = function2(on_controller)
        re_list[num][2] = function3(on_controller, v_c)
        Foods[num][dim] = 6
        Foods[num][dim + 1] = 0
def cal_1(Foods,dim,vehicle_position, controller_position,fix_controller_position):
    food = Foods
    ## 计算所有开启控制器的位置
    on_controller = []
    for i in range(len(controller_position)):
        if food[i] == 1:
            on_controller.append(controller_position[i])
    for i in fix_controller_position:
        on_controller.append(i)

    ## 计算每辆车离自己最近的控制器，v_c是最近控制器索引，v_c_d是最近控制器的离自己的距离，on_num是每个控制器所统治的车辆数量，vhop是车辆到最近控制器所需跳数
    on_num = [0 for i in range(len(on_controller))]
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
    diii = np.min(ddd.T, 0)
    pii = np.argmin(ddd.T, 0)

    # for i in range(len(vehicle_position)):
    #     mind = 999999
    #     for j in range(len(on_controller)):
    #         d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0],2) + \
    #             pow(vehicle_position[i][1] - on_controller[j][1],2))
    #         if d < mind:
    #             mind = d
    #             minj = j
    #     v_c[i] = minj
    #     v_c_d[i] = mind
    #     on_num[minj] += 1
    for i in pii:
        on_num[i] += 1
    v_c = pii
    v_c_d = diii
    ## 最短距离整除最大通信距离，即可获得车辆到最近控制器所需跳数
    vhop = np.divide(v_c_d, gp.com_dis)
    vhop = vhop.astype(np.int)
    vhop += 1
    return function1(v_c_d) + function2(on_controller) + function3(on_controller, v_c)


def multiobj(dim,vehicle_position, controller_position,fix_controller_position):
    t = time.time()
    ## 种群规模
    size1 = 30
    ## 最大迭代次数
    maxiter = 10
    ## 稳定次数
    timess = 0
    ## 上一轮解
    pre_foods = []
    ## 随机生成初始解
    Foods = [[random.randint(0, 1) for i in range(dim+2)] for i in range(size1)]


    ########################## Greedy Start ###############################
    ## 从第一个位置开始用0，1试，如果三个目标函数之和更小 ，说明该位置用0或者1好，便固定下来，从头试到尾
    Food1 = [0 for i in range(dim+2)]
    Food2 = [0 for i in range(dim+2)]
    Food2[0] = 1
    for i in range(1,dim):
        sol1 = [0 for i in range(dim + 2)]
        sol2 = [0 for i in range(dim + 2)]
        for iii in range(len(Food1)):
            sol1[iii] = Food1[iii]
            sol2[iii] = Food1[iii]
        sol1[i] = 0
        sol2[i] = 1

        sum1 = cal_1(Food1, dim, vehicle_position, controller_position, fix_controller_position)
        sum2 = cal_1(Food2, dim, vehicle_position, controller_position, fix_controller_position)
        if sum1 < sum2:
            for iii in range(len(Food1)):
                Food1[iii] = sol1[iii]
        else:
            for iii in range(len(Food1)):
                Food1[iii] = sol2[iii]

    for i in range(1,dim):
        sol1 = [0 for i in range(dim + 2)]
        sol2 = [0 for i in range(dim + 2)]
        for iii in range(len(Food2)):
            sol1[iii] = Food2[iii]
            sol2[iii] = Food2[iii]
        sol1[i] = 0
        sol2[i] = 1
        sum1 = cal_1(Food1, dim, vehicle_position, controller_position, fix_controller_position)
        sum2 = cal_1(Food2, dim, vehicle_position, controller_position, fix_controller_position)
        if sum1 < sum2:
            for iii in range(len(Food2)):
                Food2[iii] = sol1[iii]
        else:
            for iii in range(len(Food2)):
                Food2[iii] = sol2[iii]

    for iii in range(len(Foods[0])):
        Foods[0][iii] = Food1[iii]
        Foods[1][iii] = Food2[iii]
    ######################### Greedy Start ###############################


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


        cal(Foods,re_list,dim,vehicle_position, controller_position,fix_controller_position)        ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
        pareto_front(Foods, re_list, dim)
        ## 统计解密度
        sol_density(Foods, re_list, dim)

        ####################################### Local search procedure ########################################################
        ### 对非支配解的每一位进行变异，从0变1 或从1变0，新的解录入总food中一起判断，筛选
        add1 = []
        k = 0
        for i in range(len(Foods)):
            if Foods[i][dim] == 1 and k < 5:
                k += 1
                for j in range(dim):
                    sol = [0 for i in range(dim + 2)]
                    for iii in range(len(Foods[i])):
                        sol[iii] = Foods[i][iii]
                    if sol[j] == 0:
                        sol[j] = 1
                    else:
                        sol[j] = 0
                    add1.append(sol)

        for i in add1:
            Foods.append(i)
        #######################################################################################################################


        re_list = [[0 for i in range(3)] for i in range(len(Foods))]
        ## 新产生的解与原解 全部放入Foods进行筛选
        Foods = new_old_sol_compare(Foods, re_list, dim, size1, vehicle_position, controller_position,
                                    fix_controller_position)
        re_list = [[0 for i in range(3)] for i in range(len(Foods))]
        cal(Foods, re_list, dim, vehicle_position, controller_position,
            fix_controller_position)  ## 求各解评价函数并初始化后两位 ## 统计帕累托前言，与支配解数量
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

        prob = [0 for i in range(size)]
        f = np.array(Foods)
        d = f[:,dim+1]
        max = np.sort(d)
        ind = np.argsort(d)
        leng = len(max)
        for i in range(leng):
            prob[ind[i]] = (max[i]-max[0])/(max[leng-1]-max[0])
            if f[ind[i],dim] >= 3:
                aa = 1
            else:
                aa = 1
            prob[ind[i]] = prob[ind[i]] * aa

        ################################### 交配过程 #########################################################
        ###### 通过锦标赛机制，通过适应度值进行概率计算，选出一对解，对两个解进行交配，将自己一定长度解与自己的对偶进行交换，新解录入food
        add2 = onlooker_bee(Foods, prob, size, dim)
        #############################################################################


        for i in add2:
            Foods.append(i)
        re_list = [[0 for i in range(3)] for i in range(len(Foods))]
        # cal(Foods, re_list, dim,vehicle_position, controller_position,fix_controller_position)
        ## 新产生的解与原解 全部放入Foods进行筛选
        Foods = new_old_sol_compare(Foods, re_list, dim, size,vehicle_position, controller_position,fix_controller_position)


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
                ## 计算所有开启控制器的位置
                on_controller = []
                for i in range(len(controller_position)):
                    if food[i] == 1:
                        on_controller.append(controller_position[i])
                for i in fix_controller_position:
                    on_controller.append(i)

                ## 计算每辆车离自己最近的控制器，v_c是最近控制器索引，v_c_d是最近控制器的离自己的距离，on_num是每个控制器所统治的车辆数量，vhop是车辆到最近控制器所需跳数
                on_num = [0 for i in range(len(on_controller))]
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
                diii = np.min(ddd.T, 0)
                pii = np.argmin(ddd.T, 0)

                # for i in range(len(vehicle_position)):
                #     mind = 999999
                #     for j in range(len(on_controller)):
                #         d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0],2) + \
                #             pow(vehicle_position[i][1] - on_controller[j][1],2))
                #         if d < mind:
                #             mind = d
                #             minj = j
                #     v_c[i] = minj
                #     v_c_d[i] = mind
                #     on_num[minj] += 1
                for i in pii:
                    on_num[i] += 1
                v_c = pii
                v_c_d = diii
                ## 最短距离整除最大通信距离，即可获得车辆到最近控制器所需跳数
                vhop = np.divide(v_c_d, gp.com_dis)
                vhop = vhop.astype(np.int)
                vhop += 1
                re_list[num][0] = function1(v_c_d)
                re_list[num][1] = function2(on_controller)
                re_list[num][2] = function3(on_controller, v_c)
                Foods[num][dim] = 6
                Foods[num][dim + 1] = 0



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
    for num in range(len(Foods)):
        food = Foods[num]
        ## 计算所有开启控制器的位置
        on_controller = []
        for i in range(len(controller_position)):
            if food[i] == 1:
                on_controller.append(controller_position[i])
        for i in fix_controller_position:
            on_controller.append(i)

        ## 计算每辆车离自己最近的控制器，v_c是最近控制器索引，v_c_d是最近控制器的离自己的距离，on_num是每个控制器所统治的车辆数量，vhop是车辆到最近控制器所需跳数
        on_num = [0 for i in range(len(on_controller))]
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
        diii = np.min(ddd.T, 0)
        pii = np.argmin(ddd.T, 0)

        # for i in range(len(vehicle_position)):
        #     mind = 999999
        #     for j in range(len(on_controller)):
        #         d = math.sqrt(pow(vehicle_position[i][0] - on_controller[j][0],2) + \
        #             pow(vehicle_position[i][1] - on_controller[j][1],2))
        #         if d < mind:
        #             mind = d
        #             minj = j
        #     v_c[i] = minj
        #     v_c_d[i] = mind
        #     on_num[minj] += 1
        for i in pii:
            on_num[i] += 1
        v_c = pii
        v_c_d = diii
        ## 最短距离整除最大通信距离，即可获得车辆到最近控制器所需跳数
        vhop = np.divide(v_c_d, gp.com_dis)
        vhop = vhop.astype(np.int)
        vhop += 1
        re_list[num][0] = function1(v_c_d)
        re_list[num][1] = function2(on_controller)
        re_list[num][2] = function3(on_controller, v_c)
        Foods[num][dim] = 6
        Foods[num][dim + 1] = 0

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
    print (time.time()-t)
    re_list = [[0 for i in range(3)] for i in range(size1)]
    cal(Foods,re_list,dim,vehicle_position,controller_position,fix_controller_position)
    for i in Foods:
        for j in range(len(controller_position)):
            if controller_position[j][1] < 1000 or controller_position[j][0] < 600:
                i[j] = 1
    return Foods,re_list

# ##((o(^_ ^)o)) 未确定控制器的位置
# controller = np.loadtxt('controller.txt')
# ##((o(^_ ^)o)) 固定控制器的位置
# fixcontroller = np.loadtxt('fixcontroller.txt')
# ## ((o(^_ ^)o))车辆位置
# vehicle = np.loadtxt('vehicle.txt')
# controller = controller[:,[0,1]]
# fixcontroller = fixcontroller[:,[0,1]]
# a = multiobj(len(controller),vehicle,controller,fixcontroller)
# print(a)