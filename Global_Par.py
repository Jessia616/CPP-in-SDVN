# 发送路由请求节点比例
com_node_rate = 0.2
# 分组传输最大距离
com_dis = 500

# 控制器覆盖范围
con_dis = 500

# 画面更新时间
update_period = 1

# 失败次数
fail_time = 0

# 重传次数
re_time = 3

MAX = 600000

# 单跳时延
sum = 0

# 包大小
ps = 1024

# 产生包速率
pps = 10

# 控制器最大负载
cml = 20

# 到达率
ar = 50

# 处理率
rr = 60

# 最大队列长度
aifs = 20
# 最小竞争窗口
cwmin = 2
# 最大竞争窗口
cwmax = 1024
# 重发概率
p = 0.3
# 最大重试次数
c = 3
ifg = 9.6
tslot = 20
# 最大队列长度
N = 20
record = []
#成功次数
success_num = 0
effi = 0
shop = 0

def pareto_front(Foods, re_list, dim, optimal, no):
    ## 统计帕累托前言，与支配解数量
    for i in range(len(Foods)):
        re_list[i][0] = function1(Foods[i])
        re_list[i][1] = function2(Foods[i])
        re_list[i][2] = function3(Foods[i])
        Foods[i][dim] = 0
        Foods[i][dim + 1] = 0
    optimal_list = []
    no_list = []
    b = 0
    ## 统计帕累托前言，与支配解数量
    for i in range(len(re_list)):
        for j in range(len(re_list)):
            if i != j:
                b = compare(re_list[i], re_list[j])
                ## 如被支配 失去测试资格 跳出循环
                if b == 1:
                    ## 统计支配解数量
                    Foods[j][dim] += 1
                    break
        ## i被支配 劣解
        if b == 1:
            no_list.append(i)
        ## i未被任何解支配 优解
        else:
            optimal_list.append(i)
        b = 0
    for i in optimal_list:
        optimal.append(Foods[i])
    for i in no_list:
        no.append(Foods[i])
    return len(optimal_list)

def sol_density(Foods, parato_len , size1, dim):
    ## 统计解密度
    for i in Foods:
        ## 存i与其他解的距离信息
        distance = [0 for i in range(len(Foods))]
        for j in range(len(Foods)):
            lene = 0
            for num in range(len(i)):
                if i[num] != Foods[j][num]:
                    lene += 1
            distance[j] = lene
        ## 排序 找到 第k近的解
        b = sorted(range(len(distance)), key=lambda k: distance[k])
        k = int(math.sqrt(parato_len + size1))
        ## 求出密度值
        i[dim + 1] = 1 / (distance[k] + 2)

def employed_bees(Foods, size, dim, optimal_list, no_list, GlobalParams):
    for i in range(size):
        sol = [0 for i in range(dim + 2)]
        for iii in range(len(Foods[i])):
            sol[iii] = Foods[i][iii]
        ## 随机决定更改的维度
        Param2Change = random.randint(0, dim - 1)
        ## 劣解向最优解前进
        if (no_list.count(i) != 0):
            sol[Param2Change] = GlobalParams[Param2Change]
        else:
            ## 前沿中的解邻域搜索
            if (optimal_list.count(i) != 0):
                if sol[Param2Change] == 0:
                    sol[Param2Change] = 1
                else:
                    sol[Param2Change] = 0
        ## 新解融入原解中
        Foods.append(sol)

def new_old_sol_compare(Foods, re_list,dim, size):
    for i in range(len(Foods)):
        re_list[i][0] = function1(Foods[i])
        re_list[i][1] = function2(Foods[i])
        re_list[i][2] = function3(Foods[i])
        Foods[i][dim] = 0
        Foods[i][dim + 1] = 0
    Food = Pareto_Optimality_1(re_list, size, Foods, dim)
    Foods1 = []
    for i in Food:
        Foods1.append(Foods[i])
    ## 当前Foods为筛选后的新解
    return Foods1

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

def scout_bee(Foods, dim, pop_size):
    max = 0
    max_index = -1
    ## 找到密度值最高的解
    for i in range(len(Foods)):
        if Foods[i][dim + 1] > max:
            max = Foods[i][dim + 1]
            max_index = i
        else:
            ## 若相同 找更优的解 也就是 支配解更多的
            if Foods[i][dim + 1] == max:
                if Foods[i][dim] >= Foods[max_index][dim]:
                    max = Foods[i][dim]
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