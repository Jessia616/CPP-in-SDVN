function ANSGAIII(GLOBAL)
    % 变量下限
    lb = GLOBAL.lower;
    % 变量上限
    ub = GLOBAL.upper;
    % 变量维度
    dim = GLOBAL.D;
    % 种群规模
    size = GLOBAL.N;
    % 最大迭代次数
    maxiter = GLOBAL.maxgen;
    % 稳定次数
    timess = 0;
    % 种群 dim+1存非支配排序结果 dim+2存解拥挤度
    food = [];
    % 目标结果
    relist = [];
    parato_len = 0;
    times = 0;
    % 随机初始化种群  <(￣︶￣)>
    Population = GLOBAL.Initialization();
    for i = 1:size
        food = [food;[Population(i).decs,0,0]];
    end
while GLOBAL.NotTermination(Population)
        times = times + 1;
        relist = [];
        pre_food = food;
        % 记录种群目标函数结果  []~(￣▽￣)~*
        for i = 1:size
            relist = [relist;Population(i).objs];
        end
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % 非支配排序 (=￣ω￣=)
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % 解拥挤度计算 越大说明越好 (￣▽￣)~*
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
        %找到排序为1中 解拥挤度最大的作为全局最优解  (￣ˇ￣)
        a = food(:,dim+2);
        [max,ind] = sort(a);
        for i = 1:length(max)
            if food(ind(i),dim+1) == 1
                mini = ind(i);
            end
        end
        GlobalParams = food(mini,:);
        
        % 雇佣蜂操作  ╮(╯▽╰)╭
        food = employed_bees(food,size, dim, optimal_list, no_list, GlobalParams,lb,ub);
        
        % 统计包括增加解后的目标函数结果，大小为2N ( ′ｏ`) 
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % 通过非支配排序和参考点机制2N中选择N个解 (*￣?￣*)
        food = new_old_sol_compare(food, relist, size, dim);

        % 计算新的目标函数结果  （＞▽＜）
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
       % 非支配排序  （＞▽＜）
       food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
       
       % 解拥挤度计算 越大说明越好  (￣?￣) 
       food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
       % 计算轮盘赌概率 非支配排序不为1的 概率为0 否则根据解拥挤度归一化操作  (*￣?￣*)
        prob = [];
        a = food(:,dim+2);
        [max,ind] = sort(a);
        for i = 1:length(max)
                prob(ind(i)) = (max(i)-max(1))/(max(length(max))-max(1));
                if food(ind(i),dim+1) >= 2
                    aa = 0;
                else
                    aa = (5-food(ind(i),dim+1))/10+1;
                end
                prob(ind(i)) = prob(ind(i)) * aa;
        end
        
        % 跟随蜂操作  (*￣?￣*)
        food = onlooker_bee(food,size, dim,prob,lb,ub,optimal_list);
        
        % 统计包括增加解后的目标函数结果，大小为2N  ( ′ｏ`) 
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % 通过非支配排序和参考点机制2N中选择N个解  ╮(╯▽╰)╭
        food = new_old_sol_compare(food, relist, size, dim);
        
        % 计算新的目标函数结果  (￣▽￣)~*
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % 非支配排序   []~(￣▽￣)~*
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % 解拥挤度计算 越大说明越好  (=￣ω￣=)
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
         
        % 如当前种群与上一代种群相同，稳定次数加1  ╮(╯▽╰)╭
        if food == pre_food
            timess = timess + 1;
        end
        
        % 稳定次数过大或每1000次迭代，进行一次侦察蜂操作  ╮(╯▽╰)╭
        if (timess >= maxiter / 4) || (mod(times,10)== 0)
            pop_size = size/10;
             food = scout_bee(food, pop_size, dim, size,lb,ub,optimal_list);
            timess = 0;
        end
        
        % 统计包括增加解后的目标函数结果，大小为2N  <(￣︶￣)>
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % 通过非支配排序和参考点机制2N中选择N个解  <(￣︶￣)>
        food = new_old_sol_compare(food, relist, size, dim);
        
        % 计算新的目标函数结果  （＞▽＜）
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % 非支配排序   （＞▽＜）
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % 解拥挤度计算 越大说明越好  (*￣?￣*)
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
        foods = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            foods = [foods,can];
        end
        Population = foods;
        
        % 打印迭代次数
        a = times
        if mod(a,10) == 0
            a = times
        end
end 
end
        
        
            
            
    