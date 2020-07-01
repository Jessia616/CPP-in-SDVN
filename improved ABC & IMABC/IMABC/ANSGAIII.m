function ANSGAIII(GLOBAL)
    % ��������
    lb = GLOBAL.lower;
    % ��������
    ub = GLOBAL.upper;
    % ����ά��
    dim = GLOBAL.D;
    % ��Ⱥ��ģ
    size = GLOBAL.N;
    % ����������
    maxiter = GLOBAL.maxgen;
    % �ȶ�����
    timess = 0;
    % ��Ⱥ dim+1���֧�������� dim+2���ӵ����
    food = [];
    % Ŀ����
    relist = [];
    parato_len = 0;
    times = 0;
    % �����ʼ����Ⱥ  <(�����)>
    Population = GLOBAL.Initialization();
    for i = 1:size
        food = [food;[Population(i).decs,0,0]];
    end
while GLOBAL.NotTermination(Population)
        times = times + 1;
        relist = [];
        pre_food = food;
        % ��¼��ȺĿ�꺯�����  []~(������)~*
        for i = 1:size
            relist = [relist;Population(i).objs];
        end
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % ��֧������ (=���أ�=)
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % ��ӵ���ȼ��� Խ��˵��Խ�� (������)~*
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
        %�ҵ�����Ϊ1�� ��ӵ����������Ϊȫ�����Ž�  (������)
        a = food(:,dim+2);
        [max,ind] = sort(a);
        for i = 1:length(max)
            if food(ind(i),dim+1) == 1
                mini = ind(i);
            end
        end
        GlobalParams = food(mini,:);
        
        % ��Ӷ�����  �r(�s���t)�q
        food = employed_bees(food,size, dim, optimal_list, no_list, GlobalParams,lb,ub);
        
        % ͳ�ư������ӽ���Ŀ�꺯���������СΪ2N ( ���`) 
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % ͨ����֧������Ͳο������2N��ѡ��N���� (*��?��*)
        food = new_old_sol_compare(food, relist, size, dim);

        % �����µ�Ŀ�꺯�����  ����������
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
       % ��֧������  ����������
       food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
       
       % ��ӵ���ȼ��� Խ��˵��Խ��  (��?��) 
       food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
       % �������̶ĸ��� ��֧������Ϊ1�� ����Ϊ0 ������ݽ�ӵ���ȹ�һ������  (*��?��*)
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
        
        % ��������  (*��?��*)
        food = onlooker_bee(food,size, dim,prob,lb,ub,optimal_list);
        
        % ͳ�ư������ӽ���Ŀ�꺯���������СΪ2N  ( ���`) 
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % ͨ����֧������Ͳο������2N��ѡ��N����  �r(�s���t)�q
        food = new_old_sol_compare(food, relist, size, dim);
        
        % �����µ�Ŀ�꺯�����  (������)~*
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % ��֧������   []~(������)~*
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % ��ӵ���ȼ��� Խ��˵��Խ��  (=���أ�=)
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
         
        % �統ǰ��Ⱥ����һ����Ⱥ��ͬ���ȶ�������1  �r(�s���t)�q
        if food == pre_food
            timess = timess + 1;
        end
        
        % �ȶ����������ÿ1000�ε���������һ���������  �r(�s���t)�q
        if (timess >= maxiter / 4) || (mod(times,10)== 0)
            pop_size = size/10;
             food = scout_bee(food, pop_size, dim, size,lb,ub,optimal_list);
            timess = 0;
        end
        
        % ͳ�ư������ӽ���Ŀ�꺯���������СΪ2N  <(�����)>
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        % ͨ����֧������Ͳο������2N��ѡ��N����  <(�����)>
        food = new_old_sol_compare(food, relist, size, dim);
        
        % �����µ�Ŀ�꺯�����  ����������
        relist = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            relist = [relist;can.objs];
        end
        
        optimal = [];
        no = [];
        optimal_list = [];
        no_list = [];
        
        % ��֧������   ����������
        food = PF(food, relist,optimal, no, optimal_list,no_list, GLOBAL.D);
        
        % ��ӵ���ȼ��� Խ��˵��Խ��  (*��?��*)
        food = sol_density(relist,food, parato_len, size,GLOBAL.D);
        
        foods = [];
        for i =1:length(food)
            can = INDIVIDUAL(food(i,1:dim));
            foods = [foods,can];
        end
        Population = foods;
        
        % ��ӡ��������
        a = times
        if mod(a,10) == 0
            a = times
        end
end 
end
        
        
            
            
    