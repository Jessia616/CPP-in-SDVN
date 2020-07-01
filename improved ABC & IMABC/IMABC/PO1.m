% 利用非支配排序与解拥挤度，进行筛选解  <(￣︶￣)>
function optimal_list = PO1(relist, size, food, dim)
    op_list = [];
    no_list = [];
    b = 0;
    len = length(food);
    % 初始化非支配解排序（1-6，越小越好） 与解拥挤度    （越大越好）
    for i = 1:len
        food(i,dim+1) = 6;
        food(i,dim+2) = 0;
    end
    
    
%     list = relist;
%     for rank = 1:5
%     b = 0;
%     len = list(:,1);
%     len = length(len);
%     if len < 1
%         break
%     end
%     for i = 1:len;
%         for j = 1:len
%             if i ~= j
%                 b = compare(list(i,:),list(j,:));
%                 if b == 1
%                     break;
%                 end
%             end
%         end
%         if b == 0
%             food(i,dim+1) = rank;
%             op_list = [op_list,i];
%         else
%             no_list = [no_list,i];
%         end
%         b = 0;
%     end
%     list = list(no_list,:);
%     no_list = [];
%     op_list = [];
%     end
    % 用自带函数统计非支配排序，输入参数为目标结果举证，变量限制，种群大小，返回各自排序值，最大排序值
    [FrontNo,MaxFNo] = NDSort(relist,0,length(food));
    for i=1:length(FrontNo)
        if FrontNo(i)>6
            FrontNo(i) = 6;
        end
    end
    FrontNo = FrontNo';
    food(:,dim+1) = FrontNo;
    
    list = relist;
    % 计算解拥挤度
    food = sol_density(list, food, length(op_list), size, dim);
    
    % 将非支配排序排名正向排序，看2N个解中第N个解的排序，设为ind
    a = food(:,dim+1);
    [aa,ind] = sort(a);
    ind = aa(size);
    for num=1:length(food)
        if aa(num) == ind
            break
        end
    end
    
    %排序小于ind的，全部入选下一轮，目标函数结果存入op_relist中
    op_relist = [];
    for num1=1:length(food)
        if food(num1,dim+1) < ind
            op_list = [op_list,num1];
            op_relist = [op_relist;relist(num1,:)];
        end
    end
    
    %排序等于ind的，部分入选下一轮，大小由前面已入选解的数量决定，目标函数结果存入no_relist中
    no_relist = [];
    for num2=1:length(food)
        if food(num2,dim+1) == ind
            no_list = [no_list,num2];
            no_relist = [no_relist;relist(num2,:)];
        end
    end
    ab = [];
    len = size-num+1;
    % Z为归一化后的参考点，数量与种群大小相关联
    [Z,~] = UniformPoint(100,length(relist(1,:)));
    
    % 排序等于ind的，通过参考点进行筛选 ，返回值为入选解的索引
    add = RPS([op_relist;no_relist],op_relist,no_relist,Z,len);
    
    % 全部小于ind的与筛选后等于ind的为返回值
    op_list = [op_list,no_list(add)];
%     for num=1:length(food)
%         if food(num,dim+1)==ind
%             ab = [ab,num];
%         end
%     end
%     aaa = food(ab,dim+2);
%     [aaaa,indd] = sort(aaa);
%     indd=fliplr(indd);
%     for numm = 1:len
%         op_list = [op_list,ab(indd(numm))];
%     end
    optimal_list = op_list;
end