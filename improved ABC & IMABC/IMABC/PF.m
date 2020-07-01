%  非支配排序╮(╯▽╰)╭
function food = PF(food, relist,optimal, no, op_list, no_list,dim)
% 初始化非支配解排序（1-6，越小越好） 与解拥挤度    （越大越好）
for i = 1:length(food)
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
    % 将排序结果录入dim+1维度中
    food(:,dim+1) = FrontNo;
end