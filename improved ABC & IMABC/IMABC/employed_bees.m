% 雇佣蜂操作  (=￣ω￣=)
function food = employed_bees(food,size, dim, op_list, no_list, GlobalParams,lb,ub)
for i=1:size
        sol = food(i,:);
        % 如果不是非支配排序中第一层的解，向全局最优解随机步长前进
        if food(i,dim+1) ~= 1
            % 随机维度改变
            len = fix(rand*(dim))+1;
            for time=1:len*2
                Param2Change=fix(rand*dim)+1;
                %sol(Param2Change)=rand*(ub(Param2Change)-lb(Param2Change))+lb(Param2Change);
                sol(Param2Change)=food(i,Param2Change)+(food(i,Param2Change)- GlobalParams(Param2Change))*(rand-0.5)*2;
            end
        % 如果是非支配排序中第一层的解，向拥挤度比自己大且同样是第一层的解随机步长前进
        else
            len = fix(rand*(dim))+1;
            a = food(:,dim+1);
            b = food(:,dim+2);
            ab = [];
            for j = 1:length(a)
                % 向拥挤度比自己大且同样是第一层的解随机步长前进
                if a(j)==1 && b(j)>food(i,dim+2)
                    ab = [ab,j];
                end
            end
            neighbour=fix(rand*(length(ab)))+1;
            for time=1:len
                Param2Change=fix(rand*dim)+1;
                %sol(Param2Change)=rand*(ub(Param2Change)-lb(Param2Change))+lb(Param2Change);
                sol(Param2Change)=food(i,Param2Change)+(food(i,Param2Change)- food(neighbour,Param2Change))*(rand-0.5)*2;
            end
        end
        food = [food;sol];
end
end