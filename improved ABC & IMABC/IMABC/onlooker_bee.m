% 跟随蜂操作  <(￣︶￣)>
function food = onlooker_bee(food, size, dim, prob, lb, ub,op_list)
    i = 1;
    t = 0;
    % 轮盘赌
    while (t < size)
        % 随机数小于其概率 获得繁衍资格
        if rand < prob(i)
            t = t+1;
            % 向拥挤度比自己大且同样是第一层的解随机步长前进
            len = fix(rand*(dim))+1;
            a = food(:,dim+1);
            b = food(:,dim+2);
            ab = [];
            % 寻找拥挤度比自己大且同样是第一层的解
            for j = 1:length(a)
                if a(j)==1 && b(j)>food(i,dim+2)
                    ab = [ab,j];
                end
            end
            neighbour=fix(rand*(length(ab)))+1;
            for time=1:len
                sol = food(i,:);
                Param2Change=fix(rand*dim)+1;
                %sol(Param2Change)=rand*(ub(Param2Change)-lb(Param2Change))+lb(Param2Change);
                sol(Param2Change)=food(i,Param2Change)+(food(i,Param2Change)- food(neighbour,Param2Change))*(rand-0.5)*2;
            end
            food = [food;sol];
        end
        i = i+1;
        if i == size+1
            i = 1;
        end
    end
end