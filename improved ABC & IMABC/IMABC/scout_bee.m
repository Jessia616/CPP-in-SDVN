% �������  (=���أ�=)
function food = scout_bee(food, pop_size, dim, size,lb,ub, op_list)
    % Ѱ�ҵ�ǰ�з�֧������Ϊ1����ӵ�������Ľ⣬��õĽ⣬�����������
    a = food(:,dim+2);
    [max,ind] = sort(a);
    for i=length(ind):1
        if food(ind(i),dim+1) == 1
            break;
        end
    end
    maxi = ind(i);
    
    sol = food(maxi,:);
    
    if maxi~=-1
        % ����pop_size����������
        for i= 1:pop_size
            can = sol;
            len = fix(rand*(dim))+1;
            a = food(:,dim+1);
            b = food(:,dim+2);
            ab = [];
            for j = 1:length(a)
                if a(j)==1 && b(j)>food(i,dim+2)
                    ab = [ab,j];
                end
            end
            neighbour=fix(rand*(length(ab)))+1;            
            for time=1:len*2
            Param2Change=fix(rand*dim)+1;
            % ���ά�ȵ�ֵ������仯
            can(Param2Change)=rand*(ub(Param2Change)-lb(Param2Change))+lb(Param2Change);
            % can(Param2Change)=food(i,Param2Change)+(food(i,Param2Change)- food(neighbour,Param2Change))*(rand-0.5)*2;
            end
            food = [food;can];
        end
    end
end
            