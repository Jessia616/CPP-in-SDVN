% ��������  <(�����)>
function food = onlooker_bee(food, size, dim, prob, lb, ub,op_list)
    i = 1;
    t = 0;
    % ���̶�
    while (t < size)
        % �����С������� ��÷����ʸ�
        if rand < prob(i)
            t = t+1;
            % ��ӵ���ȱ��Լ�����ͬ���ǵ�һ��Ľ��������ǰ��
            len = fix(rand*(dim))+1;
            a = food(:,dim+1);
            b = food(:,dim+2);
            ab = [];
            % Ѱ��ӵ���ȱ��Լ�����ͬ���ǵ�һ��Ľ�
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