% ��Ӷ�����  (=���أ�=)
function food = employed_bees(food,size, dim, op_list, no_list, GlobalParams,lb,ub)
for i=1:size
        sol = food(i,:);
        % ������Ƿ�֧�������е�һ��Ľ⣬��ȫ�����Ž��������ǰ��
        if food(i,dim+1) ~= 1
            % ���ά�ȸı�
            len = fix(rand*(dim))+1;
            for time=1:len*2
                Param2Change=fix(rand*dim)+1;
                %sol(Param2Change)=rand*(ub(Param2Change)-lb(Param2Change))+lb(Param2Change);
                sol(Param2Change)=food(i,Param2Change)+(food(i,Param2Change)- GlobalParams(Param2Change))*(rand-0.5)*2;
            end
        % ����Ƿ�֧�������е�һ��Ľ⣬��ӵ���ȱ��Լ�����ͬ���ǵ�һ��Ľ��������ǰ��
        else
            len = fix(rand*(dim))+1;
            a = food(:,dim+1);
            b = food(:,dim+2);
            ab = [];
            for j = 1:length(a)
                % ��ӵ���ȱ��Լ�����ͬ���ǵ�һ��Ľ��������ǰ��
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