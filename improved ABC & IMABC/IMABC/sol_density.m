% ��ӵ���ȼ���  ����������
function food = sol_density(relist, food, parato_len, size, dim)
for i = 1:length(relist(1,:))
    % ��ÿ��Ŀ�꺯��������������Լ���һ���Ľ��ȥǰһ���Ľ⣬���Լ��ھӵĲ��Ϊ�������Ŀ��ά�ȵ�ӵ��������Ŀ��ά����Ӽ��ɻ�ô˽��ӵ����
    [re,ind] = sort(relist(:,i));
    for j = 1:length(ind)
        if j == 1
            food(ind(j),dim+2) = food(ind(j),dim+2)+2*(re(j+1)-re(j))/(re(length(ind))-re(1));
        else
        if j == length(ind)
            food(ind(j),dim+2) = food(ind(j),dim+2)+2*(re(j)-re(j-1))/(re(length(ind))-re(1));
        else
        food(ind(j),dim+2) = food(ind(j),dim+2)+(re(j+1)-re(j-1))/(re(length(ind))-re(1));
        end
        end
    end
end