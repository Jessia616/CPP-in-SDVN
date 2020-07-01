% 解拥挤度计算  （＞▽＜）
function food = sol_density(relist, food, parato_len, size, dim)
for i = 1:length(relist(1,:))
    % 对每个目标函数结果进行排序，自己后一个的解减去前一个的解，即自己邻居的差，即为我在这个目标维度的拥挤，所有目标维度相加即可获得此解的拥挤度
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