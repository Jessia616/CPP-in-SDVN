function food1 = new_old_sol_compare(food, relist, size, dim)
    for i = 1:length(food)
        food(i,dim+1) = 0;
        food(i,dim+2) = 6;
    end
    food1 = [];
    % ͨ��POl���������2N��ɸѡN���⣬�����������  []~(������)~*
    food_no = PO1(relist, size, food, dim);
    for i =1:length(food_no)
        food1 = [food1;food(food_no(i),:)];
end
end
