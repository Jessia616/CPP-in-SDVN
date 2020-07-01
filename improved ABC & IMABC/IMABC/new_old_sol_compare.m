function food1 = new_old_sol_compare(food, relist, size, dim)
    for i = 1:length(food)
        food(i,dim+1) = 0;
        food(i,dim+2) = 6;
    end
    food1 = [];
    % 通过POl函数获得在2N中筛选N个解，并获得其索引  []~(￣▽￣)~*
    food_no = PO1(relist, size, food, dim);
    for i =1:length(food_no)
        food1 = [food1;food(food_no(i),:)];
end
end
