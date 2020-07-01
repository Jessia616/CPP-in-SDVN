% 该文件没用上  ( ′ｏ`) 
function [op_list,no_list] = PO(list, n_o)
    op_list = [];
    no_list = [];
    b = 0;
    len = length(list);
    for i = 1:len;
        for j = 1:len
            if i ~= j
                b = compare(list(i,:),list(j,:));
                if b == 1
                    break;
                end
            end
        end
        if b == 1
            no_list = [no_list i];
            
        else
            op_list = [op_list i];
        end
        b = 0;
    end
end
        