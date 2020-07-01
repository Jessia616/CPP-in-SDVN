% 判断listed是否被list支配 该文件没用上  (￣▽￣)~*
function judge = compare(listed, list)
    a = 1;
    for i=1:length(listed)
        % 任何一项小于list，即不被支配  ╮(╯▽╰)╭
        if listed(i)<list(i)
            a = 0;
            break
        end
    end
    % 0为不被支配 1为被支配  （＞▽＜）
    judge = a;
end
