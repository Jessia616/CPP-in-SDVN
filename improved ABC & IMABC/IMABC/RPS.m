% 参考点机制  <(￣︶￣)> <(￣︶￣)> <(￣︶￣)>
function add = RPS(relist,relistl,relistll,Z,len)
% 计算各个目标维度的最小值 以下为所有解参与的步骤
z = [];   
for i = 1:length(relist(1,:))
    re = relist(:,i);
    re = sort(re);
    z = [z,re(1)];
end

% 找到离各个目标维度坐标轴最近的解，以这些解组成一个超平面
a = [];
 for i = 1:length(relist(1,:))
     w = [];
    for j =1:length(relist(1,:))
        w = [w,0.0000001];
    end
    w(i) = 1;
    fi = [];
    for j =1:length(relist(:,1))
        aa = relist(j,:)./w;
        aa = sort(aa);
        fi = [fi, aa(length(relist(j,:)))];
    end
    [fi,ind] = sort(fi);
    a = [a,ind(1)];
 end
 
 % 根据刚刚的超平面，计算到各个坐标轴的解距，作为等会归一化的标准
 ep = [];
 for i=1:length(a)
     ep = [ep;relist(a(i),:)];
 end
 re = ones(length(ep(1,:)),1);
 
 a = ep\re;
 a = 1./a;
 a = a';
 bc = a - z;
 
% 通过截距，计算所有点的归一化坐标
list = [];
for i = 1:length(relist(1,:))
    re = relist(:,i);
    re = re - z(i);
    list = [list,re];
end
fn = [];
for i=1:length(list(1,:))
    aa = list(:,i)./bc(i);
    fn = [fn, aa];
end

% 通过截距，计算优解点的归一化坐标，并统计到参考点的距离
num_p = zeros(length(Z(:,1)),1);
if length(relistl)~= 0
    listl = [];
    for i = 1:length(relistl(1,:))
        re = relistl(:,i);
        re = re - z(i);
        listl = [listl,re];
    end
    fnl = [];
    for i=1:length(listl(1,:))
        aa = listl(:,i)./bc(i);
        fnl = [fnl, aa];
    end
    Cosine   = 1 - pdist2(fnl,Z,'cosine');
    % 所有解到参考点的距离
    Distance = repmat(sqrt(sum(fnl.^2,2)),1,length(Z(:,1))).*sqrt(1-Cosine.^2);
    % 将每个解关联到离自己的参考点，dl为最短距离，pil为最近的参考点索引
    [dl,pil] = min(Distance',[],1);
    % 统计每个参考点与自己关联的解数量
    for i = 1:length(pil)
        num_p(pil(i)) = num_p(pil(i))+1;
    end
end

% 通过截距，计算待筛选点的归一化坐标，并统计到参考点的距离
listll = [];
for i = 1:length(relistll(1,:))
    re = relistll(:,i);
    re = re - z(i);
    listll = [listll,re];
end
fnll = []
for i=1:length(listll(1,:))
    aa = listll(:,i)./bc(i);
    fnll = [fnll, aa];
end
Cosine   = 1 - pdist2(fnll,Z,'cosine');
 % 所有解到参考点的距离
Distance = repmat(sqrt(sum(fnll.^2,2)),1,length(Z(:,1))).*sqrt(1-Cosine.^2);
% 将每个解关联到离自己的参考点，dl为最短距离，pil为最近的参考点索引
[dll,pill] = min(Distance',[],1);
count = 0;

% 开始筛选，直到满足所需数量结束
add = [];
while count~=len
    % 找优解中最少解与之关联的参考点
    [jmin,ind] = sort(num_p);
    ind = find(num_p==jmin(1));
    numm = (fix(rand*(length(ind)))+1);
    ind = ind(numm);
    % 看带筛选解里有没有与此参考点相关联的
    f = find(pill==ind);
    % 筛选解里有的话
    if length(f)~=0
        % 如果优解里无解与该参考点关联，找待筛选解中距离最近的那个，将其入选
        if jmin(1) == 0
            dlll = dll(f);
            nlll = sort(dlll);   
            indd = find(dll==nlll(1));
            indd = indd(fix(rand*(length(indd)))+1);
            pill(indd) = 999;
            num_p(ind) = num_p(ind)+1;
            add = [add,indd];
            count = count +1;
        % 如果优解里有解与该参考点关联，待筛选解中随机找一个，将其入选    
        else
            indd = f(fix(rand*(length(f)))+1);
            pill(indd) = 999;
            num_p(ind) = num_p(ind)+1;
            add = [add,indd];
            count = count +1;
        end
    % 筛选解里没有与该参考点相关联的，本轮中放弃该参考点
    else
        num_p(ind) = 9999999;  
    end
end
end
        