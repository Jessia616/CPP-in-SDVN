% �ο������  <(�����)> <(�����)> <(�����)>
function add = RPS(relist,relistl,relistll,Z,len)
% �������Ŀ��ά�ȵ���Сֵ ����Ϊ���н����Ĳ���
z = [];   
for i = 1:length(relist(1,:))
    re = relist(:,i);
    re = sort(re);
    z = [z,re(1)];
end

% �ҵ������Ŀ��ά������������Ľ⣬����Щ�����һ����ƽ��
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
 
 % ���ݸոյĳ�ƽ�棬���㵽����������Ľ�࣬��Ϊ�Ȼ��һ���ı�׼
 ep = [];
 for i=1:length(a)
     ep = [ep;relist(a(i),:)];
 end
 re = ones(length(ep(1,:)),1);
 
 a = ep\re;
 a = 1./a;
 a = a';
 bc = a - z;
 
% ͨ���ؾ࣬�������е�Ĺ�һ������
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

% ͨ���ؾ࣬�����Ž��Ĺ�һ�����꣬��ͳ�Ƶ��ο���ľ���
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
    % ���н⵽�ο���ľ���
    Distance = repmat(sqrt(sum(fnl.^2,2)),1,length(Z(:,1))).*sqrt(1-Cosine.^2);
    % ��ÿ������������Լ��Ĳο��㣬dlΪ��̾��룬pilΪ����Ĳο�������
    [dl,pil] = min(Distance',[],1);
    % ͳ��ÿ���ο������Լ������Ľ�����
    for i = 1:length(pil)
        num_p(pil(i)) = num_p(pil(i))+1;
    end
end

% ͨ���ؾ࣬�����ɸѡ��Ĺ�һ�����꣬��ͳ�Ƶ��ο���ľ���
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
 % ���н⵽�ο���ľ���
Distance = repmat(sqrt(sum(fnll.^2,2)),1,length(Z(:,1))).*sqrt(1-Cosine.^2);
% ��ÿ������������Լ��Ĳο��㣬dlΪ��̾��룬pilΪ����Ĳο�������
[dll,pill] = min(Distance',[],1);
count = 0;

% ��ʼɸѡ��ֱ������������������
add = [];
while count~=len
    % ���Ž������ٽ���֮�����Ĳο���
    [jmin,ind] = sort(num_p);
    ind = find(num_p==jmin(1));
    numm = (fix(rand*(length(ind)))+1);
    ind = ind(numm);
    % ����ɸѡ������û����˲ο����������
    f = find(pill==ind);
    % ɸѡ�����еĻ�
    if length(f)~=0
        % ����Ž����޽���òο���������Ҵ�ɸѡ���о���������Ǹ���������ѡ
        if jmin(1) == 0
            dlll = dll(f);
            nlll = sort(dlll);   
            indd = find(dll==nlll(1));
            indd = indd(fix(rand*(length(indd)))+1);
            pill(indd) = 999;
            num_p(ind) = num_p(ind)+1;
            add = [add,indd];
            count = count +1;
        % ����Ž����н���òο����������ɸѡ���������һ����������ѡ    
        else
            indd = f(fix(rand*(length(f)))+1);
            pill(indd) = 999;
            num_p(ind) = num_p(ind)+1;
            add = [add,indd];
            count = count +1;
        end
    % ɸѡ����û����òο���������ģ������з����òο���
    else
        num_p(ind) = 9999999;  
    end
end
end
        