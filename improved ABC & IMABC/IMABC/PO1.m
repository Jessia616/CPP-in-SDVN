% ���÷�֧���������ӵ���ȣ�����ɸѡ��  <(�����)>
function optimal_list = PO1(relist, size, food, dim)
    op_list = [];
    no_list = [];
    b = 0;
    len = length(food);
    % ��ʼ����֧�������1-6��ԽСԽ�ã� ���ӵ����    ��Խ��Խ�ã�
    for i = 1:len
        food(i,dim+1) = 6;
        food(i,dim+2) = 0;
    end
    
    
%     list = relist;
%     for rank = 1:5
%     b = 0;
%     len = list(:,1);
%     len = length(len);
%     if len < 1
%         break
%     end
%     for i = 1:len;
%         for j = 1:len
%             if i ~= j
%                 b = compare(list(i,:),list(j,:));
%                 if b == 1
%                     break;
%                 end
%             end
%         end
%         if b == 0
%             food(i,dim+1) = rank;
%             op_list = [op_list,i];
%         else
%             no_list = [no_list,i];
%         end
%         b = 0;
%     end
%     list = list(no_list,:);
%     no_list = [];
%     op_list = [];
%     end
    % ���Դ�����ͳ�Ʒ�֧�������������ΪĿ������֤���������ƣ���Ⱥ��С�����ظ�������ֵ���������ֵ
    [FrontNo,MaxFNo] = NDSort(relist,0,length(food));
    for i=1:length(FrontNo)
        if FrontNo(i)>6
            FrontNo(i) = 6;
        end
    end
    FrontNo = FrontNo';
    food(:,dim+1) = FrontNo;
    
    list = relist;
    % �����ӵ����
    food = sol_density(list, food, length(op_list), size, dim);
    
    % ����֧�����������������򣬿�2N�����е�N�����������Ϊind
    a = food(:,dim+1);
    [aa,ind] = sort(a);
    ind = aa(size);
    for num=1:length(food)
        if aa(num) == ind
            break
        end
    end
    
    %����С��ind�ģ�ȫ����ѡ��һ�֣�Ŀ�꺯���������op_relist��
    op_relist = [];
    for num1=1:length(food)
        if food(num1,dim+1) < ind
            op_list = [op_list,num1];
            op_relist = [op_relist;relist(num1,:)];
        end
    end
    
    %�������ind�ģ�������ѡ��һ�֣���С��ǰ������ѡ�������������Ŀ�꺯���������no_relist��
    no_relist = [];
    for num2=1:length(food)
        if food(num2,dim+1) == ind
            no_list = [no_list,num2];
            no_relist = [no_relist;relist(num2,:)];
        end
    end
    ab = [];
    len = size-num+1;
    % ZΪ��һ����Ĳο��㣬��������Ⱥ��С�����
    [Z,~] = UniformPoint(100,length(relist(1,:)));
    
    % �������ind�ģ�ͨ���ο������ɸѡ ������ֵΪ��ѡ�������
    add = RPS([op_relist;no_relist],op_relist,no_relist,Z,len);
    
    % ȫ��С��ind����ɸѡ�����ind��Ϊ����ֵ
    op_list = [op_list,no_list(add)];
%     for num=1:length(food)
%         if food(num,dim+1)==ind
%             ab = [ab,num];
%         end
%     end
%     aaa = food(ab,dim+2);
%     [aaaa,indd] = sort(aaa);
%     indd=fliplr(indd);
%     for numm = 1:len
%         op_list = [op_list,ab(indd(numm))];
%     end
    optimal_list = op_list;
end