%  ��֧������r(�s���t)�q
function food = PF(food, relist,optimal, no, op_list, no_list,dim)
% ��ʼ����֧�������1-6��ԽСԽ�ã� ���ӵ����    ��Խ��Խ�ã�
for i = 1:length(food)
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
    % ��������¼��dim+1ά����
    food(:,dim+1) = FrontNo;
end