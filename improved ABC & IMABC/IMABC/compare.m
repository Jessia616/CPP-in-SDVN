% �ж�listed�Ƿ�list֧�� ���ļ�û����  (������)~*
function judge = compare(listed, list)
    a = 1;
    for i=1:length(listed)
        % �κ�һ��С��list��������֧��  �r(�s���t)�q
        if listed(i)<list(i)
            a = 0;
            break
        end
    end
    % 0Ϊ����֧�� 1Ϊ��֧��  ����������
    judge = a;
end
