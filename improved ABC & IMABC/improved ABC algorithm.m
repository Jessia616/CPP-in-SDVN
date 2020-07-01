
function [GlobalMin,memorize,time]=CBA(SearchAgents_no,Max_iter,Lb,Ub,dim,fobj)

starttime=cputime;
%/* �˹���Ⱥ�㷨�Ŀ��Ʋ���*/
NP=SearchAgents_no; %/* ��Ⱥ���� (��Ӷ�������������ܺ�)*/
FoodNumber=(SearchAgents_no/2); %/*ʳ����Դ�������൱�ڷ�Ⱥ������һ��*/
maxCycle=Max_iter; %/*����ѭ������ {��ֹ����������}*/
memorize=zeros(1,maxCycle);


%/* ���ĳһ��������Ĳ�������*/
objfun=fobj; %���Ż��Ĵ��ۺ���
D=dim; %/*�����д��Ż��Ĳ���������*/
ub=ones(1,D)*Ub; %/*����1���½�: ����һ��Ԫ��ȫΪ1������*/
lb=ones(1,D)*Lb;%/*����1���Ͻ�.*/
%/*���޶�������֮��Ŀռ��Ⱥ���޷��ﵽ��������ʳ��*/
%limit=FoodNumber*D���޶�������limit�Ĺ�ʽ����; 
limit=FoodNumber*D; 
runtime=1;%/*�㷨�������ж���Բ�����³����*/
times = 0;
numm = 0;
%%%%%%%%%%%%%%%%%%%%%% �õ��Ĳ���%%%%%%%%%%%%%%%%%%%%
%ObjVal[FoodNumber];  /*Ŀ�꺯��f ��һ������Ŀ�꺯����ֵ�ĺ�������ʳ����Դ�й�ϵ */
%Fitness[FoodNumber]; /*fitness is ������Ӧ��ֵ��һ���������������Ƕȣ� ͬ����ʳ����Դ�й�*/
%trial[FoodNumber]; /*trial is ��¼����ʧ��ʱ����������������*/
%prob[FoodNumber]; /*prob isһ������ʳ����н⣩�����������ʴ�С������*/
%solution [D]; /*�µĿ��н� (����) �ɹ�ʽ v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})������ j is ��������ɵĲ�����r and k �ǲ�ͬ�� i�������ѡ��Ľ�*/
%ObjValSol; /*һ���¿��н��Ŀ�꺯����ֵ*/
%FitnessSol; /*�¿��н����Ӧֵ*/
%neighbour, param2change; /*param2change ��Ӧ�� j, neighbour ��Ӧ��ʽ v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})*/
%GlobalMin; /*�˹���Ⱥ�㷨�ó������Ž�*/
%GlobalParams[D]; /*���Ž���Ż�������������ʽ�����*/
%GlobalMins[runtime]; /*��¼���ִ�й����в������Ž��ʵ������ʱ��*/

GlobalMins=zeros(1,runtime);

for r=1:runtime
  
% /*����ʳ����Դ������ʼ�� */
%/*������ȡֵ��Χ����[lb,ub]֮��. �������һ�������в�ͬ��ȡֵ��Χ, ��������lb[j], ub[j]���� lb ��ub */
    
%MAX and MIN

Range = repmat((ub-lb),[FoodNumber 1]);
Lower = repmat(lb, [FoodNumber 1]);
%Foodsָʳ������� 
%of Foods �����ÿһ��is һ���������д��Ż���D���� . 
%The number of rows of Foods matrix equals to the FoodNumber
Foods = rand(FoodNumber,D) .* Range + Lower;
% fprintf('foods(1,1)=%g\n',Foods(1,1));


for k=1:FoodNumber
    ObjVal(k)=feval(objfun,Foods(k,:));
end
%ObjVal=feval(objfun,Foods);

%fprintf('ObjVal=%d\n',ObjVal);


Fitness=calculateFitness(ObjVal);

%����ʵ����̣�����������
trial=zeros(1,FoodNumber);

%/*��¼���ʳ����Դλ��*/
BestInd=find(ObjVal==min(ObjVal));
BestInd=BestInd(end);
GlobalMin=ObjVal(BestInd);
GlobalParams=Foods(BestInd,:);


iter=1;

while ((iter <= maxCycle)) %��Ӷ��
    % new �Թ�Ӷ����Ӧ��ֵ�������� \(��o��)
    [aa,index]=sort(ObjVal);
    % new ������Ӷ������Ĳ������ɵ��������ͻع�ֵ���� (��o��)/
    ia = iter - Max_iter/10/1.5 * numm;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��Ӷ��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:(FoodNumber)
        sol = Foods(index(i),:);
         %/*���仯�Ĳ����������*/ 
        Param2Change=fix(rand*D)+1;
        % new �Թ�Ӷ����Ӧ��ֵ����������Ӧ�ȿ�ǰ�Ĺ�Ӷ�����ԭ���������������Ӧ�ȿ���Ĺ�Ӷ�������Ž����������ǰ����ǰ���������������ı䣬ǰ�����������࣬���������Ž�ǰ���ࡣ \(��o��)
        if i <= FoodNumber * (yyec(0,maxCycle,1,1,ia))        
            %/*һ�������ѡ��Ŀ��нⷽ�������ڲ���һ����i�ı������*/
            neighbour=fix(rand*(FoodNumber))+1;       
            %/*ȷ����������Ľ��ԭ��i�Ĳ�ͬ��*/        
            while(neighbour==index(i))
                neighbour=fix(rand*(FoodNumber))+1;
            end      
            % ��Ⱥ�˶��ٶȵ�����ʽ /*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
            % new �������� (��_��)
            sol(Param2Change)=Foods(index(i),Param2Change)+(Foods(index(i),Param2Change)-Foods(neighbour,Param2Change))*(rand-0.5)*2;
        else    
            % new �����Ž�ǰ�� (��o��)/
            sol(Param2Change)=Foods(index(i),Param2Change)+(Foods(index(i),Param2Change)- GlobalParams(Param2Change))*(rand-1);
        end
        %  /*��������Ĳ���ֵԽ��, �ͽ���ֵ�趨Ϊ�ڽ��߽�ֵ*/
        ind=find(sol<lb);
        sol(ind)=lb(ind);
        ind=find(sol>ub);
        sol(ind)=ub(ind);
        
        %�����²����Ľ�
        ObjValSol=feval(objfun,sol);
        %fprintf('FOODS2 OVER\n');
        FitnessSol=calculateFitness(ObjValSol);
        
       % /*һ��̰��ѡ����Ա������ڵ�ǰ��i�������������½���*/
       %/*�������������½������Ҫ���ڵ�ǰ��i,��
       %r�ñ�����滻��ǰ��i������׷�ټ�����*/
       if (FitnessSol>Fitness(index(i))) 
            Foods(index(i),:)=sol;
            Fitness(index(i))=FitnessSol;
            ObjVal(index(i))=ObjValSol;
            trial(index(i))=0;
        else
            trial(index(i))=trial(index(i))+1; %/*�����ǰ��i���ܱ��Ľ�, ��׷�ټ�������ֵ��1*/
       end;       
    end;

    %%%%%%%%%%%%%%%%%%%%%%%% ���ʼ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %/* ʳ�ﱻѡ��ĸ��ʺ�ʳ���������Ӧ�ɱ���*/
    %/*���Բ��ò�ͬ�ķ������������ֵ��С*/
    %/*�ٸ����� prob(i)=fitness(i)/sum(fitness)*/ 
    %/*���߲���ƫ���½�ĳ̶������� prob(i)=a*fitness(i)/max(fitness)+b*/

    prob=(0.9.*Fitness./max(Fitness))+0.1;
  
    %%%%%%%%%%%%%%%%%%%%%%%% �����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    i=1;
    t=0;
    while(t<FoodNumber)
        if(rand<prob(i))
            t=t+1;
            %/*���ı�����������������*/
            Param2Change=fix(rand*D)+1;
            
            %/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
            neighbour=fix(rand*(FoodNumber))+1;
        
            %/*Randomly selected solution must be different from the solution i*/        
            while(neighbour==i)
                neighbour=fix(rand*(FoodNumber))+1;
            end;
            
            sol=Foods(i,:);
            %  /*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
            %mutant operation
            sol(Param2Change)=Foods(i,Param2Change)+(Foods(i,Param2Change)-Foods(neighbour,Param2Change))*(rand-0.5)*2;
            
            %  /*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
            ind=find(sol<lb);
            sol(ind)=lb(ind);
            ind=find(sol>ub);
            sol(ind)=ub(ind);
            
            %evaluate new solution
            ObjValSol=feval(objfun,sol);
        
            FitnessSol=calculateFitness(ObjValSol);
            
            % /*a greedy selection is applied between the current solution i and its mutant*/
            %/*If the mutant solution is better than the current solution i, 
            %replace the solution with the mutant and reset the trial counter of solution i*/
            if (FitnessSol>Fitness(i)) 
                Foods(i,:)=sol;
                Fitness(i)=FitnessSol;
                ObjVal(i)=ObjValSol;
                trial(i)=0;
            else
                trial(i)=trial(i)+1; %/*if the solution i can not be improved, increase its trial counter*/
            end;
        end;     
        i=i+1;
        if (i==(FoodNumber)+1) 
            i=1;
        end;   
    end; 


    %/*The best food source is memorized*/
    aaa = GlobalMin;
    ind=find(ObjVal==min(ObjVal));
    ind=ind(end);
    if (ObjVal(ind)<GlobalMin)
        GlobalMin=ObjVal(ind);
        GlobalParams=Foods(ind,:);
    end
    if (aaa == GlobalMin)
        times = times + 1;
    end

    % new ���Ž��ȶ�����n�Σ�nΪ��������������4�����������Ž�������� \(��o��)
    % sizeΪ������Ⱥ��С
    size = 10;
    if (times >= Max_iter/4) || (mod(iter,1000) == 0)
        can = zeros(size+1,dim);
        sol=Foods(ind,:);
         % new ��������Ž���Χ��10���� (��o��)/
        for f = 1:size
            % new ���ȡ�ı��ά�� \(��o��)
            Param2Change=fix(rand*D)+1;
            % new ����Խ��Ĳ��� (��o��)/
            if (ub - Foods(ind,Param2Change) <= Foods(ind,Param2Change) - lb)
                minx = ub - Foods(ind,Param2Change);
            else
                minx = Foods(ind,Param2Change) - lb;
            end
            % new ���ȡƫ���� \(��o��)
            a = (rand - 0.5) * 2 * 0.5;
            a = minx * a;
            % new ��ȡһ���������� (��o��)/
            cand = sol;
            cand(Param2Change) = Foods(ind,Param2Change) + a(1);
            for (d = 1:dim)
                can(f,d) = cand(d);
            end
        end
        % new ��ԭ���������ĵ�n+1��λ�� \(��o��)
        for d = 1:dim
            can(size+1,d) = Foods(ind,d);
        end
        minx = 99999;
        % new �ҳ�10�����������ԭ��֮������Ž� (��o��)/
        for d = 1:size+1
            re = feval(objfun,can(d));
            if (re < minx)
                minx = re;
                mini = d;
            end
        end
        % new ���·�Դ�����δ�䣬����Դ���������һ�����ı䣬������Դ \(��o��)
        if (mini == size+1)
            trial(ind) = trial(ind) + 1;
        else
            Foods(ind,:)=can(mini);
            Fitness(ind)=calculateFitness(minx);
            ObjVal(ind)=minx;
            trial(ind)=0;
        end
        % new �ȶ��������� (��o��)/
        times = 0;
        % new ���ӻع�ֵ \(��o��)
        numm = numm + 1;
    end     
    %%%%%%%%%%%%����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %/*determine the food sources whose trial counter exceeds the "limit" value. 
    %In Basic ABC, only one scout is allowed to occur in each cycle*/

    ind=find(trial==max(trial));
    ind=ind(end);
    if (trial(ind)>limit)
        Bas(ind)=0;
        sol=(ub-lb).*rand(1,D)+lb;
        ObjValSol=feval(objfun,sol);
        FitnessSol=calculateFitness(ObjValSol);
        Foods(ind,:)=sol;
        Fitness(ind)=FitnessSol;
        ObjVal(ind)=ObjValSol;
    end;



    
    memorize(iter)=memorize(iter)+GlobalMin;
    iter=iter+1;
end % ��Ⱥ�㷨����

GlobalMins(r)=GlobalMin;
end; %���й��̽���
GloblaMin = min(GlobalMins)
endtime=cputime;
time=endtime-starttime;
end