
function [GlobalMin,memorize,time]=CBA(SearchAgents_no,Max_iter,Lb,Ub,dim,fobj)

starttime=cputime;
%/* 人工蜂群算法的控制参数*/
NP=SearchAgents_no; %/* 种群数量 (雇佣蜂和侦查蜂的数量总和)*/
FoodNumber=(SearchAgents_no/2); %/*食物资源的数量相当于蜂群数量的一半*/
maxCycle=Max_iter; %/*搜索循环次数 {终止迭代的条件}*/
memorize=zeros(1,maxCycle);


%/* 针对某一具体问题的参数假设*/
objfun=fobj; %待优化的代价函数
D=dim; %/*问题中待优化的参数的数量*/
ub=ones(1,D)*Ub; %/*参数1的下界: 产生一个元素全为1的数组*/
lb=ones(1,D)*Lb;%/*参数1的上界.*/
%/*‘限定条件’之外的空间蜂群将无法达到其中搜索食物*/
%limit=FoodNumber*D‘限定条件’limit的公式定义; 
limit=FoodNumber*D; 
runtime=1;%/*算法将被运行多次以测试其鲁棒性*/
times = 0;
numm = 0;
%%%%%%%%%%%%%%%%%%%%%% 用到的参数%%%%%%%%%%%%%%%%%%%%
%ObjVal[FoodNumber];  /*目标函数f 是一个反回目标函数估值的函数，与食物来源有关系 */
%Fitness[FoodNumber]; /*fitness is 反回适应度值的一个函数（从数量角度） 同样与食物来源有关*/
%trial[FoodNumber]; /*trial is 记录搜索失败时所产生的搜索次数*/
%prob[FoodNumber]; /*prob is一个包含食物（可行解）被搜索到概率大小的向量*/
%solution [D]; /*新的可行解 (邻域) 由公式 v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})产生， j is 是随机生成的参数，r and k 是不同于 i的随机被选择的解*/
%ObjValSol; /*一个新可行解的目标函数估值*/
%FitnessSol; /*新可行解的适应值*/
%neighbour, param2change; /*param2change 对应于 j, neighbour 对应等式 v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})*/
%GlobalMin; /*人工蜂群算法得出的最优解*/
%GlobalParams[D]; /*最优解的优化方案（迭代公式结果）*/
%GlobalMins[runtime]; /*记录多次执行过程中产生最优解的实际运行时间*/

GlobalMins=zeros(1,runtime);

for r=1:runtime
  
% /*所有食物来源均被初始化 */
%/*参数的取值范围均在[lb,ub]之间. 如果任意一个参数有不同的取值范围, 则用数组lb[j], ub[j]代替 lb 和ub */
    
%MAX and MIN

Range = repmat((ub-lb),[FoodNumber 1]);
Lower = repmat(lb, [FoodNumber 1]);
%Foods指食物的数量 
%of Foods 矩阵的每一行is 一个向量持有待优化的D参数 . 
%The number of rows of Foods matrix equals to the FoodNumber
Foods = rand(FoodNumber,D) .* Range + Lower;
% fprintf('foods(1,1)=%g\n',Foods(1,1));


for k=1:FoodNumber
    ObjVal(k)=feval(objfun,Foods(k,:));
end
%ObjVal=feval(objfun,Foods);

%fprintf('ObjVal=%d\n',ObjVal);


Fitness=calculateFitness(ObjVal);

%重置实验过程，计数器清零
trial=zeros(1,FoodNumber);

%/*记录最佳食物来源位置*/
BestInd=find(ObjVal==min(ObjVal));
BestInd=BestInd(end);
GlobalMin=ObjVal(BestInd);
GlobalParams=Foods(BestInd,:);


iter=1;

while ((iter <= maxCycle)) %雇佣蜂
    % new 对雇佣蜂适应度值进行排序 \(＾o＾)
    [aa,index]=sort(ObjVal);
    % new 决定雇佣蜂比例的参数，由迭代次数和回滚值决定 (＾o＾)/
    ia = iter - Max_iter/10/1.5 * numm;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%雇佣蜂%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:(FoodNumber)
        sol = Foods(index(i),:);
         %/*待变化的参数随机产生*/ 
        Param2Change=fix(rand*D)+1;
        % new 对雇佣蜂适应度值进行排序，适应度靠前的雇佣蜂进行原版的邻域搜索，适应度靠后的雇佣蜂向最优解以随机步长前进，前后比例随迭代次数改变，前期邻域搜索多，后期向最优解前进多。 \(＾o＾)
        if i <= FoodNumber * (yyec(0,maxCycle,1,1,ia))        
            %/*一个随机被选择的可行解方案被用在产生一个解i的变异解上*/
            neighbour=fix(rand*(FoodNumber))+1;       
            %/*确保变异产生的解和原解i的不同性*/        
            while(neighbour==index(i))
                neighbour=fix(rand*(FoodNumber))+1;
            end      
            % 蜂群运动速度迭代公式 /*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
            % new 邻域搜索 (〃_〃)
            sol(Param2Change)=Foods(index(i),Param2Change)+(Foods(index(i),Param2Change)-Foods(neighbour,Param2Change))*(rand-0.5)*2;
        else    
            % new 向最优解前进 (＾o＾)/
            sol(Param2Change)=Foods(index(i),Param2Change)+(Foods(index(i),Param2Change)- GlobalParams(Param2Change))*(rand-1);
        end
        %  /*如果产生的参数值越界, 就将其值设定为邻近边界值*/
        ind=find(sol<lb);
        sol(ind)=lb(ind);
        ind=find(sol>ub);
        sol(ind)=ub(ind);
        
        %评估新产生的解
        ObjValSol=feval(objfun,sol);
        %fprintf('FOODS2 OVER\n');
        FitnessSol=calculateFitness(ObjValSol);
        
       % /*一种贪婪选择策略被运用在当前解i及其变异产生的新解上*/
       %/*如果变异产生的新解的质量要优于当前解i,，
       %r用变异解替换当前解i并重置追踪计数器*/
       if (FitnessSol>Fitness(index(i))) 
            Foods(index(i),:)=sol;
            Fitness(index(i))=FitnessSol;
            ObjVal(index(i))=ObjValSol;
            trial(index(i))=0;
        else
            trial(index(i))=trial(index(i))+1; %/*如果当前解i不能被改进, 将追踪技术器的值加1*/
       end;       
    end;

    %%%%%%%%%%%%%%%%%%%%%%%% 概率计算%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %/* 食物被选择的概率和食物的质量对应成比例*/
    %/*可以采用不同的方案来计算概率值大小*/
    %/*举个例子 prob(i)=fitness(i)/sum(fitness)*/ 
    %/*或者采用偏离下界的程度来衡量 prob(i)=a*fitness(i)/max(fitness)+b*/

    prob=(0.9.*Fitness./max(Fitness))+0.1;
  
    %%%%%%%%%%%%%%%%%%%%%%%% 跟随蜂%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    i=1;
    t=0;
    while(t<FoodNumber)
        if(rand<prob(i))
            t=t+1;
            %/*待改变参数由随机函数生成*/
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

    % new 最优解稳定超过n次（n为最大迭代次数除以4），进行最优解随机变异 \(＾o＾)
    % size为变异种群大小
    size = 10;
    if (times >= Max_iter/4) || (mod(iter,1000) == 0)
        can = zeros(size+1,dim);
        sol=Foods(ind,:);
         % new 随机在最优解周围撒10个点 (＾o＾)/
        for f = 1:size
            % new 随机取改变的维度 \(＾o＾)
            Param2Change=fix(rand*D)+1;
            % new 避免越界的操作 (＾o＾)/
            if (ub - Foods(ind,Param2Change) <= Foods(ind,Param2Change) - lb)
                minx = ub - Foods(ind,Param2Change);
            else
                minx = Foods(ind,Param2Change) - lb;
            end
            % new 随机取偏移量 \(＾o＾)
            a = (rand - 0.5) * 2 * 0.5;
            a = minx * a;
            % new 获取一个随机撒点解 (＾o＾)/
            cand = sol;
            cand(Param2Change) = Foods(ind,Param2Change) + a(1);
            for (d = 1:dim)
                can(f,d) = cand(d);
            end
        end
        % new 将原解放入数组的第n+1个位置 \(＾o＾)
        for d = 1:dim
            can(size+1,d) = Foods(ind,d);
        end
        minx = 99999;
        % new 找出10个随机撒点解和原解之间的最优解 (＾o＾)/
        for d = 1:size+1
            re = feval(objfun,can(d));
            if (re < minx)
                minx = re;
                mini = d;
            end
        end
        % new 更新蜂源，如果未变，该蜜源不变次数加一，若改变，更新蜜源 \(＾o＾)
        if (mini == size+1)
            trial(ind) = trial(ind) + 1;
        else
            Foods(ind,:)=can(mini);
            Fitness(ind)=calculateFitness(minx);
            ObjVal(ind)=minx;
            trial(ind)=0;
        end
        % new 稳定次数清零 (＾o＾)/
        times = 0;
        % new 叠加回滚值 \(＾o＾)
        numm = numm + 1;
    end     
    %%%%%%%%%%%%侦查蜂%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
end % 蜂群算法结束

GlobalMins(r)=GlobalMin;
end; %运行过程结束
GloblaMin = min(GlobalMins)
endtime=cputime;
time=endtime-starttime;
end