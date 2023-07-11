%% load 
load("data_nnModel.mat");
%% x pm u
nn_pred =[]; nn_resp =[];temp_a = [];
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
temp_a(1,:) = zeros(1,4);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,1:2:end);
nn_pred = [temp_a,testData.pd_psi];
nn_resp = [mocapResult.state_array(:,1:2:end),testData.pm_psi];

testData = par_set.trial1;
mocapResult = funcComputeStateVar_v1(testData,par_set);temp_a(1,:) = zeros(1,4);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,1:2:end);
val_pred = [temp_a,testData.pd_psi];
val_resp = [mocapResult.state_array(:,1:2:end),testData.pm_psi];
%%% chose diff mode
obj_test = nnMod3_1.Network;
yn = sim(obj_test,val_pred');
close all
figure(1)
title1 = {'theta1','l1','theta2','l2'};
for i =1:4
subplot(4,1,i)
plot(yn(i,:))
hold on
plot(val_resp(:,i))
legend('sim','tru')
title(title1{i})
end
figure(2)
for i =1:6
subplot(6,1,i)
plot(yn(i+4,:))
hold on
plot(val_resp(:,i+4))
legend('sim','tru')
end
title('pm')

% load nnMod and meeting for data test
% time horizon and initial stateclear all
clc
close all
format long;

%% options for NLP solvers
optNLP = optimset('LargeScale','off','GradObj','off','GradConstr','off',...
                    'DerivativeCheck','off','Display','iter','TolX',1e-9,...
                    'TolFun',1e-9,'TolCon',1e-9,'MaxFunEval',100,...
                    'DiffMinChange',1e-5);

t0 = 0;
tf = 10;


ns = 3;
ts = [t0:(tf-t0)/ns:tf];
x_ref = nn_pred(500,1:4);
u0 = nn_pred(2,5)*ones(1,ns*6);
q0 = nn_pred(2,1:4);
A = []; b =[]; Aeq =[]; beq =[];nonlcon =[];
lb = [0*ones(1,6*ns) ];
ub = [40*ones(1,6*ns)];
net = nnMod3_1.Network;
x0 = [q0';u0(1:6)'];

[uOpt] = fmincon(@(us)func_obj(x0,ns,ts,us,net,x_ref),u0,A,b,Aeq,beq,lb,ub,nonlcon,optNLP);
uk =reshape(uOpt,[6,ns]);