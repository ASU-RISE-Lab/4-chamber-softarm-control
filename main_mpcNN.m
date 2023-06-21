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
%%
% 
% % find the state trajectory
% 
% xfs = zeros(ns+1,4);
% z0 = q0';
% xfs(1,:) = q0';
% dt = 0.001;
% for ks = 1:ns
% %     [~,zs] = ode15s(@(t,x)state(t,x,wopt,ks),[ts(ks),ts(ks+1)], z0, optODE);
% %     z0 = zs(end,:)';
%     for j = ts(ks):dt:ts(ks+1)
%         state_next = sim(net,[x(1:4);us']);%10x1
%         dx = [(state_next(1:4) - x(1:4))*60 ;%10x1
%         x1 = x1 + (x2 + wopt(ks))*dt;
%         x2 = x2 + (-wopt(ks))*dt;
%     end
%     xfs(ks+1,:) = [x1 x2];
%     ks;
% end
% % plot the control input and states, here position and velocity
% plots
    