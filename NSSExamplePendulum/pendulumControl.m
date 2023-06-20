function [c, x0] = pendulumControl(nssName,TruthName,UseCT)
x0 = [1.5;1];
Ts = 0.1;
Steps = 80;
% open-loop simulation (torque = 0)
figure; 
subplot(2,1,1);
truthFcn = str2func(TruthName);
[~,x] = ode45(@(t,x) truthFcn(x,0), (0:Steps)*Ts, x0);
plot((0:Steps)*Ts,x,'.');
xlabel('time');
ylabel('states');
title('Open-Loop (state trajectory of neural state space model)')
% design a nonlinear mpc that bring theta and omega to zero quickly
c = nlmpcMultistage(10,2,1);
c.Ts = 0.1;
c.States(1).Min = -pi;
c.States(1).Max = pi;
c.ManipulatedVariables.Min = -2;
c.ManipulatedVariables.Max = 2;
for ct=2:11
    c.Stages(ct).CostFcn = "pendulumCostFcn";
end
% close-loop with true plant
c.Model.StateFcn = TruthName;
simdata = getSimulationData(c);
x = x0;
mv = 0;
subplot(2,1,2); hold on;
xlabel('time');
ylabel('states');
title('Closed-Loop (circle - Nonlinear MPC with Neural State Space Model)')
tic
for ct=0:Steps
    plot(ct*Ts,x,'.');
    drawnow
    [mv,simdata,info] = nlmpcmove(c,x,mv,simdata);
    [~,tmp] = ode45(@(t,x) truthFcn(x,mv), [0 Ts], x);
    x = tmp(end,:)';
    %[info.ExitFlag info.Iterations]
end
toc
% close-loop with neural ODE
c.Model.StateFcn = nssName;
c.Model.StateJacFcn = [nssName 'Jacobian'];
c.Model.IsContinuousTime = UseCT;
simdata = getSimulationData(c);
x = x0;
mv = 0;
tic
for ct=0:Steps
    plot(ct*Ts,x,'o');
    drawnow
    [mv,simdata,info] = nlmpcmove(c,x,mv,simdata);
    [~,tmp] = ode45(@(t,x) truthFcn(x,mv), [0 Ts], x);
    x = tmp(end,:)';
    %[info.ExitFlag info.Iterations]
end
toc


