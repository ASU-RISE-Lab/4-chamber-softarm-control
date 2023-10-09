
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
fprintf( 'Loading... \n' );
load('trainData3.mat');
fprintf( 'Data loaded \n' );


par_set.Ts=1/30;
par_set.fz_a0 = (25/1000)*(60/1000);%m^2 contact area of pillow
par_set.tau_l0 =48/1000;%m distance between center of pillow to rotation axis
par_set.R1_stand_off = 0.05;% m
% par_set.enco_volt_p0 = [1.0191    1.0408    1.0858    1.0750];% V wire encoder readings at mid p=0 psi;
% par_set.enco_volt_p0 = [1.2642    1.2977    1.6169    1.6009];% V wire encoder readings at mid p=1 psi;
par_set.enco_volt_p0 = [1.4692    1.5103    1.8416    1.8475];% V wire encoder readings at mid p=2 psi;
par_set.r0 = 0.043;% m distance between left and right encoder wires
% par_set.R1_stand_off = 0.03;% m
fprintf('System initialization done \n')
%%% End %%%
%% Read txt file or mat file
if par_set.flag_read_exp==1
    for i = 1:15
        par_set= funcLoadExp2Seg(par_set,i);
    end
    %     par_set= funcLoadExp2Seg(par_set,1);
    %     par_set= funcLoadExp2Seg(par_set,2);
    %     par_set= funcLoadExp2Seg(par_set,3);
    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
%%
%%% Check data raw readings
close all
clc
figure(1)
testData= par_set.trial1;
for i =1:4
subplot(2,2,i)
plot(testData.enco_volts(:,i))
hold on
ylabel('Encoder readings')
tstr = "encoder "+string(i);
title(tstr)
end
mean(testData.enco_volts(1:100,:),1)
testData= par_set.trial2;
for i =1:4
subplot(2,2,i)
plot(testData.enco_volts(:,i))
hold on
ylabel('Encoder readings')
end
mean(testData.enco_volts(1:100,:),1)
testData= par_set.trial4;
for i =1:4
subplot(2,2,i)
plot(testData.enco_volts(:,i))
hold on
ylabel('Encoder readings')
end
testData= par_set.trial5;
for i =1:4
subplot(2,2,i)
plot(testData.enco_volts(:,i))
hold on
ylabel('Encoder readings')
end
legend
mean(testData.enco_volts(1:100,:),1)
%%
%%% EOM using l term %%%
par_set = funcEOMbaseFrame2segwire_v1(par_set);
%%% END %%%

%%
%%% EOM using theta term %%%
par_set = funcEOMbaseFrame2seg_v5(par_set);
%%% END %%%
%%
%%% Check theta and delta lc close to zero %%%
syms theta1 theta2 lc1 lc2
Bqtheta1limit =limit(par_set.B_q,theta1,0);
Bqtheta2limit = limit(par_set.B_q,theta2,0);
Bqtotallimit = limit(Bqtheta1limit,theta2,0);

Cqtheta1limit =limit(par_set.C_q,theta1,0);
Cqtheta2limit = limit(par_set.C_q,theta2,0);
Cqtotallimit = limit(Cqtheta1limit,theta2,0);


Gqtheta1limit =limit(par_set.G_q,theta1,0);
Gqtheta2limit = limit(par_set.G_q,theta2,0);
Gqtotallimit = limit(Gqtheta1limit,theta2,0);
%%
%%% Check lii close to zero %%
syms l11 l12 l21 l22 
Bql11limit =limit(par_set.B_q,l11,0);
Bql12limit =limit(par_set.B_q,l12,0);


Bql21limit =limit(par_set.B_q,l21,0);
Bql22limit =limit(par_set.B_q,l22,0);

BqSeg1limit =limit(Bql11limit,l12,0);
BqSeg2limit =limit(Bql21limit,l22,0);

BqFulllimit =limit(BqSeg1limit,l21,0);
BqFulllimit =limit(BqFulllimit,l22,0)
det(BqFulllimit)
% rank =3 
%%% End %%%
%%
%%% Calculate State variable from sensor readings%%%
testData = par_set.trial6;
outputKnown = [];
% Get deltaL1*(t) = L1*(t,p>0)-L1*(p=0) Unit meters
s1.l_t = (testData.enco_volts(:,1)-par_set.enco_volt_p0(1))/5;
s1.r_t = (testData.enco_volts(:,2)-par_set.enco_volt_p0(2))/5;
% Get deltaL2*(t) = L2*(t,p>0)-L2*(p=0)-deltaL1* Unit meters
s2.l_t = (testData.enco_volts(:,4)-par_set.enco_volt_p0(4))/5-s1.l_t;
s2.r_t = (testData.enco_volts(:,3)-par_set.enco_volt_p0(3))/5-s1.r_t;
outputKnown.raw_wire_readings = [s1.l_t,s1.r_t,s2.l_t,s2.r_t];
% Get wire velocity readings
    for i = 1:length(s1.l_t)
        if i ==1
            outputKnown.state_vel(i,1) = 0; 
            outputKnown.state_vel(i,2) = 0; 
            outputKnown.state_vel(i,3) = 0; 
            outputKnown.state_vel(i,4) = 0; 


        else
            outputKnown.state_vel(i,1) = (s1.l_t(i,1)-s1.l_t(i-1))/par_set.Ts; 
            outputKnown.state_vel(i,2) = (s1.r_t(i,1)-s1.r_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,3) = (s2.l_t(i,1)-s2.l_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,4) = (s2.r_t(i,1)-s2.r_t(i-1))/par_set.Ts; 
        end
    end
    for i = 1:length(s1.l_t)
        if i ==1
            outputKnown.state_acc(i,1) = 0; 
            outputKnown.state_acc(i,2) = 0; 
            outputKnown.state_acc(i,3) = 0; 
            outputKnown.state_acc(i,4) = 0; 


        else
            outputKnown.state_acc(i,1) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1))/par_set.Ts; 
            outputKnown.state_acc(i,2) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1))/par_set.Ts;
            outputKnown.state_acc(i,3) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1))/par_set.Ts;
            outputKnown.state_acc(i,4) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1))/par_set.Ts; 
        end
    end
% Get theta, deltaLc for each segment
s1.theta_wire_rad = (-s1.r_t + s1.l_t)/par_set.r0;
s1.l_wire_m = (s1.r_t + s1.l_t)/2;
s2.theta_wire_rad = (-s2.r_t + s2.l_t)/par_set.r0;
s2.l_wire_m = (s2.r_t + s2.l_t)/2;
s1.dtheta_rad = (outputKnown.state_vel(:,1)-outputKnown.state_vel(:,2))/par_set.r0;
s1.dl_m = (outputKnown.state_vel(:,1)+outputKnown.state_vel(:,2))/2;
s2.dtheta_rad = (outputKnown.state_vel(:,3)-outputKnown.state_vel(:,4))/par_set.r0;
s2.dl_m = (outputKnown.state_vel(:,3)+outputKnown.state_vel(:,4))/2;
outputKnown.arc_state_wire = [s1.theta_wire_rad,s1.l_wire_m,s2.theta_wire_rad,s2.l_wire_m,s1.dtheta_rad,s1.dl_m,s2.dtheta_rad,s2.dl_m];
outputKnown.arc_acc_wire = [(outputKnown.state_acc(:,1)-outputKnown.state_acc(:,2))/par_set.r0,...
    (outputKnown.state_vel(:,1)+outputKnown.state_vel(:,2))/2,...
    (outputKnown.state_vel(:,3)-outputKnown.state_vel(:,4))/par_set.r0,...
    (outputKnown.state_vel(:,3)+outputKnown.state_vel(:,4))/2];
% Get torque force using pm measurement
outputKnown.u_pm_psi(:,1) = testData.pm_psi(:,2) - testData.pm_psi(:,1);
outputKnown.u_pm_psi(:,2) = testData.pm_psi(:,2) + testData.pm_psi(:,1);
outputKnown.u_pm_psi(:,3) = (testData.pm_psi(:,5) - testData.pm_psi(:,4));
outputKnown.u_pm_psi(:,4) = testData.pm_psi(:,5) + testData.pm_psi(:,4);
outputKnown.u_pm_pa = outputKnown.u_pm_psi * 6894.76; % from psi to pa
% Convert unit to Nm; N; Nm;N
outputKnown.u_pm_tf(:,1) = outputKnown.u_pm_pa(:,1) * par_set.fz_a0* par_set.tau_l0 ;
outputKnown.u_pm_tf(:,2) = outputKnown.u_pm_pa(:,2) * par_set.fz_a0;
outputKnown.u_pm_tf(:,3) = outputKnown.u_pm_pa(:,3) * par_set.fz_a0* par_set.tau_l0;
outputKnown.u_pm_tf(:,4) = outputKnown.u_pm_pa(:,4) * par_set.fz_a0;
a1=mean((testData.rigid_1_pose(:,3)-testData.rigid_2_pose(:,3)-par_set.R1_stand_off)/2);
a2=mean((testData.rigid_2_pose(:,3)-testData.rigid_3_pose(:,3))/2);
fprintf('State Var estimation done \n')
%%% End %%%
close all
figure(1)
for i =1:4
    subplot(4,1,i)
    yyaxis left
    plot(outputKnown.arc_state_wire(:,i))
    hold on
    yyaxis right
    plot(outputKnown.u_pm_tf(:,i))
end
%%
%%% gather data for training
gall=[];gnmall=[];
spt =1;ept =length(testData.enco_volts);
q1all = iddata(outputKnown.arc_state_wire(spt:ept,1),outputKnown.u_pm_psi(spt:ept,1),par_set.Ts);
q2all = iddata(outputKnown.arc_state_wire(spt:ept,2),outputKnown.u_pm_psi(spt:ept,2),par_set.Ts);
q3all = iddata(outputKnown.arc_state_wire(spt:ept,3),outputKnown.u_pm_psi(spt:ept,3),par_set.Ts);
q4all = iddata(outputKnown.arc_state_wire(spt:ept,4),outputKnown.u_pm_psi(spt:ept,4),par_set.Ts);

gall2 = iddata(outputKnown.arc_state_wire(spt:ept,1:4),outputKnown.u_pm_tf(spt:ept,:),par_set.Ts);
gnmall = iddata(outputKnown.arc_state_wire(spt:ept,1:8),outputKnown.u_pm_tf(spt:ept,:),par_set.Ts);
%%
spt =1;ept =length(testData.enco_volts);
q1i = iddata(outputKnown.arc_state_wire(spt:ept,1),outputKnown.u_pm_psi(spt:ept,1),par_set.Ts);
q2i = iddata(outputKnown.arc_state_wire(spt:ept,2),outputKnown.u_pm_psi(spt:ept,2),par_set.Ts);
q3i = iddata(outputKnown.arc_state_wire(spt:ept,3),outputKnown.u_pm_psi(spt:ept,3),par_set.Ts);
q4i = iddata(outputKnown.arc_state_wire(spt:ept,4),outputKnown.u_pm_psi(spt:ept,4),par_set.Ts);
gi = iddata(outputKnown.arc_state_wire(spt:ept,1:4),outputKnown.u_pm_tf(spt:ept,:),par_set.Ts);
gnmi = iddata(outputKnown.arc_state_wire(spt:ept,1:8),outputKnown.u_pm_tf(spt:ept,:),par_set.Ts);

q1all = merge(q1all,q1i);
q2all = merge(q2all,q2i);
q3all = merge(q3all,q3i);
q4all = merge(q4all,q4i);

gall2  = merge(gall2,gi);
gnmall  = merge(gnmall,gnmi);
%% 
%%% system id 1st order
spt =1;ept =length(testData.enco_volts);
z1 = iddata(outputKnown.arc_state_wire(spt:ept,1),outputKnown.u_pm_psi(spt:ept,1),par_set.Ts);
z2 = iddata(outputKnown.arc_state_wire(spt:ept,2),outputKnown.u_pm_psi(spt:ept,2),par_set.Ts);
z3 = iddata(outputKnown.arc_state_wire(spt:ept,3),outputKnown.u_pm_psi(spt:ept,3),par_set.Ts);
z4 = iddata(outputKnown.arc_state_wire(spt:ept,4),outputKnown.u_pm_psi(spt:ept,4),par_set.Ts);
%%
spt =1;ept =length(testData.enco_volts);
v1 = iddata(outputKnown.arc_state_wire(spt:ept,1),outputKnown.u_pm_psi(spt:ept,1),par_set.Ts);
v2 = iddata(outputKnown.arc_state_wire(spt:ept,2),outputKnown.u_pm_psi(spt:ept,2),par_set.Ts);
v3 = iddata(outputKnown.arc_state_wire(spt:ept,3),outputKnown.u_pm_psi(spt:ept,3),par_set.Ts);
v4 = iddata(outputKnown.arc_state_wire(spt:ept,4),outputKnown.u_pm_psi(spt:ept,4),par_set.Ts);
%%
%%% M mat verification %%%
M4x4 ={};detM=[];Mq=[];Gq=[];
xx = outputKnown.arc_state_wire;
ddq = outputKnown.arc_acc_wire;
for i = 1:length(xx)
[M4x4i,Gi,detMi] = funcMCGcalv2(xx(i,:));
M4x4{i} = M4x4i;
Mq(:,i) = M4x4i*ddq(i,:)';
Gq(:,i) = Gi;
invM{i} = inv(M4x4i);
detM(i) = detMi;
end
close all
figure(1)
subplot(4,1,1)
plot(detM)
subplot(4,1,2)
for i =1:2
plot(outputKnown.arc_state_wire(:,2*i-1))
hold on
legend('theta1','theta2')
end
subplot(4,1,3)
for i =1:2
plot(outputKnown.arc_state_wire(:,2*i))
hold on
legend('lc1','lc2')
end
subplot(4,1,4)
for i =1:2
yyaxis left
plot(outputKnown.u_pm_tf(:,2*i-1))
hold on
yyaxis right
plot(outputKnown.u_pm_tf(:,2*i))

legend('tau1','tau2','f1','f2')
end
%%%End%%%
%%
%%% Linear fit
spt=1;ept=length(testData.enco_volts)-900;
i=1
    fit1x = outputKnown.arc_state_wire(:,i);
    fit1y = outputKnown.arc_state_wire(:,i+4)';
    fit1z =(outputKnown.u_pm_tf(:,i)'-Mq(i,:)-Gq(i,:));

    i=2
    fit2x = outputKnown.arc_state_wire(spt:ept,i);
    fit2y = outputKnown.arc_state_wire(spt:ept,i+4)';
    fit2z =(outputKnown.u_pm_tf(spt:ept,i)'-Mq(i,spt:ept));
    i=2
    wirefit2x = outputKnown.raw_wire_readings(spt:ept,i);
    wirefit2y = outputKnown.state_vel(spt:ept)';
    wirefit2z =(outputKnown.u_pm_psi(spt:ept,i)');
    %%
    %%% RK4 with mid p=2
x_pred = [];
h=1/30
spt =1;ept =length(testData.enco_volts);
x4x1 = [outputKnown.arc_state_wire(spt,1:4)]';
for i = spt:ept
    u4x1 = outputKnown.u_pm_psi(i,1:4)';
    x_pred(i,:) = funcRK4NoPmDynSegall_m(x4x1,u4x1,h,par_set);
    x4x1 = x_pred(i,:)';
end
%%%%%%% Plot result
close all
clc
ylabelvec={'theta1';'lc1';'theta2';'lc2'};
figure(1)
    for i =1:4
    subplot(4,1,i)
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(:,i))
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(:,i))
    hold on
    ylabel(ylabelvec{i})
    % ylim([0 20])
    if i ==1
        legend('exp','rk4')
        title('rk4 state sim')
    end
    hold on
    end
%% 
%%% Build full model no pm dynamics%%%
clc
spt =1;ept =length(testData.enco_volts);
input_array=[];output_array=[];z=[];
% output_array = [outputKnown.arc_state_wire(spt:ept,1:2),outputKnown.arc_state_wire(spt:ept,5:6)];
% input_array = outputKnown.u_pm_tf(spt:ept,1:2);
z = gall2;
FileName      = 'func1stNoPmUnitpsi';       % File describing the model structure.
Order         = [4 4 4];           % Model orders [ny nu nx].
ParName = {'ktheta1';'klc1';'ktheta2';'klc2';...
    'dtheta1';'dlc1';'dtheta2';'dlc2';};
ParVal    = {31.05;2667.2;20.7519;3373.2;...
18.1638;944.3;29.6254;1084.5;};  
ParUnit ={'N/rad';'N/m';'N/rad';'N/m';...
'Nms/rad';'Ns/m';'Nms/rad';'Ns/m';};% Initial parameters. Np = 3*4
InitialStates = zeros(4,1);           % Initial initial states.
Ts            = 0;                 % Time-continuous system.

ParMin   = {eps(0);eps(0);eps(0);eps(0);...
             eps(0);eps(0);eps(0);eps(0);};
ParMax   = {Inf;Inf;Inf;Inf;...
    Inf;Inf;Inf;Inf;};   % No maximum constraint.
ParFix = {0; 0;0; 0;...
    0; 0;0; 0;};
Parameters = struct('Name', ParName, 'Unit', ParUnit,'Value',ParVal,'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', ParFix);
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'nlgr');
present(nlgr)
close all
compare(nlgr,z)
%%
close all
opt = nlgreyestOptions('Display', 'on');
nlgr1z = nlgreyest(z, nlgr, opt);
figure
compare(nlgr1z,nlgr,z);
present(nlgr1z)
%%% END %%%
%%
close all
gi = iddata(outputKnown.arc_state_wire(spt:ept,1:4),outputKnown.u_pm_psi(spt:ept,:),par_set.Ts);
figure
nlgr1.OutputName={'theta1';'lc1';'theta2';'lc2'};
compare(nlgr1,gi);
present(nlgr1)

%%    
    %%% RK4 with mid p=2
x_pred = [];
h=1/30
spt =1;ept =length(testData.enco_volts);
x8x1 = [outputKnown.arc_state_wire(spt,1:8)]';
for i = spt:ept
    u4x1 = outputKnown.u_pm_tf(i,1:4)';
    x_pred(i,:) = funcRK4NoPmDyn2ndSegall_m(x8x1,u4x1,h,par_set);
    x8x1 = x_pred(i,:)';
end
%%%%%%% Plot result
close all
clc
ylabelvec={'theta1';'lc1';'theta2';'lc2'};
figure(1)
    for i =1:4
    subplot(4,1,i)
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(:,2*i-1))
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(:,2*i-1))
    hold on
    ylabel(ylabelvec{i})
    % ylim([0 20])
    if i ==1
        legend('exp','rk4')
        title('rk4 state sim')
    end
    hold on
    end
%% 
%%% Build full model no pm dynamics%%%
clc
spt =1;ept =length(testData.enco_volts);
input_array=[];output_array=[];z=[];
% output_array = [outputKnown.arc_state_wire(spt:ept,1:2),outputKnown.arc_state_wire(spt:ept,5:6)];
% input_array = outputKnown.u_pm_tf(spt:ept,1:2);
z = gnmall;
FileName      = 'func2ndNoPmUnitnm';       % File describing the model structure.
Order         = [8 4 8];           % Model orders [ny nu nx].
ParName = {'ktheta1';'klc1';'ktheta2';'klc2';...
    'dtheta1';'dlc1';'dtheta2';'dlc2';};
ParVal    = {31.05;2667.2;20.7519;3373.2;...
18.1638;944.3;29.6254;1084.5;};  
ParUnit ={'N/rad';'N/m';'N/rad';'N/m';...
'Nms/rad';'Ns/m';'Nms/rad';'Ns/m';};% Initial parameters. Np = 3*4
InitialStates = zeros(8,1);           % Initial initial states.
Ts            = 0;                 % Time-continuous system.

ParMin   = {eps(0);eps(0);eps(0);eps(0);...
             eps(0);eps(0);eps(0);eps(0);};
ParMax   = {Inf;Inf;Inf;Inf;...
    Inf;Inf;Inf;Inf;};   % No maximum constraint.
ParFix = {0; 0;0; 0;...
    0; 0;0; 0;};
Parameters = struct('Name', ParName, 'Unit', ParUnit,'Value',ParVal,'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', ParFix);
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'nlgr');
present(nlgr)
close all
% compare(nlgr,z)
%%
close all
opt = nlgreyestOptions('Display', 'on');
nlgr1 = nlgreyest(z, nlgr, opt);
figure
compare(nlgr1,z);
present(nlgr1)



