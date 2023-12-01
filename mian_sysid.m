
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
par_set.flag_read_exp=1;
par_set.Ts=1/30;
par_set.fz_a0 = (25/1000)*(60/1000);%m^2 contact area of pillow
par_set.tau_l0 =48/1000;%m distance between center of pillow to rotation axis
par_set.R1_stand_off = 0.05;% m
par_set.enco_volt_p0 = [1.4612    1.4975    1.8306    1.8261];% V wire encoder readings at mid p=2 psi Dec 1st;
% par_set.enco_volt_p0 = [1.4692    1.5103    1.8416    1.8475];% V wire encoder readings at mid p=2 psi;
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
% calculate par_set.enco_volt_p0 
testData = par_set.trial1;
spt = 1; ept = 600;
figure(1)
m1 = mean(testData.enco_volts(spt:ept,:),1);
testData = par_set.trial2;
spt = 1; ept = 600;
figure(1)
m2 = mean(testData.enco_volts(spt:ept,:),1);
(m1+m2)/2

%%
%%% Calculate State variable from sensor readings%%%
testData = par_set.trial1;
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
            outputKnown.state_vel(i,1) = (s1.l_t(i)-s1.l_t(i-1))/par_set.Ts; 
            outputKnown.state_vel(i,2) = (s1.r_t(i)-s1.r_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,3) = (s2.l_t(i)-s2.l_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,4) = (s2.r_t(i)-s2.r_t(i-1))/par_set.Ts; 
        end
    end
    for i = 1:length(s1.l_t)
        if i ==1
            outputKnown.state_acc(i,1) = 0; 
            outputKnown.state_acc(i,2) = 0; 
            outputKnown.state_acc(i,3) = 0; 
            outputKnown.state_acc(i,4) = 0; 


        else
            outputKnown.state_acc(i,1) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1,1))/par_set.Ts; 
            outputKnown.state_acc(i,2) = (outputKnown.state_vel(i,2)-outputKnown.state_vel(i-1,2))/par_set.Ts;
            outputKnown.state_acc(i,3) = (outputKnown.state_vel(i,3)-outputKnown.state_vel(i-1,3))/par_set.Ts;
            outputKnown.state_acc(i,4) = (outputKnown.state_vel(i,4)-outputKnown.state_vel(i-1,4))/par_set.Ts; 
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
%%% Get Mddtdq %%%
% for i  = 1:length(s1.l_t)
%     Mati = [];
%     [Mmati,Ci,Gi] = funcMCGcalv3(outputKnown.arc_state_wire);
%     outputKnown.Mddtdq(i,:) = Mmati*outputKnown.arc_acc_wire(i,:)'+Ci*outputKnown.state_vel(i,:)';
%     outputKnown.MCG(i,:) = Mmati*outputKnown.arc_acc_wire(i,:)'+Ci*;
% end
% fprintf('State Var estimation done \n')
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
    hold on
    plot(outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i))
    hold on
end
%%
q1x =[];q1y=[];q1z=[];q2x =[];q2y=[];q2z=[];
q3x =[];q3y=[];q3z=[];q4x =[];q4y=[];q4z=[];
spt =1;ept =length(testData.enco_volts);
i=1
q1x = outputKnown.arc_state_wire(spt:ept,i);
q1y = outputKnown.arc_state_wire(spt:ept,i+4);
q1z = outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i);
i=2
q2x = outputKnown.arc_state_wire(spt:ept,i);
q2y = outputKnown.arc_state_wire(spt:ept,i+4);
q2z = outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i);
i=3
q3x = outputKnown.arc_state_wire(spt:ept,i);
q3y = outputKnown.arc_state_wire(spt:ept,i+4);
q3z = outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i);
i=4
q4x = outputKnown.arc_state_wire(spt:ept,i);
q4y = outputKnown.arc_state_wire(spt:ept,i+4);
q4z = outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i);
ft =fittype('poly11')
options = fitoptions(ft)
options.Lower = [0 0 0];
options.Upper = [0 Inf Inf];
fitobj1 = fit([q1x,q1y],q1z,ft,options)
fitobj2 = fit([q2x,q2y],q2z,ft,options)
fitobj3 = fit([q3x,q3y],q3z,ft,options)
fitobj4 = fit([q4x,q4y],q4z,ft,options)
pa_i = [fitobj1.p10,fitobj2.p10,fitobj3.p10,fitobj4.p10,fitobj1.p01,fitobj2.p01,fitobj3.p01,fitobj4.p01]; 
%%
j=1;%%
mean_coeff = mean(para_array);

%%
para_array(j,:) = pa_i;
j=j+1
%%
%%% gather data for training
gall=[];gnmall=[];
spt =1;ept =length(testData.enco_volts);
% spt =1;ept =;
q1all = iddata(outputKnown.raw_wire_readings(spt:ept,1),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q2all = iddata(outputKnown.raw_wire_readings(spt:ept,2),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q3all = iddata(outputKnown.raw_wire_readings(spt:ept,3),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q4all = iddata(outputKnown.raw_wire_readings(spt:ept,4),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
gall = iddata(outputKnown.raw_wire_readings(spt:ept,1:4),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1:4),par_set.Ts);
% gall2 = iddata(outputKnown.arc_state_wire(spt:ept,1:4),outputKnown.u_pm_psi(spt:ept,:),par_set.Ts);
% gnmall = iddata(outputKnown.arc_state_wire(spt:ept,1:8),outputKnown.u_pm_tf(spt:ept,:),par_set.Ts);
%%
spt =1;ept =length(testData.enco_volts);
q1i = iddata(outputKnown.arc_state_wire(spt:ept,1),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q2i = iddata(outputKnown.arc_state_wire(spt:ept,2),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q3i = iddata(outputKnown.arc_state_wire(spt:ept,3),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
q4i = iddata(outputKnown.arc_state_wire(spt:ept,4),outputKnown.u_pm_tf(:,1)-outputKnown.Mddtdq(:,1),par_set.Ts);
galli = iddata(outputKnown.arc_state_wire(spt:ept,1:4),outputKnown.u_pm_tf(:,1:4)-outputKnown.Mddtdq(:,1:4),par_set.Ts);

q1all = merge(q1all,q1i);
q2all = merge(q2all,q2i);
q3all = merge(q3all,q3i);
q4all = merge(q4all,q4i);
% 
gall  = merge(gall,galli);
% gnmall  = merge(gnmall,gnmi);
%%%%%%%
%% 
%%% Build full model no pm dynamics%%%
clc
spt =1;ept =length(testData.enco_volts);
input_array=[];output_array=[];z=[];nlgr=[];
% output_array = [outputKnown.arc_state_wire(spt:ept,1:2),outputKnown.arc_state_wire(spt:ept,5:6)];
% input_array = outputKnown.u_pm_tf(spt:ept,1:2);
% z = gall2;
z = gall;

FileName      = 'func_otc22_1st_u_minus_M';       % File describing the model structure.
Order         = [4 4 4];           % Model orders [ny nu nx].
ParName = {'ktheta1';'klc1';'ktheta2';'klc2';...

    'dtheta1';'dlc1';'dtheta2';'dlc2';};
% kk1 =31.66;kk2 =952;kk3 =30.3006;kk4 =1096.67;
% d1= 18.3847;d2= 2709.77;d3= 20.9183;d4= 3430.97;
ParVal    = {11;7985;13;9135;...
0.257;2629;0.0697;1228.9;};  
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