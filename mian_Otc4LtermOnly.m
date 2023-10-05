
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
fprintf( 'Loading... \n' );
load('trainData1.mat');
fprintf( 'Data loaded \n' );

par_set.Ts=1/30;
par_set.fz_a0 = (25/1000)*(60/1000);%m^2 contact area of pillow
par_set.tau_l0 =48/1000;%m distance between center of pillow to rotation axis
par_set.R1_stand_off = 0.05;% m
par_set.enco_volt_p0 = [1.0191    1.0408    1.0858    1.0750];% V wire encoder readings at p=0 psi;
par_set.r0 = 0.043;% m distance between left and right encoder wires
% par_set.R1_stand_off = 0.03;% m
fprintf('System initialization done \n')
%%% End %%%

%%% Calculate State variable from sensor readings%%%
testData = par_set.trial2;
% Get deltaL1*(t) = L1*(t,p>0)-L1*(p=0) Unit meters
s1.l_t = (testData.enco_volts(:,1)-par_set.enco_volt_p0(1))/5;
s1.r_t = (testData.enco_volts(:,2)-par_set.enco_volt_p0(2))/5;
% Get deltaL2*(t) = L2*(t,p>0)-L2*(p=0)-deltaL1* Unit meters
s2.l_t = (testData.enco_volts(:,4)-par_set.enco_volt_p0(4))/5-s1.l_t;
s2.r_t = (testData.enco_volts(:,3)-par_set.enco_volt_p0(3))/5-s1.r_t;
outputKnown.raw_wire_readings = [s1.l_t,s1.r_t,s2.l_t,s2.r_t];
% Get theta, deltaLc for each segment
s1.theta_wire_rad = (-s1.r_t + s1.l_t)/par_set.r0;
s1.l_wire_m = (s1.r_t + s1.l_t)/2;
s2.theta_wire_rad = (-s2.r_t + s2.l_t)/par_set.r0;
s2.l_wire_m = (s2.r_t + s2.l_t)/2;
outputKnown.state_wire = [s1.theta_wire_rad,s1.l_wire_m,s2.theta_wire_rad,s2.l_wire_m];
% Get torque force using pm measurement
outputKnown.u_pm_psi(:,1) = testData.pm_psi(:,2) + testData.pm_psi(:,3);
outputKnown.u_pm_psi(:,2) = testData.pm_psi(:,1) +testData.pm_psi(:,3);
outputKnown.u_pm_psi(:,3) = (testData.pm_psi(:,5) + testData.pm_psi(:,6));
outputKnown.u_pm_psi(:,4) = testData.pm_psi(:,4) + testData.pm_psi(:,6);
outputKnown.u_pm_pa = outputKnown.u_pm_psi * 6894.76; % from psi to pa
% Convert unit to Nm; N; Nm;N
outputKnown.u_pm_tf(:,1) = outputKnown.u_pm_pa(:,1) * par_set.fz_a0 ;
outputKnown.u_pm_tf(:,2) = outputKnown.u_pm_pa(:,2) * par_set.fz_a0;
outputKnown.u_pm_tf(:,3) = outputKnown.u_pm_pa(:,3) * par_set.fz_a0;
outputKnown.u_pm_tf(:,4) = outputKnown.u_pm_pa(:,4) * par_set.fz_a0;
fprintf('State Var estimation done \n')
%%% End %%%

%%
%%% Build Grey box model %%%
alpha = -0.9665; beta = 0.9698;% For dpm6x1 = alpha*eye(6)*pm6x1 + beta*pd6x1 
% spt=1;ept=800;
spt=1;ept=length(testData.enco_volts);
input_array=[];output_array=[];
output_array = [testData.pm_psi(spt:ept,:), outputKnown.raw_wire_readings(spt:ept,1:end)];
input_array = testData.pd_psi(spt:ept,:);

z = iddata(output_array,input_array,par_set.Ts,'Name','train');
FileName      = 'func1stWithPmDynLtermOnly';       % File describing the model structure.
Order         = [10 6 10];           % Model orders [ny nu nx].
ParName = {'k1';'k2';'k3';'k4';...
    'ko1';'ko2';'ko3';'ko4';...
    'd1';'d2';'d3';'d4';...
    'a1';'a2';'a3';'a4'};
ParUnit ={'N/m';'N/m';'N/m';'N/m';...
    'N';'N';'N';'N';...
'Ns/rad';'Ns/rad';'Ns/rad';'Ns/rad';...
    'none';'none';'none';'none';};
ParVal    ={20993; 19867.7;23824.3; 23670.9;... % Par initial values
    2093.65; 1951.15; 2309.74; 2227.23;...
    99719.5; 94343.2 ; 93506.2; 73528.2;...
    1;1;1;1};
InitialStates = output_array(1,:)';           % Initial initial states.
Ts            = 0;                 % Time-continuous system.

ParMin   = {eps(0);eps(0);eps(0);eps(0);...
    -Inf;-Inf;-Inf;-Inf;...
    eps(0);eps(0);eps(0);eps(0);...
    eps(0);eps(0);eps(0);eps(0);};
ParMax   = {Inf;Inf;Inf;Inf;...
    Inf;Inf;Inf;Inf;...
    Inf;Inf;Inf;Inf;...
    Inf;Inf;Inf;Inf;};   
ParFix = {0; 0; 0; 0;...
    0; 0; 0; 0;...
    0; 0; 0; 0;...
    1;1;1;1;};
Parameters = struct('Name', ParName, 'Unit', ParUnit,'Value',ParVal,'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', ParFix);
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'og');
present(nlgr)
close all
compare(nlgr,z);
%%% End %%%
%%
%%% Grey box estimation %%%
opt = nlgreyestOptions('Display', 'off');
nlgr_ref = nlgreyest(z, nlgr, opt);
nlgr_ref.Name = 'refined';
%%% End %%%
%%
%%% Present Result %%%
close all
compare(nlgr_ref,nlgr,z);
present(nlgr_ref)
%%% End %%%
%%
%%% RK4 simulation %%%
x_pred = [];
h=1/30
x10x1 = [testData.pm_psi(1,:),outputKnown.raw_wire_readings(1,1:end)]';
for i = 1:length(testData.pm_psi)
    u6x1 = testData.pd_psi(i,:)';
    x_pred(i,:) = funcRK4fullODELtermOnly1_m(x10x1,u6x1,h,par_set);
    x10x1 = x_pred(i,:)';
end
% Plot result
close all
figure(1)
for i  = 1:6
    subplot(6,1,i)
    plot(testData.time_stamp, testData.pm_psi(:,i),'k')
    hold on
    plot(testData.time_stamp,x_pred(:,i),'r--')
    hold on
    ylim([0 20])
    if i ==1
        legend('exp','rk4')
        title('rk4 pm sim')
    end
    hold on
end

figure(2)
for i  = 1:4
    subplot(4,1,i)
    plot(testData.time_stamp,outputKnown.state_wire(:,i))
    hold on
    plot(testData.time_stamp,x_pred(:,i+6))
    hold on
    % ylim([0 20])
    if i ==1
        legend('exp','rk4')
        title('rk4 state sim')
    end
    hold on
end
figure(3)
subplot(3,1,1)
plot(outputKnown.state_wire(:,1))
legend('theta')
subplot(3,1,2)
plot(outputKnown.state_wire(:,2),'k')
hold on
plot(outputKnown.raw_wire_readings(:,1),'b--')
hold on
plot(outputKnown.raw_wire_readings(:,2),'r--')
legend('lc1','L1left','L1right')
subplot(3,1,3)
plot(testData.pm_psi(:,1),'r')
hold on
plot(testData.pm_psi(:,2),'b')
hold on
plot(testData.pm_psi(:,1)+testData.pm_psi(:,2),'r')
legend('pm1right','pm1left')
%%% End %%%



