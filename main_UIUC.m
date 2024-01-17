%%%
% Initialization
%%%
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
% fprintf( 'Loading... \n' );
% load('trainData4.mat','par_set');
% 
% fprintf( 'Data loaded \n' );

par_set.flag_read_exp =1;
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
%% Mean std for data
%%%
% Change line 28 for different cases
% Use Oct26 rmse asmc/new/nsmc for figure 
%%%
ctrl_flag =2;% 1:asmc 2:nsmc 3:inasmc
if par_set.flag_read_exp==1
    for i = 1:7
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
noload = [];
outputKnown = [];
testData = par_set.trial1;
outputKnown =funcComputeStateVar_v3(testData,par_set);
noload.exp1.angle_rad = outputKnown.state_array(:,5);
noload.exp1.pd_psi = testData.pd_psi(:,4:6);
noload.exp1.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial2;
outputKnown =funcComputeStateVar_v3(testData,par_set);
noload.exp2.angle_rad = outputKnown.state_array(:,5);
noload.exp2.pd_psi = testData.pd_psi(:,4:6);
noload.exp2.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial3;
outputKnown =funcComputeStateVar_v3(testData,par_set);
noload.exp3.angle_rad = outputKnown.state_array(:,5);
noload.exp3.pd_psi = testData.pd_psi(:,4:6);
noload.exp3.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial4;
outputKnown =funcComputeStateVar_v3(testData,par_set);
noload.exp4.angle_rad = outputKnown.state_array(:,5);
noload.exp4.pd_psi = testData.pd_psi(:,4:6);
noload.exp4.pm_psi = testData.pm_psi(:,4:6);


%%
load_400g = [];
outputKnown = [];
testData = par_set.trial1;
outputKnown =funcComputeStateVar_v3(testData,par_set);
load_400g.exp1.angle_rad = outputKnown.state_array(:,5);
load_400g.exp1.pd_psi = testData.pd_psi(:,4:6);
load_400g.exp1.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial2;
outputKnown =funcComputeStateVar_v3(testData,par_set);
load_400g.exp2.angle_rad = outputKnown.state_array(:,5);
load_400g.exp2.pd_psi = testData.pd_psi(:,4:6);
load_400g.exp2.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial3;
outputKnown =funcComputeStateVar_v3(testData,par_set);
load_400g.exp3.angle_rad = outputKnown.state_array(:,5);
load_400g.exp3.pd_psi = testData.pd_psi(:,4:6);
load_400g.exp3.pm_psi = testData.pm_psi(:,4:6);

outputKnown = [];
testData = par_set.trial4;
outputKnown =funcComputeStateVar_v3(testData,par_set);
load_400g.exp4.angle_rad = outputKnown.state_array(:,5);
load_400g.exp4.pd_psi = testData.pd_psi(:,4:6);
load_400g.exp4.pm_psi = testData.pm_psi(:,4:6);
%%
load('uiuc_bend_noload.mat','noload');
load('uiuc_bend_400g.mat','load_400g');

spt =600;ept = 700;
testData1 = [];testData2=[];
testData1 = noload.exp1;
testData2 = load_400g.exp1;
ext_tqr = -0.4*9.8*sin(testData1.angle_rad)*0.05;

close all
figure(1)
subplot(3,1,1)
plot(rad2deg(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))
subplot(3,1,2)
plot(rad2deg(testData1.angle_rad(spt:ept)));
hold on
plot(rad2deg(testData2.angle_rad(spt:ept)));
subplot(3,1,3)
plot(ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))

exp1.theta = testData1.angle_rad(spt:ept);
exp1.k_value = ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept));

testData1 = [];testData2=[];
testData1 = noload.exp2;
testData2 = load_400g.exp2;
ext_tqr = -0.4*9.8*sin(testData1.angle_rad)*0.05;
close all
figure(1)
subplot(3,1,1)
plot(rad2deg(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))
subplot(3,1,2)
plot(rad2deg(testData1.angle_rad(spt:ept)));
hold on
plot(rad2deg(testData2.angle_rad(spt:ept)));
subplot(3,1,3)
plot(ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))

exp2.theta = testData1.angle_rad(spt:ept);
exp2.k_value = ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept));

testData1 = [];testData2=[];
testData1 = noload.exp3;
testData2 = load_400g.exp3;
ext_tqr = -0.4*9.8*sin(testData1.angle_rad)*0.05;
close all
figure(1)
subplot(3,1,1)
plot(rad2deg(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))
subplot(3,1,2)
plot(rad2deg(testData1.angle_rad(spt:ept)));
hold on
plot(rad2deg(testData2.angle_rad(spt:ept)));
subplot(3,1,3)
plot(ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))

exp3.theta = testData1.angle_rad(spt:ept);
exp3.k_value = ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept));

testData1 = [];testData2=[];
testData1 = noload.exp4;
testData2 = load_400g.exp4;
ext_tqr = -0.4*9.8*sin(testData1.angle_rad)*0.05;
close all
figure(1)
subplot(3,1,1)
plot(rad2deg(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))
subplot(3,1,2)
plot(rad2deg(testData1.angle_rad(spt:ept)));
hold on
plot(rad2deg(testData2.angle_rad(spt:ept)));
subplot(3,1,3)
plot(ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept)))

exp4.theta = testData1.angle_rad(spt:ept);
exp4.k_value = ext_tqr(spt:ept)./(testData1.angle_rad(spt:ept) - testData2.angle_rad(spt:ept));
close all
%%
close all
figure(1)
scatter(rad2deg(exp1.theta),exp1.k_value)
hold on
scatter(rad2deg(exp2.theta),exp2.k_value)
hold on
scatter(rad2deg(exp3.theta),exp3.k_value)
hold on
scatter(rad2deg(exp4.theta),exp4.k_value)
hold on
xlabel('Angle(deg)')
ylabel('Stiffness (Nm/rad)')
%%
close all
figure(1)
subplot(2,1,1)
plot(outputKnown.state_array(:,1))
hold on
plot(outputKnown.state_array(:,3))
hold on
plot(outputKnown.state_array(:,5))
hold on
plot(outputKnown.state_array(:,7))
hold on
legend
subplot(2,1,2)
plot(testData.pd_psi(:,1))
hold on
plot(testData.pd_psi(:,2))
hold on
plot(testData.pd_psi(:,4))
hold on
plot(testData.pd_psi(:,5))
hold on
legend