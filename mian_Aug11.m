%% mpc main
clc;close all;clear all;
%% Initialize the system
par_set=[];
%flag for EOM deriviation
par_set.EOM=0;
%flag for plot
par_set.flag_plot_rawData =0;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;

%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 0;
% Check data readme.txt for detail input reference
par_set.Ts=1/30;

par_set.fz_a0 = (25/1000)*(60/1000);%m^2
par_set.tau_l0 =48/1000;%m

par_set.R1_stand_off = 0.05;% m
par_set.train_ratio = 1.0;
% par_set.R1_stand_off = 0.03;% m
fprintf('System initialization done \n')
%% ode fix 1 seg 3 link
par_set.EOM=1;
par_set = funcEOMbaseFrame1seg_v3(par_set);

%% Read txt file or mat file
if par_set.flag_read_exp==1
    for i = 1:11
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
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
%% Forward Kinematics
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
fkResult = funcCompuFK_v2(mocapResult.state_array,par_set.Ti);
close all
figure(1)
subplot(3,1,1)
plot(fkResult.camFrameE1(:,1),'r')
hold on
plot(fkResult.camFrameE1(:,2),'b')
hold on
plot(fkResult.camFrameE1(:,3),'k')
hold on

plot(testData.rigid_2_pose(:,1),'g--')
hold on
plot(testData.rigid_2_pose(:,2),'y--')
hold on
plot(testData.rigid_2_pose(:,3),'c--')
hold on
title('FK result for Endeffector 1 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)
plot(mocapResult.state_array(:,1))
hold on
legend('theta1')
subplot(3,1,3)
plot(testData.pd_psi(:,1))
hold on
plot(testData.pd_psi(:,2))
hold on
plot(testData.pd_psi(:,3))
hold on
legend('1','2','3')
figure(2)
subplot(2,1,1)
plot(fkResult.camFrameE2(:,1),'r')
hold on
plot(fkResult.camFrameE2(:,2),'b')
hold on
plot(fkResult.camFrameE2(:,3),'k')
hold on

plot(testData.rigid_3_pose(:,1),'g--')
hold on
plot(testData.rigid_3_pose(:,2),'y--')
hold on
plot(testData.rigid_3_pose(:,3),'c--')
hold on
title('FK result for Endeffector 2 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(2,1,2)
plot(mocapResult.state_array(:,5))
hold on
legend('theta2')
%% least-square for k d term
close all
testData = par_set.trial1;
ls_data_prep = [];
ls_data_prep = funcKnownTerm_v7(testData,par_set);

figure
subplot(2,1,1)
plot(ls_data_prep.sum_mcgTauf2xn(1,:))
hold on
plot(ls_data_prep.state_array4xn(1,:))
hold on
plot(ls_data_prep.state_array4xn(2,:))
hold on
plot(ls_data_prep.sum_mcg2xn(1,:))
legend('y','x','dx')
subplot(2,1,2)
plot(ls_data_prep.sum_mcgTauf2xn(2,:))
hold on
plot(ls_data_prep.state_array4xn(3,:))
hold on
plot(ls_data_prep.state_array4xn(4,:))
hold on
plot(ls_data_prep.sum_mcg2xn(2,:))
legend('y','x','dx')
ytau = ls_data_prep.sum_mcgTauf2xn(1,:);
xtau = ls_data_prep.state_array4xn(1,:);
dxtau = ls_data_prep.state_array4xn(2,:);
yf = ls_data_prep.sum_mcgTauf2xn(2,:);
xf = ls_data_prep.state_array4xn(3,:);
dxf = ls_data_prep.state_array4xn(4,:);

% Result summary
% ytau = 0.03795 * theta (R^2 = 0.9989)
% yf = 6.414 * lc1 + 2.41* dc1 + -1.92
figure
plot(ls_data_prep.sum_mcgTauf2xn(2,:))
hold on
plot(6.414*xf)
legend('1','2')
%% Euler-lag simulation
MCG_result = funcCompuMCG_v1(mocapResult.state_array);
% rigidMCG_result = funcCompuRigidMCG_v2(mocapResult.state_array);
close all
figure(1)
subplot(2,1,1)
plot(MCG_result.detM)
% hold on
% plot(rigidMCG_result.detM)
% hold on 
% plot(rigidMCG_result.rankM)
subplot(2,1,2)
% plot(MCG_result.detC)
hold on
% plot(rigidMCG_result.detC)
%%
x=[];
t0 = 0; tfinal =10;
x0 = mocapResult.state_array(:,1:4);
x0(2:2:4) = 0;
% x0(1) =0;x0(5)=0;

[t,x] = ode45(@funcMCGode_v1,[t0 tfinal],x0);
close all
figure(2)
subplot(2,1,1)
plot(t,rad2deg(x(:,1)))
hold on
title('bending angle(deg)')
legend('1')
subplot(2,1,2)
hold on
plot(t,x(:,3))
title('lc(m)')
