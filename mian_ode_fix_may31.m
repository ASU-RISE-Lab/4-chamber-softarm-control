%% Greybox main
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
par_set = funcEOMbaseFrame1seg_v1(par_set);
%%
par_set = funcGitTest(par_set);
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
%% Forward Kinematics
testData = par_set.trial1;
mocapResult = funcComputeStateVar_v1(testData,par_set);
fkResult = funcCompuFK_v1(mocapResult.state_array,par_set.Ti);
close all
figure(1)
subplot(2,1,1)
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
subplot(2,1,2)
plot(mocapResult.state_array(:,1))
hold on
legend('theta1')

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