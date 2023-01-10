%% Main function use data in 2023
%%% Major chanages:
%%%
clear all
close all
clc
%% Initialize the system
par_set=[];
%flag for EOM deriviation
par_set.EOM=0;
%flag for plot
par_set.flag_plot_rawData =1;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;

%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 0;
% Check data readme.txt for detail input reference
par_set.Ts=1/30;

par_set.L=0.185;%actuator length
par_set.n=4;% # of joints for augmented rigid arm
par_set.m0=0.35;%kg segment weight
par_set.g=9.8;%% gravity constant
par_set.R1_stand_off = 0.05;% m
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set= funcLoadExp2Seg(par_set,1);
    par_set= funcLoadExp2Seg(par_set,2);
    par_set= funcLoadExp2Seg(par_set,3);

    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
%% PLot raw data
testData = par_set.trial2;
if par_set.flag_plot_rawData == 1
    funcPlotRawData(testData)
end
%% Calculate bending angle
% Encoder reading based theta = (si-r - si-l)/ r0

% Mocap based theta = 2 * (pi/2 - atan2(R2.pose.x - R1.pos.x,R1.pose.z -
% 0.05 - R2.pos.z))

testData = par_set.trial2;
par_set.R1_stand_off = 0.05;% m
% testData.theta_mocap = 2 * (pi/2 - atan2(testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1),...
%     testData.rigid_1_pose(:,3) - par_set.R1_stand_off - testData.rigid_2_pose(:,3))); 

testData.theta_mocap =2 * (atan2(testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1),...
    testData.rigid_1_pose(:,3) - par_set.R1_stand_off - testData.rigid_2_pose(:,3)));  
close all;
figure(1)
subplot(2,1,1)
plot(rad2deg(testData.theta_mocap))
ylabel('rad')
subplot(2,1,2)
plot((testData.enco_volts(:,2) - (testData.enco_volts(:,1))))
ylabel('m')
ylim([-0.1 0.1])
yy = testData.enco_volts(:,2) - testData.enco_volts(:,1);
xx = testData.theta_mocap;