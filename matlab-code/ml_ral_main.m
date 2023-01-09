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
testData = par_set.trial1;
if par_set.flag_plot_rawData == true
    close all;
    figure(1)
    title('xyz pos in cam frame')
    plot3(testData.rigid_1_pose(:,1),testData.rigid_1_pose(:,2),testData.rigid_1_pose(:,3),'r')
    hold on
    plot3(testData.rigid_2_pose(:,1),testData.rigid_2_pose(:,2),testData.rigid_2_pose(:,3),'b')
    hold on
    plot3(testData.rigid_3_pose(:,1),testData.rigid_3_pose(:,2),testData.rigid_3_pose(:,3),'k')
    hold on
    legend('R1','R2','R3')
end