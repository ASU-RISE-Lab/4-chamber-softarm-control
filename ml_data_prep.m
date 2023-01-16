%% Main function for stiffness ID use data 0722
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
par_set.flag_plot_rawData =0;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;
%flag for plotting moving constant layer
par_set.flag_plot_movingCC = 0;
%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 0;
% Check data readme.txt for detail input reference
par_set.Ts=1/40;
% Geometric para.
par_set.trianlge_length=70*1e-03;% fabric triangle edge length
par_set.L=0.185;%actuator length
par_set.n=4;% # of joints for augmented rigid arm
par_set.m0=0.35;%kg segment weight
par_set.g=9.8;%% gravity constant
par_set.a0=15*1e-03;%% 1/2 of pillow width
par_set.r_f=sqrt(3)/6*par_set.trianlge_length+par_set.a0; % we assume the force are evenly spread on a cirlce with radius of r_f
%% Update location of 3 chambers P1, P2, P3
par_set.p1_angle=150;%deg p1 position w/ the base frame
% update force position of p1 p2 and p3
for i =1:3
    par_set.r_p{i}=[par_set.r_f*cosd(par_set.p1_angle+120*(i-1)),par_set.r_f*sind(par_set.p1_angle+120*(i-1)),0].';
%     par_set.f_p{i}=588.31*par_set.pm_MPa(:,i+1);
end
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcHighLevelExpPositionTracking(par_set,1);
    par_set=funcHighLevelExpPositionTracking(par_set,2);
    par_set=funcHighLevelExpPositionTracking(par_set,3);
    par_set=funcHighLevelExpPositionTracking(par_set,4);
    par_set=funcHighLevelExpPositionTracking(par_set,5);
%     par_set=funcHighLevelExpPositionTracking(par_set,6);
%     par_set=funcHighLevelExpPositionTracking(par_set,7);
%     par_set=funcHighLevelExpPositionTracking(par_set,8);
    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
%%
testData= par_set.trial1;
s_pt=1;e_pt=length(testData.xd_exp(:,2));
testData = funcPostProcess(testData,s_pt,e_pt);

rmse=testData.rmse;
Ep=testData.inputEnergy;
testData= par_set.trial2;
s_pt=1;e_pt=length(testData.xd_exp(:,2));
testData = funcPostProcess(testData,s_pt,e_pt);
rmse=testData.rmse+rmse;
Ep=testData.inputEnergy+Ep;
testData= par_set.trial3;
s_pt=1;e_pt=length(testData.xd_exp(:,2));
testData = funcPostProcess(testData,s_pt,e_pt);
rmse=testData.rmse+rmse;
Ep=testData.inputEnergy+Ep;
par_set.flag_fixed_beta=0
testData = funcGetPhiThetaRifromXYZ(testData,par_set);
%%%%% 
rmse=rmse/3
Ep=Ep/3
%%%%%%
% testData= par_set.trial4;
% s_pt=1;e_pt=length(testData.xd_exp(:,2));
% testData = funcPostProcess(testData,s_pt,e_pt);
% rmse=testData.rmse+rmse;
% Ep=testData.inputEnergy+Ep;
% testData= par_set.trial5;
% s_pt=1;e_pt=length(testData.xd_exp(:,2));
% testData = funcPostProcess(testData,s_pt,e_pt);
% rmse=testData.rmse+rmse;
% Ep=testData.inputEnergy+Ep;
% rmse=rmse/5;
% Ep=Ep/5;
% return
%%
close all
testData= par_set.trial1;
fig_width=7;
fig_height=7/4;
%%% fig 1
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
plot(testData.xd_exp(:,2))
hold on
plot(testData.x1_exp(:,2))
s_pt=1;e_pt=length(testData.xd_exp(:,2));
%%% calcualte avg. pressure during contact
% p_s_pt=148;p_e_pt=822;
% p_avg=mean(testData.pm_MPa(p_s_pt:p_e_pt,2))
%%% fig 2
s_pt=1;e_pt=length(testData.xd_exp(:,2));
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
plot(testData.xd_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xd_exp(s_pt:e_pt,2),'r','LineStyle','-.','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xdNew(s_pt:e_pt),'b','LineStyle',':','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.x1_exp(s_pt:e_pt,2),'k','LineStyle','-','LineWidth',1)
ylabel('Angle (rad)')
xlabel('Time (second)')
xlim([0,50])
ylim([-0.7,-0.1])
leg=legend({'${\theta_d}$','$\theta_a$','$\theta$'},'Orientation','horizontal','Location','south','Units','inches','Interpreter','latex')
leg.ItemTokenSize = [20,20];
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;

%%% fig 3
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.dist_est_inner_tau(s_pt:e_pt),'r','LineWidth',2)
leg=legend('$\hat{\Delta}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
leg.ItemTokenSize = [20,20];
ylabel('Disturbance (Nm)')
xlabel('Time (second)')
% ylim([-0.2,2])
xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;

%%% fig 4
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.pm_MPa(s_pt:e_pt,2),'b','LineWidth',2)
leg=legend('$p_{m_1}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
leg.ItemTokenSize = [20,20];
ylabel('Air Pressrue (MPa)')
xlabel('Time (second)')
ylim([0,0.3])
xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
testData = funcPostProcess(testData,s_pt,e_pt);
return
%% animation
close all
% Animi 1
fig_width=7/2.8*2;
fig_height=7/4*2;
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
frame_speed =20;
curve1= animatedline('Color','r','LineStyle','-.','LineWidth',2);
curve2= animatedline('Color','b','LineStyle',':','LineWidth',2);
curve3= animatedline('Color','k','LineStyle','-','LineWidth',1);
set(gca,'Xlim',[0,50],'Ylim',[-0.7,-0.1],'Color','none')
ylabel('Angle (rad)')
xlabel('Time (second)')
leg=legend({'${\theta_d}$','$\theta_a$','$\theta$'},'Orientation','horizontal','Location','south','Units','inches','Interpreter','latex')
leg.ItemTokenSize = [20,20];
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;

obj = VideoWriter('smart_base_pos_1_2.avi');
obj.Quality = 100;
obj.FrameRate=40;
open(obj)
for cnt = 1:length(testData.xd_exp(:,2))
  addpoints(curve1,testData.xd_exp(cnt,1)-testData.xd_exp(1,1),testData.xd_exp(cnt,2))
  hold on
  addpoints(curve2,testData.x1_exp(cnt,1)-testData.xd_exp(1,1),testData.xdNew(cnt))
    hold on
   addpoints(curve3,testData.x1_exp(cnt,1)-testData.xd_exp(1,1),testData.x1_exp(cnt,2))
%   pause(par_set.Ts/frame_speed);
    f =getframe(gcf);
    writeVideo(obj,f);
end
obj.close();
return
%% Animation 2
close all
fig_width=7/2.8*2;
fig_height=7/4*2;
fp=figure('units','inches','Position',[1,1,fig_width,fig_height]);
ylabel('Air Pressrue (MPa)')
xlabel('Time (second)')
set(gca,'Xlim',[0,50],'Ylim',[0,0.3],'Color','none')
curve1=animatedline('Color','b','LineWidth',2);
leg=legend('$p_{m_1}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
leg.ItemTokenSize = [20,20];
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
obj = VideoWriter('smart_base_pm_1_2.avi');
obj.Quality = 100;
obj.FrameRate=40;
open(obj)
for cnt = 1:length(testData.xd_exp(:,2))
addpoints(curve1,testData.x1_exp(cnt,1)-testData.xd_exp(1,1),testData.pm_MPa(cnt,2))
% pause(par_set.Ts/frame_speed);
    f =getframe(gcf);
    writeVideo(obj,f);
end
obj.close();
%% Segment data4 smcndob
% close all
testData= par_set.trial2;
fp=figure('Name','ramp','Position',[100,100,600,800]);
plot(testData.xd_exp(:,2))
s_pt=1;e_pt=length(testData.xd_exp(:,2));
%%% fig 2
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.xd_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xd_exp(s_pt:e_pt,2),'r','LineStyle','-','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.x1_exp(s_pt:e_pt,2),'b','LineStyle','-.','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xdNew(s_pt:e_pt),'k','LineStyle',':','LineWidth',2)
ylabel('Angle (rad)')
xlabel('Time (second)')
% xlim([0,50])
ylim([-0.7,-0.1])
legend('\theta_d','\theta','\theta_a','Orientation','horizontal','Location','north')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;
%%% fig 3
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.dist_est_inner_tau(s_pt:e_pt),'r','LineWidth',2)
legend('$\hat{\Delta}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
ylabel('Disturbance (Nm)')
xlabel('Time (second)')
ylim([-0.2,2])
% xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;

%%% fig 4
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.pm_MPa(s_pt:e_pt,2),'k','LineWidth',2)
legend('$p_{m_1}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
ylabel('Air Pressrue (MPa)')
xlabel('Time (second)')
ylim([0,0.3])
% xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;
testData = funcPostProcess(testData,s_pt,e_pt);
%% segment data 5 smcndob with switch
close all

testData= par_set.trial1;
s_pt=1;e_pt=length(testData.xd_exp(:,2));
%%% fig 2
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.xd_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xd_exp(s_pt:e_pt,2),'r','LineStyle','-','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.x1_exp(s_pt:e_pt,2),'b','LineStyle','-.','LineWidth',2)
hold on
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.xdNew(s_pt:e_pt),'k','LineStyle',':','LineWidth',2)
ylabel('Angle (rad)')
xlabel('Time (second)')
xlim([0,50])
ylim([-0.7,-0.05])
legend('\theta_d','\theta','\theta_a','Orientation','horizontal','Location','north')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;
%%% fig 3
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.dist_est_inner_tau(s_pt:e_pt),'r','LineWidth',2)
legend('$\hat{\Delta}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
ylabel('Disturbance (Nm)')
xlabel('Time (second)')
ylim([-0.2,6])
xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;

%%% fig 4
fp=figure('Name','fig1','Position',[100,100,800,600]);
plot(testData.x1_exp(s_pt:e_pt,1)-testData.xd_exp(s_pt,1),testData.pm_MPa(s_pt:e_pt,2),'k','LineWidth',2)
legend('$p_{m_1}$','Interpreter','latex','Orientation','horizontal','Location','northeast')
ylabel('Air Pressrue (MPa)')
xlabel('Time (second)')
ylim([0,0.3])
xlim([0,50])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=20;
testData = funcPostProcess(testData,s_pt,e_pt);
%% Update 1/3/2022 save data into new set for later overlapped
smart=[];
smart=testData;
save('smart_base.mat','smart');
%%
smart=[];
smart=testData;
save('smart_wood.mat','smart');
%% ksf
smart=[];
smart=testData;
save('smart_ksf_base_10.mat','smart');
%%
smart=[];
smart=testData;
save('smart_ksf_low_01.mat','smart');
%%
smart=[];
smart=testData;
save('smart_ksf_low_50.mat','smart');
%%
smart=[];
smart=testData;
save('smart_ksf_high_100.mat','smart');
%% kw
smart=[];
smart=testData;
save('smart_kw_base_5.mat','smart');
%%
smart=[];
smart=testData;
save('smart_kw_low_1.mat','smart');
%%
smart=[];
smart=testData;
save('smart_kw_high_20.mat','smart');
%%
smart=[];
smart=testData;
save('smart_grape','smart');
%% eta
smart=[];
smart=testData;
save('smart_eta_base_10.mat','smart');
%%
smart=[];
smart=testData;
save('smart_eta_low_01.mat','smart');
%%
smart=[];
smart=testData;
save('smart_eta_high_100.mat','smart');
%% lambda
smart=[];
smart=testData;
save('smart_lambda_base_10.mat','smart');
%%
smart=[];
smart=testData;
save('smart_lambda_low_1.mat','smart');
%%
smart=[];
smart=testData;
save('smart_lambda_high_100.mat','smart');
%% Openloop sim
opensim=[];
opensim=testData;
save('opensim.mat','opensim');