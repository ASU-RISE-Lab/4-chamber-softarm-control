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
par_set.offset_mount = 30; %mm;
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
return
%% plot raw data
testData = par_set.trial1;
figure(1)
subplot(2,1,1)
plot(testData.pd_psi(:,1),testData.pd_psi(:,2),'r')
hold on
plot(testData.pd_psi(:,1),testData.pd_psi(:,3),'b')
hold on
plot(testData.pd_psi(:,1),testData.pd_psi(:,4),'g')
hold on
legend("p1","p2","p3")
ylabel('psi')
ylim([-1 21])
subplot(2,1,2)
plot(testData.tip_exp(:,1),testData.tip_exp(:,2))
hold on
plot(testData.tip_exp(:,1),testData.tip_exp(:,3))
hold on
plot(testData.tip_exp(:,1),testData.tip_exp(:,4))
legend("x",'y','z')
ylabel('meter')
xlabel('second')
%% ground mount
save('p32_1.mat','testData');
%% offset mount
save('np33_1.mat','testData');
%% combine diff figures
t0 = load('p30_1.mat');
t1 = load('p31_1.mat');
t2 = load('p32_1.mat');
nt0 = load('np30_1.mat');
nt1 = load('np31_1.mat');
nt2 = load('np32_1.mat');
% t3 = load('np33_1.mat');
%%
figure(1)
testData = t0.testData;
subplot(3,1,1)
plot(testData.pd_psi(:,1),testData.pd_psi(:,2),'r')
hold on
plot(nt0.testData.pd_psi(:,1),nt0.testData.pd_psi(:,2),'b--')
hold on
legend("p1",'np1','Location','eastoutside')
ylabel('psi')
ylim([-1 21])
subplot(3,1,2)
plot(t0.testData.tip_exp(:,1),t0.testData.tip_exp(:,4)-0.016,'r')
hold on
plot(t1.testData.tip_exp(:,1),t1.testData.tip_exp(:,4)-0.016,'b')
hold on
plot(t2.testData.tip_exp(:,1),t2.testData.tip_exp(:,4)-0.016,'k')
hold on
plot(nt0.testData.tip_exp(:,1),nt0.testData.tip_exp(:,4),'r--')
hold on
plot(nt1.testData.tip_exp(:,1),nt1.testData.tip_exp(:,4),'b--')
hold on
plot(nt2.testData.tip_exp(:,1),nt2.testData.tip_exp(:,4),'k--')
hold on
% plot(t3.testData.tip_exp(:,1),t3.testData.tip_exp(:,4))
legend("z0psi",'z1psi','z2psi',"nz0psi",'nz1psi','nz2psi','Location','eastoutside')
%% elvation angle theta = 2 * sign(x) * atan(sqrt(x^2 + y^2)/z)
subplot(3,1,3)
testData = [];xx=[];yy=[];zz=[];
testData=t0.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'r')
hold on
testData = [];xx=[];yy=[];zz=[];
testData=t1.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'b')
hold on
testData = [];xx=[];yy=[];zz=[];
testData=t2.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'k')
hold on
testData = [];xx=[];yy=[];zz=[];
testData=nt0.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'r--')
hold on
testData = [];xx=[];yy=[];zz=[];
testData=nt1.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'b--')
hold on
testData = [];xx=[];yy=[];zz=[];
testData=nt2.testData;
xx = testData.tip_exp(:,2);
yy= testData.tip_exp(:,3);
zz = testData.tip_exp(:,4);
theta_deg = 2 * sign(xx).*atand(sqrt(xx.^2 + yy.^2)./zz);
plot(testData.tip_exp(:,1),theta_deg,'k--')
hold on
legend("0psi",'1psi','2psi',"n0psi",'n1psi','n2psi','Location','eastoutside')
ylabel('angle deg')
%%
figure(1)
testData = t0.testData;
subplot(3,1,1)
plot(testData.pd_psi(:,1),testData.pd_psi(:,2),'r')
hold on
legend("p1",'Location','eastoutside')
ylabel('psi')
ylim([-1 21])
subplot(3,1,2)
plot(t0.testData.tip_exp(:,1),t0.testData.tip_exp(:,2))
hold on
plot(t1.testData.tip_exp(:,1),t1.testData.tip_exp(:,2))
hold on
plot(t2.testData.tip_exp(:,1),t2.testData.tip_exp(:,2))
hold on
% plot(t3.testData.tip_exp(:,1),t3.testData.tip_exp(:,2))
legend("x0psi",'x1psi','x2psi','Location','eastoutside')
ylabel('X meter')
xlabel('second')
subplot(3,1,3)
plot(t0.testData.tip_exp(:,1),t0.testData.tip_exp(:,4))
hold on
plot(t1.testData.tip_exp(:,1),t1.testData.tip_exp(:,4))
hold on
plot(t2.testData.tip_exp(:,1),t2.testData.tip_exp(:,4))
hold on
% plot(t3.testData.tip_exp(:,1),t3.testData.tip_exp(:,4))
legend("z0psi",'z1psi','z2psi','Location','eastoutside')
ylabel('Z meter')
xlabel('second')
%%
figure(2)
plot(t0.testData.tip_exp(:,2),t0.testData.tip_exp(:,4))
hold on
plot(t1.testData.tip_exp(:,2),t1.testData.tip_exp(:,4))
hold on
plot(t2.testData.tip_exp(:,2),t2.testData.tip_exp(:,4))
hold on
% plot(t3.testData.tip_exp(:,2),t3.testData.tip_exp(:,4))
legend("z0psi",'z1psi','z2psi','Location','eastoutside')
ylabel('Z meter')
xlabel('X meter')
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