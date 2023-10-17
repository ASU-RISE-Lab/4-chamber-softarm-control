%% EKF main
% q8x1 = [theta1,lc1,theta2,lc2,pm1,pm2,pm3,pm4]'; 
% u4x1_psi = [pd1,pd2,pd3,pd4]';
% kk1 = -1.767*u4x1(1)^2 + 17.55*abs(u4x1(1))+33.471;
% kk2 = 10.25*u4x1(2)^2 - 325.1*u4x1(2)+3299;
% kk3 =-1.013*u4x1(3)^2+11.55*abs(u4x1(3))+4.419;
% kk4 = 15.34*u4x1(4)^2 - 474.1*u4x1(4)+4475;
% d1= 0.9725*u4x1(1)^2- 11.2*abs(u4x1(1))+38.471;
% d2= -0.9725*u4x1(2)^2+ 30.23*u4x1(2)+435.471;
% d3=  0.1125*u4x1(3)^2- 1.2*abs(u4x1(3))+14.471;
% d4= 4.34*u4x1(4)^2 - 155.21*u4x1(4)+2146;
%u4x1(1,1) = -(pm11 - pm12);
%u4x1(2,1) = (pm11 + pm12);
%u4x1(3,1) = -(pm21 - pm22);
%u4x1(4,1) = (pm21 + pm22);
% Kmat = diag([kk1,kk2,kk3,kk4]);
% Dmat = diag([d1,d2,d3,d4]);
% pm11 = x10x1(1); pm12 = x10x1(2); pm13 = x10x1(3);
% pm21 = x10x1(4); pm22 = x10x1(5); pm23 = x10x1(6);

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

xold = zeros(8,1); uold = zeros(4,1); 
Pk_old = 1*eye(8);Pk_old2 = Pk_old;Pk_old3 =Pk_old;
Pk_old4 = 1*eye(12);
q_arc =10;r_arc =200;
Q = diag([q_arc*ones(4,1);ones(4,1)]) ;R = diag([r_arc*ones(4,1);10*ones(4,1)]); H = eye(8);
Q4 = diag([q_arc*ones(4,1);ones(4,1);q_arc*ones(4,1);]) ;R4 = diag([r_arc*ones(4,1);10*ones(4,1);]); H4 = [eye(8),zeros(8,4)];
dT = 1/1000;
e_max = 1.0; e_min = 0.2;
x_min=[-pi/4,0,-pi/4,0,zeros(1,4)];
x_max =[pi/4,0.025,pi/4,0.025,20,20,20,20];
spt =2;ept=length(testData.pd_psi);
xk_old=[];x_pred=[];x_pred2=[];x_pred3=[];
xk_old = [outputKnown.arc_state_wire(spt-1,1:4),testData.pm_psi(spt-1,1:2),testData.pm_psi(spt-1,4:5)];
xk_old2 =xk_old;
xk_old3 = xk_old;
xk_old4 = [xk_old3,zeros(1,4)];
x_pred = xk_old;
x_pred2 = xk_old2;
x_pred3 = xk_old3;
x_pred4 = xk_old4;
lamda = 1.1;
for i = spt:ept
    uk = [testData.pd_psi(i,1:2),testData.pd_psi(i,4:5)]';
    zk = [outputKnown.arc_state_wire(i,1:4),testData.pm_psi(i,1:2),testData.pm_psi(i,4:5)]';
[xk_est,Pk] =funcEKF_baseline(xk_old,uk,zk,Pk_old,Q,R,H,dT);
P_old = Pk;
x_pred(i,:)  = xk_est;
xk_old =xk_est;

% [xk_est2,Pk2] =funcEKF_improve(xk_old2,uk,zk,Pk_old2,Q,R,H,dT,e_max,e_min,x_min,x_max);
% P_old2 = Pk2;
% x_pred2(i,:)  = xk_est2;
% xk_old2 =xk_est2;

[xk_est3,Pk3] =funcEKF_robust(xk_old3,uk,zk,Pk_old3,Q,R,H,dT,lamda);
P_old3 = Pk3;
x_pred3(i,:)  = xk_est3;
xk_old3 =xk_est3;

[xk_est4,Pk4] =funcEKF_disturbance(xk_old4,uk,zk,Pk_old4,Q4,R4,H4,dT);
P_old4 = Pk4;
x_pred4(i,:)  = xk_est4;
xk_old4 =xk_est4;
end

close all
clc
ylabelvec={'theta1';'lc1';'theta2';'lc2';'pm11';'pm12';'pm21';'pm22';'d1';'d2';'d3';'d4'};
exp_arr = [outputKnown.arc_state_wire(:,1:4),testData.pm_psi(:,1:2),testData.pm_psi(:,4:5)];
figure(1)
hold on
    for i =1:8
    subplot(2,4,i)
    plot(testData.time_stamp(spt:ept),exp_arr(spt:ept,i),'r',LineWidth=2)
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(spt:ept,i),'b')
    hold on
%     plot(testData.time_stamp(spt:ept),x_pred2(spt:ept,i),'k')
%     hold on
        plot(testData.time_stamp(spt:ept),x_pred4(spt:ept,i),'g')
    hold on
    ylabel(ylabelvec{i})
    % ylim([0 20])
    hold on
    end
            legend('exp','ekf','ekf_robust')
       sgtitle("ekf state est"+ " P="+string(Pk_old(1,1))+" Q="+string(Q(1,1))+" R="+string(R(1,1))+" T="+string(dT))


 figure(3)
hold on
    for i =1:12
    subplot(3,4,i)
    if i <=8
    plot(testData.time_stamp(spt:ept),exp_arr(spt:ept,i),'r',LineWidth=2)
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(spt:ept,i),'b')
    hold on
%     plot(testData.time_stamp(spt:ept),x_pred2(spt:ept,i),'k')
%     hold on
        plot(testData.time_stamp(spt:ept),x_pred4(spt:ept,i),'g')
    hold on
    else
    plot(testData.time_stamp(spt:ept),x_pred4(spt:ept,i),'g')
    hold on    
    ylabel(ylabelvec{i})
    % ylim([0 20])
    hold on
    end
    end
            legend('exp','ekf','ekf_d')
       sgtitle("ekf state est"+ " P="+string(Pk_old(1,1))+" Q="+string(Q(1,1))+" R="+string(R(1,1))+" T="+string(dT)) 
% figure(2)
% 
%         hold on
%     for i =1:8
%     subplot(2,4,i)
% 
% 
%     plot(testData.time_stamp(spt:ept),(exp_arr(spt:ept,i)-x_pred2(spt:ept,i))/(x_max(i)-x_min(i)),'k')
%     hold on
%     plot(testData.time_stamp(spt:ept),(exp_arr(spt:ept,i)-x_pred(spt:ept,i))/(x_max(i)-x_min(i)),'r:',LineWidth=2)
%     ylabel(ylabelvec{i})
%     % ylim([0 20])
%     hold on
%     end
%             legend('ekf_imp','ekf')
%        sgtitle("ekf error"+ " P="+string(Pk_old(1,1))+" Q="+string(Q(1,1))+" R="+string(R(1,1))+" T="+string(dT))


