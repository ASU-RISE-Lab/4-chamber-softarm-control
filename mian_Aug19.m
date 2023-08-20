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
%% ode fix 1 seg 4 link
par_set.EOM=1;
par_set = funcEOMbaseFrame1seg_v4(par_set);
simplify(par_set.Ti{end})
%% ode fix 2 seg 8 link
par_set.EOM=1;
par_set = funcEOMbaseFrame2seg_v4(par_set);
simplify(par_set.Ti{end})
par_set.Jxythetaz = par_set.J_xyz2q;
par_set.Jxythetaz(3,:) = [1 0 1 0 ];
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
%% Forward Kinematics 1 seg
testData = par_set.trial2;
mocapResult=[];
mocapResult = funcComputeStateVar_v1(testData,par_set);
fkResult = funcCompuFK1seg_v1(mocapResult.state_array);
close all
figure(1)
subplot(3,1,1)
plot(fkResult.camFrameE1(:,1),'r')
hold on
plot(fkResult.camFrameE1(:,2),'b')
hold on
plot(fkResult.camFrameE1(:,3),'k')
hold on

plot(testData.rigid_2_pose(:,1),'r--')
hold on
plot(testData.rigid_2_pose(:,2),'b--')
hold on
plot(testData.rigid_2_pose(:,3),'k--')
hold on
title('FK result for Endeffector 1 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)
title('Normalized input-output pair')
plot((mocapResult.state_array(:,1)-min(mocapResult.state_array(:,1)))./(max(mocapResult.state_array(:,1))-min(mocapResult.state_array(:,1))))
hold on
plot((mocapResult.u_pm_tf(:,1)-min(mocapResult.u_pm_tf(:,1)))./(max(mocapResult.u_pm_tf(:,1))-min(mocapResult.u_pm_tf(:,1))))
hold on
plot((mocapResult.state_array(:,3)-min(mocapResult.state_array(:,3)))./(max(mocapResult.state_array(:,3))-min(mocapResult.state_array(:,3))))
hold on
plot((mocapResult.u_pm_tf(:,2)-min(mocapResult.u_pm_tf(:,2)))./(max(mocapResult.u_pm_tf(:,2))-min(mocapResult.u_pm_tf(:,2))))
legend('theta1','tau1','lc1','f1')
subplot(3,1,3)
title('fk errors')
plot(fkResult.camFrameE1(:,1)-testData.rigid_2_pose(:,1))
hold on
plot(fkResult.camFrameE1(:,2)-testData.rigid_2_pose(:,2))
hold on
plot(fkResult.camFrameE1(:,3)-testData.rigid_2_pose(:,3))
hold on
legend('x','y','z')
%% FK RESULT 2 seg
testData = par_set.trial2;
mocapResult=[];
mocapResult = funcComputeStateVar_v1(testData,par_set);
fkResult = funcCompuFK2seg_v1(mocapResult.state_array);
close all
figure(1)
subplot(3,1,1)
plot(fkResult.camFrameE1(:,1),'r')
hold on
plot(fkResult.camFrameE1(:,2),'b')
hold on
plot(fkResult.camFrameE1(:,3),'k')
hold on

plot(testData.rigid_2_pose(:,1),'r--')
hold on
plot(testData.rigid_2_pose(:,2),'b--')
hold on
plot(testData.rigid_2_pose(:,3),'k--')
hold on
title('FK result for Endeffector 1 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)

plot((mocapResult.state_array(:,1)-min(mocapResult.state_array(:,1)))./(max(mocapResult.state_array(:,1))-min(mocapResult.state_array(:,1))))
hold on
plot((mocapResult.u_pm_tf(:,1)-min(mocapResult.u_pm_tf(:,1)))./(max(mocapResult.u_pm_tf(:,1))-min(mocapResult.u_pm_tf(:,1))))
hold on
plot((mocapResult.state_array(:,3)-min(mocapResult.state_array(:,3)))./(max(mocapResult.state_array(:,3))-min(mocapResult.state_array(:,3))))
hold on
plot((mocapResult.u_pm_tf(:,2)-min(mocapResult.u_pm_tf(:,2)))./(max(mocapResult.u_pm_tf(:,2))-min(mocapResult.u_pm_tf(:,2))))
hold on
legend('theta1','tau1','lc1','f1')
title('Normalized input-output pair')
subplot(3,1,3)

plot(fkResult.camFrameE1(:,1)-testData.rigid_2_pose(:,1))
hold on
plot(fkResult.camFrameE1(:,2)-testData.rigid_2_pose(:,2))
hold on
plot(fkResult.camFrameE1(:,3)-testData.rigid_2_pose(:,3))
hold on
title('fk errors in cam frame')
legend('x','y','z')

figure(2)
subplot(3,1,1)
plot(fkResult.camFrameE2(:,1),'r')
hold on
plot(fkResult.camFrameE2(:,2),'b')
hold on
plot(fkResult.camFrameE2(:,3),'k')
hold on

plot(testData.rigid_3_pose(:,1),'r--')
hold on
plot(testData.rigid_3_pose(:,2),'b--')
hold on
plot(testData.rigid_3_pose(:,3),'k--')
hold on
title('FK result for Endeffector 2 in Cam frame')
legend('xfk','yfk','zfk','xmo','ymo','zmo')
subplot(3,1,2)

plot((mocapResult.state_array(:,5)-min(mocapResult.state_array(:,5)))./(max(mocapResult.state_array(:,5))-min(mocapResult.state_array(:,5))))
hold on
plot((mocapResult.u_pm_tf(:,3)-min(mocapResult.u_pm_tf(:,3)))./(max(mocapResult.u_pm_tf(:,3))-min(mocapResult.u_pm_tf(:,3))))
hold on
plot((mocapResult.state_array(:,7)-min(mocapResult.state_array(:,7)))./(max(mocapResult.state_array(:,7))-min(mocapResult.state_array(:,7))))
hold on
plot((mocapResult.u_pm_tf(:,4)-min(mocapResult.u_pm_tf(:,4)))./(max(mocapResult.u_pm_tf(:,4))-min(mocapResult.u_pm_tf(:,4))))
legend('theta1','tau1','lc1','f1')
title('Normalized input-output pair')
subplot(3,1,3)
plot(fkResult.camFrameE2(:,1)-testData.rigid_3_pose(:,1))
hold on
plot(fkResult.camFrameE2(:,2)-testData.rigid_3_pose(:,2))
hold on
plot(fkResult.camFrameE2(:,3)-testData.rigid_3_pose(:,3))
hold on
title('fk errors in cam frame')
legend('x','y','z')
%% least square for K and D
testData = par_set.trial1;
outputKnown = funcKnownTerm2seg_v1(testData,par_set);
% Kx + Ddx = -u +mddq +cqdq +gq = y
% (K+Ds)X(s) = Y(s) ----- G(s) = X(s)/Y(s) =  1/(K + Ds)
spt = 100; ept = length(outputKnown.state_array);
var1 = iddata(outputKnown.state_array(spt:ept,1),(outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)'),par_set.Ts);
var2 = iddata(outputKnown.state_array(spt:ept,3),(outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)'),par_set.Ts);
var3 = iddata(outputKnown.state_array(spt:ept,5),(outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)'),par_set.Ts);
var4 = iddata(outputKnown.state_array(spt:ept,7),(outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)'),par_set.Ts);

var1x = outputKnown.state_array(spt:ept,1);
var1y = outputKnown.state_array(spt:ept,2);
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = outputKnown.state_array(spt:ept,3);
var2y = outputKnown.state_array(spt:ept,4);
var2z = (outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)');

var3x = outputKnown.state_array(spt:ept,5);
var3y = outputKnown.state_array(spt:ept,6);
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = outputKnown.state_array(spt:ept,7);
var4y = outputKnown.state_array(spt:ept,8);
var4z = (outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)');
%% RNN model
testData = par_set.trial1;
mocapResult = funcComputeStateVar_v1(testData,par_set);
nn_pred = [mocapResult.state_array,mocapResult.u_pm_psi,testData.pd_psi];
nn_resp = [mocapResult.state_array];

testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
nn_val_pred = [mocapResult.state_array,mocapResult.u_pm_psi,testData.pd_psi];
nn_val_resp = [mocapResult.state_array];
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
