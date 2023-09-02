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
syms theta1 theta2 lc1 lc2 m0
Bqtheta1limit =limit(par_set.B_q,theta1,0);
Bqtheta2limit = limit(par_set.B_q,theta2,0);
Bqtotallimit = limit(Bqtheta1limit,theta2,0);

Cqtheta1limit =limit(par_set.C_q,theta1,0);
Cqtheta2limit = limit(par_set.C_q,theta2,0);
Cqtotallimit = limit(Cqtheta1limit,theta2,0);


Gqtheta1limit =limit(par_set.G_q,theta1,0);
Gqtheta2limit = limit(par_set.G_q,theta2,0);
Gqtotallimit = limit(Gqtheta1limit,theta2,0);
%% ode fix 2 seg 8 link using wire enco readings only
par_set.EOM=1;
par_set = funcEOMbaseFrame2segwire_v1(par_set);
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
testData = par_set.trial4;
output = funcComputeNNInputOutputPair_v1(testData,par_set);
% mocapResult = funcComputeStateVar_v1(testData,par_set);
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
testData = par_set.trial3;
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
%% least square for K and D aug28
testData = par_set.trial3;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
% Kx + Ddx = -u +mddq +cqdq +gq = y
% (K+Ds)X(s) = Y(s) ----- G(s) = X(s)/Y(s) =  1/(K + Ds)
spt = 1; ept = length(outputKnown.state_array);
var1 = iddata(outputKnown.state_array(spt:ept,1),(outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)'),par_set.Ts);
var2 = iddata(outputKnown.state_array(spt:ept,3),(-outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)'),par_set.Ts);
var3 = iddata(outputKnown.state_array(spt:ept,5),(outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)'),par_set.Ts);
var4 = iddata(outputKnown.state_array(spt:ept,7),(-outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)'),par_set.Ts);

var1x = outputKnown.state_array(spt:ept,1);
var1y = outputKnown.state_array(spt:ept,2);
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = outputKnown.state_array(spt:ept,3);
var2y = outputKnown.state_array(spt:ept,4);
var2z = -(outputKnown.u_pm_tf(spt:ept,2) + outputKnown.mcg_array(2,spt:ept)');

var3x = outputKnown.state_array(spt:ept,5);
var3y = outputKnown.state_array(spt:ept,6);
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = outputKnown.state_array(spt:ept,7);
var4y = outputKnown.state_array(spt:ept,8);
var4z = -(outputKnown.u_pm_tf(spt:ept,4) + outputKnown.mcg_array(4,spt:ept)');
% k1 = 26.04,d1 = 20.60
% k2 = -2.729e+04 d2 = -2.823e+04, a2 =3680
% k3 =42.66 d3 =33.87
%%
testData = par_set.trial3;
trainningdata.wireEncoderReadings_unit_mV = testData.enco_volts;
    d_x = testData.rigid_3_pose(:,1) - testData.rigid_1_pose(:,1);
    d_y = testData.rigid_3_pose(:,2) - testData.rigid_1_pose(:,2);
    d_z = testData.rigid_3_pose(:,3) - testData.rigid_1_pose(:,3)  + 0.004;
    theta_mocap_rad = -(2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2)));
trainningdata.xzphiy_unit_m_m_rad = [d_x,d_z,theta_mocap_rad];

testData = par_set.trial4;
validationdata.wireEncoderReadings_unit_mV = testData.enco_volts;
    d_x = testData.rigid_3_pose(:,1) - testData.rigid_1_pose(:,1);
    d_y = testData.rigid_3_pose(:,2) - testData.rigid_1_pose(:,2);
    d_z = testData.rigid_3_pose(:,3) - testData.rigid_1_pose(:,3)  + 0.004;
    theta_mocap_rad = -(2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2)));
validationdata.xzphiy_unit_m_m_rad = [d_x,d_z,theta_mocap_rad];
%%
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
% Kx + Ddx = -u +mddq +cqdq +gq = y
% (K+Ds)X(s) = Y(s) ----- G(s) = X(s)/Y(s) =  1/(K + Ds)
spt = 100; ept = length(outputKnown.state_array);
var2 = iddata(outputKnown.mean_u_pm_psi(spt:ept),(outputKnown.u_pm_tf(spt:ept,2) + outputKnown.mcg_array(2,spt:ept)'./outputKnown.state_array(spt:ept,3)),par_set.Ts);
pmvar1x = outputKnown.mean_u_pm_psi(spt:ept);
pmvar1y = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)'./outputKnown.state_array(spt:ept,1));

pmvar2x = outputKnown.mean_u_pm_psi(spt:ept);
pmvar2y = outputKnown.state_array(spt:ept,3);
pmvar2z = (outputKnown.u_pm_tf(spt:ept,2) + outputKnown.mcg_array(2,spt:ept));

pmvar4x = outputKnown.mean_u_pm_psi(spt:ept);
pmvar4y = (outputKnown.u_pm_tf(spt:ept,4) + outputKnown.mcg_array(4,spt:ept)'./outputKnown.state_array(spt:ept,7));

% k2_pm = 41.08 * mean(pm_psi)+46.5 0.99R^2
% k4_pm = 41.42 * mean(pm_psi)+3.933 0.99R^2

%% validation
testData = par_set.trial2;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
yright = outputKnown.u_pm_tf - ...
    [26.64*outputKnown.state_array(:,1),... 
    (41.08 * outputKnown.mean_u_pm_psi + 46.5).*outputKnown.state_array(:,3),... 
    52.4075*outputKnown.state_array(:,5),... 
    (41.42 * outputKnown.mean_u_pm_psi + 3.933).*outputKnown.state_array(:,7)]...
    -    [23.0894*outputKnown.state_array(:,2),... 
    0*outputKnown.state_array(:,4),... 
    50.6842*outputKnown.state_array(:,6),... 
    0*outputKnown.state_array(:,8)];

yupm = outputKnown.u_pm_tf;
ykpm=    [26.64*outputKnown.state_array(:,1),... 
    (41.08 * outputKnown.mean_u_pm_psi + 46.5).*outputKnown.state_array(:,3),... 
    52.4075*outputKnown.state_array(:,5),... 
    (41.42 * outputKnown.mean_u_pm_psi + 3.933).*outputKnown.state_array(:,7)];

yleft = outputKnown.mcg_array';
close all
figure(1)
titlelist = {'theta1','lc1','theta2','lc2'};
for i  =1 :4
subplot(4,1,i)
plot(yleft(:,i))
hold on 
plot(yright(:,i))
hold on
title(titlelist{i})
legend('mcg','u-kq')
end 
return
for i  =1 :4
subplot(4,1,i)
yyaxis left
plot(yupm(i,:))
ylabel(unitlistleft{i})
ylim(ylimleft(i,:))
hold on
yyaxis right
ylabel(unitlistright{i})
ylim(ylimright(i,:))
plot(output.state_array(:,2*i-1))
title(titlelist{i})
end
%% least square for K and alpha
testData = par_set.trial3;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 100; ept = length(outputKnown.state_array);
var1x = outputKnown.state_array(spt:ept,1);
var1y = outputKnown.u_pm_tf(spt:ept,1);
var1z = outputKnown.mcg_array(1,spt:ept);

var2x = outputKnown.state_array(spt:ept,3);
var2y = outputKnown.u_pm_tf(spt:ept,2);
var2z = outputKnown.mcg_array(2,spt:ept);

var3x = outputKnown.state_array(spt:ept,5);
var3y = outputKnown.u_pm_tf(spt:ept,3);
var3z = outputKnown.mcg_array(3,spt:ept);

var4x = outputKnown.state_array(spt:ept,7);
var4y = outputKnown.u_pm_tf(spt:ept,4);
var4z = outputKnown.mcg_array(4,spt:ept);


%% GP use data set 1

xob1 = [var1x,var1y];
yob1 = var1z;
gprMdl1 = fitrgp(xob1,yob1);
[ypred1,~,yint1] = predict(gprMdl1,xob1);
close all
fig = figure;
fig.Position(3) = fig.Position(3)*4;
tiledlayout(2,2,'TileSpacing','compact')
nexttile
hold on
scatter3(xob1(:,1),xob1(:,2),yob1,'r');
plot3(xob1(:,1),xob1(:,2),ypred1,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('theta1')
xlabel('x (rad)')
ylabel('dx (rad/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob2 = [var2x,var2y];
yob2 = var2z;
gprMdl2 = fitrgp(xob2,yob2);
[ypred2,~,yint2] = predict(gprMdl2,xob2);

nexttile
hold on
scatter3(xob2(:,1),xob2(:,2),yob2,'r');
plot3(xob2(:,1),xob2(:,2),ypred2,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('lc1')
xlabel('x (m)')
ylabel('dx (m/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob3 = [var3x,var3y];
yob3 = var3z;
gprMdl3 = fitrgp(xob3,yob3);
[ypred3,~,yint3] = predict(gprMdl3,xob3);

nexttile
hold on
scatter3(xob3(:,1),xob3(:,2),yob3,'r');
plot3(xob3(:,1),xob3(:,2),ypred3,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('theta2')
xlabel('x (rad)')
ylabel('dx (rad/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob4 = [var4x,var4y];
yob4 = var4z;
gprMdl4 = fitrgp(xob4,yob4);
[ypred4,~,yint4] = predict(gprMdl4,xob4);

nexttile
hold on
scatter3(xob4(:,1),xob4(:,2),yob4,'r');
plot3(xob4(:,1),xob4(:,2),ypred4,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('lc2')
xlabel('x (m)')
ylabel('dx (m/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

%% sysid for pneumatic controller
spt =1;
pctrl1 = iddata(testData.pm_psi(spt:ept,1),testData.pd_psi(spt:ept,1));
pctrl2 = iddata(testData.pm_psi(spt:ept,2),testData.pd_psi(spt:ept,2));
pctrl3 = iddata(testData.pm_psi(spt:ept,3),testData.pd_psi(spt:ept,3));
pctrl4 = iddata(testData.pm_psi(spt:ept,4),testData.pd_psi(spt:ept,4));
pctrl5 = iddata(testData.pm_psi(spt:ept,5),testData.pd_psi(spt:ept,5));
pctrl6 = iddata(testData.pm_psi(spt:ept,6),testData.pd_psi(spt:ept,6));

pctrlMerge = merge(pctrl1,pctrl4);
% dpm = -0.0353*pm + 0.03768*pd
%% RNN training
testData = par_set.trial2;
outputvalid = funcKnownTerm2seg_v1(testData,par_set);
spt = 100; ept = length(outputvalid.state_array);
var1x = outputvalid.state_array(spt:ept,1);
var1y = outputvalid.state_array(spt:ept,2);
var1z = (outputvalid.u_pm_tf(spt:ept,1) - outputvalid.mcg_array(1,spt:ept)');

var2x = outputvalid.state_array(spt:ept,3);
var2y = outputvalid.state_array(spt:ept,4);
var2z = (-outputvalid.u_pm_tf(spt:ept,2) - outputvalid.mcg_array(2,spt:ept)');

var3x = outputvalid.state_array(spt:ept,5);
var3y = outputvalid.state_array(spt:ept,6);
var3z = (outputvalid.u_pm_tf(spt:ept,3) - outputvalid.mcg_array(3,spt:ept)');

var4x = outputvalid.state_array(spt:ept,7);
var4y = outputvalid.state_array(spt:ept,8);
var4z = (-outputvalid.u_pm_tf(spt:ept,4) - outputvalid.mcg_array(4,spt:ept)');

xob1 = [var1x,var1y];
yob1 = var1z;
%%
% gprMdl1 = fitrgp(xob1,yob1);
[ypred1,~,yint1] = predict(gprMdl1,xob1);
close all
fig = figure;
fig.Position(3) = fig.Position(3)*4;
tiledlayout(2,2,'TileSpacing','compact')
nexttile
hold on
scatter3(xob1(:,1),xob1(:,2),yob1,'r');
plot3(xob1(:,1),xob1(:,2),ypred1,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('theta1')
xlabel('x (rad)')
ylabel('dx (rad/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob2 = [var2x,var2y];
yob2 = var2z;
% gprMdl2 = fitrgp(xob2,yob2);
[ypred2,~,yint2] = predict(gprMdl2,xob2);

nexttile
hold on
scatter3(xob2(:,1),xob2(:,2),yob2,'r');
plot3(xob2(:,1),xob2(:,2),ypred2,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('lc1')
xlabel('x (m)')
ylabel('dx (m/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob3 = [var3x,var3y];
yob3 = var3z;
% gprMdl3 = fitrgp(xob3,yob3);
[ypred3,~,yint3] = predict(gprMdl3,xob3);

nexttile
hold on
scatter3(xob3(:,1),xob3(:,2),yob3,'r');
plot3(xob3(:,1),xob3(:,2),ypred3,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('theta2')
xlabel('x (rad)')
ylabel('dx (rad/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

xob4 = [var4x,var4y];
yob4 = var4z;
% gprMdl4 = fitrgp(xob4,yob4);
[ypred4,~,yint4] = predict(gprMdl4,xob4);

nexttile
hold on
scatter3(xob4(:,1),xob4(:,2),yob4,'r');
plot3(xob4(:,1),xob4(:,2),ypred4,'g')                  % GPR predictions
% patch([xob;flipud(xob)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
view(-37.5,30)
title('lc2')
xlabel('x (m)')
ylabel('dx (m/s)')
zlabel('f(x) = u-mcg')
legend({'Noise-free observations','GPR predictions'},'Location','best')

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
