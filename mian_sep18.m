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
% %% ode fix 1 seg 4 link
% par_set.EOM=1;
% par_set = funcEOMbaseFrame1seg_v4(par_set);
% simplify(par_set.Ti{end})
%% ode fix 2 seg 8 link
par_set.EOM=1;
par_set = funcEOMbaseFrame2seg_v4(par_set);
simplify(par_set.Ti{end});
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
% %%
% testData = par_set.trial3;
% mocapResult = funcComputeStateVar_v2(testData,par_set);
% %%
% testData = par_set.trial4;
% output = funcComputeNNInputOutputPair_v1(testData,par_set);
% %% Forward Kinematics 1 seg
% testData = par_set.trial2;
% mocapResult=[];
% mocapResult = funcComputeStateVar_v1(testData,par_set);
% fkResult = funcCompuFK1seg_v1(mocapResult.state_array);
% close all
% figure(1)
% subplot(3,1,1)
% plot(fkResult.camFrameE1(:,1),'r')
% hold on
% plot(fkResult.camFrameE1(:,2),'b')
% hold on
% plot(fkResult.camFrameE1(:,3),'k')
% hold on
% 
% plot(testData.rigid_2_pose(:,1),'r--')
% hold on
% plot(testData.rigid_2_pose(:,2),'b--')
% hold on
% plot(testData.rigid_2_pose(:,3),'k--')
% hold on
% title('FK result for Endeffector 1 in Cam frame')
% legend('xfk','yfk','zfk','xmo','ymo','zmo')
% subplot(3,1,2)
% title('Normalized input-output pair')
% plot((mocapResult.state_array(:,1)-min(mocapResult.state_array(:,1)))./(max(mocapResult.state_array(:,1))-min(mocapResult.state_array(:,1))))
% hold on
% plot((mocapResult.u_pm_tf(:,1)-min(mocapResult.u_pm_tf(:,1)))./(max(mocapResult.u_pm_tf(:,1))-min(mocapResult.u_pm_tf(:,1))))
% hold on
% plot((mocapResult.state_array(:,3)-min(mocapResult.state_array(:,3)))./(max(mocapResult.state_array(:,3))-min(mocapResult.state_array(:,3))))
% hold on
% plot((mocapResult.u_pm_tf(:,2)-min(mocapResult.u_pm_tf(:,2)))./(max(mocapResult.u_pm_tf(:,2))-min(mocapResult.u_pm_tf(:,2))))
% legend('theta1','tau1','lc1','f1')
% subplot(3,1,3)
% title('fk errors')
% plot(fkResult.camFrameE1(:,1)-testData.rigid_2_pose(:,1))
% hold on
% plot(fkResult.camFrameE1(:,2)-testData.rigid_2_pose(:,2))
% hold on
% plot(fkResult.camFrameE1(:,3)-testData.rigid_2_pose(:,3))
% hold on
% legend('x','y','z')
%% FK RESULT 2 seg
testData = par_set.trial3;
funcFullFK2seg(testData,par_set);
%% least square for K and D aug28
testData = par_set.trial3;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
% Kx + Ddx = u -(mddq +cqdq +gq) = y
% (K+Ds)X(s) = Y(s) ----- G(s) = X(s)/Y(s) =  1/(K + Ds)
spt = 1; ept = length(outputKnown.state_array_wire);
var1 = iddata(outputKnown.state_array_wire(spt:ept,1),(outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)'),par_set.Ts);
var2 = iddata(outputKnown.state_array_wire(spt:ept,3),(outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)'),par_set.Ts);
var3 = iddata(outputKnown.state_array_wire(spt:ept,5),(outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)'),par_set.Ts);
var4 = iddata(outputKnown.state_array_wire(spt:ept,7),(outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)'),par_set.Ts);

var1x = outputKnown.state_array_wire(spt:ept,1);
var1y = outputKnown.state_array_wire(spt:ept,2);
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = outputKnown.state_array_wire(spt:ept,3);
var2y = outputKnown.state_array_wire(spt:ept,4);
var2z = (outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)');

var3x = outputKnown.state_array_wire(spt:ept,5);
var3y = outputKnown.state_array_wire(spt:ept,6);
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = outputKnown.state_array_wire(spt:ept,7);
var4y = outputKnown.state_array_wire(spt:ept,8);
var4z = (outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)');
% k1 = 25.3,d1 = 20.22
% k2 = -2.729e+04 d2 = -2.823e+04, a2 =3680
% k3 =42.66 d3 =33.87
% k4 = -4.786e+04 d2 =-1.863e+04 a4 = 4623

%% least square for K and D aug28 z = a*y - b*x
testData = par_set.trial3;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);

var1x = outputKnown.state_array_wire(spt:ept,1);
var1y = outputKnown.u_pm_psi(spt:ept,1);
var1z = outputKnown.mcg_array(1,spt:ept)';

var2x = outputKnown.state_array_wire(spt:ept,3);
var2y = outputKnown.u_pm_psi(spt:ept,2);
var2z = outputKnown.mcg_array(2,spt:ept)';

var3x = outputKnown.state_array_wire(spt:ept,5);
var3y = outputKnown.u_pm_psi(spt:ept,3);
var3z = outputKnown.mcg_array(3,spt:ept)';

var4x = outputKnown.state_array_wire(spt:ept,7);
var4y = outputKnown.u_pm_psi(spt:ept,4);
var4z = outputKnown.mcg_array(4,spt:ept);


%% GP use data set 1
testData = par_set.trial1;

outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = outputKnown.state_array_wire(spt:ept,1:4);
var1y = outputKnown.state_array_wire(spt:ept,1:4);
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = outputKnown.state_array_wire(spt:ept,1:4);
var2y = outputKnown.state_array_wire(spt:ept,1:4);
var2z = (outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)');

var3x = outputKnown.state_array_wire(spt:ept,1:8);
var3y = outputKnown.state_array_wire(spt:ept,1:8);
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = outputKnown.state_array_wire(spt:ept,1:8);
var4y = outputKnown.state_array_wire(spt:ept,1:8);
var4z = outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)';

xob1 = [var1x,var1y];
yob1 = var1z;
gprMdl1 = fitrgp(xob1,yob1);
[ypred1,~,yint1] = predict(gprMdl1,xob1);
close all

xob2 = [var2x,var2y];
yob2 = var2z;
gprMdl2 = fitrgp(xob2,yob2);
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
gprMdl3 = fitrgp(xob3,yob3);
[ypred3,~,yint3] = predict(gprMdl3,xob3);



xob4 = [var4x,var4y];
yob4 = var4z;
gprMdl4 = fitrgp(xob4,yob4);
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')
%% validation GP
testData = par_set.trial7;

outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = outputKnown.state_array_wire(spt:ept,1:4);
var1y = outputKnown.state_array_wire(spt:ept,1:4);
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = outputKnown.state_array_wire(spt:ept,1:4);
var2y = outputKnown.state_array_wire(spt:ept,1:4);
var2z = outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)';

var3x = outputKnown.state_array_wire(spt:ept,1:8);
var3y = outputKnown.state_array_wire(spt:ept,1:8);
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = outputKnown.state_array_wire(spt:ept,1:8);
var4y = outputKnown.state_array_wire(spt:ept,1:8);
var4z = outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)';


xob1 = [var1x,var1y];
yob1 = var1z;
[ypred1,~,yint1] = predict(gprMdl1,xob1);

xob2 = [var2x,var2y];
yob2 = var2z;
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
[ypred3,~,yint3] = predict(gprMdl3,xob3);



xob4 = [var4x,var4y];
yob4 = var4z;
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')


%% GP use data set 1 with pm
testData = par_set.trial4;

outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = [outputKnown.state_array_wire(spt:ept,1:2)];
var1y = testData.pm_psi(:,1:3)/20;
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = [outputKnown.state_array_wire(spt:ept,3:4)];
var2y = testData.pm_psi(:,1:3)/20;
var2z = (outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)');

var3x = [outputKnown.state_array_wire(spt:ept,5:6)];
var3y = testData.pm_psi(:,4:6)/20;
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = [outputKnown.state_array_wire(spt:ept,7:8)];
var4y = testData.pm_psi(:,4:6)/20;
var4z = outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)';

xob1 = [var1x,var1y];
yob1 = var1z;
gprMdl1 = fitrgp(xob1,yob1);
[ypred1,~,yint1] = predict(gprMdl1,xob1);
close all

xob2 = [var2x,var2y];
yob2 = var2z;
gprMdl2 = fitrgp(xob2,yob2);
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
gprMdl3 = fitrgp(xob3,yob3);
[ypred3,~,yint3] = predict(gprMdl3,xob3);



xob4 = [var4x,var4y];
yob4 = var4z;
gprMdl4 = fitrgp(xob4,yob4);
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')
%% validation GP with pm
testData = par_set.trial7;

outputKnown = funcKnownTerm2seg_v2(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = [outputKnown.state_array_wire(spt:ept,1:2)];
var1y = testData.pm_psi(:,1:3)/20;
var1z = (outputKnown.u_pm_tf(spt:ept,1) - outputKnown.mcg_array(1,spt:ept)');

var2x = [outputKnown.state_array_wire(spt:ept,3:4)];
var2y = testData.pm_psi(:,1:3)/20;
var2z = (outputKnown.u_pm_tf(spt:ept,2) - outputKnown.mcg_array(2,spt:ept)');

var3x = [outputKnown.state_array_wire(spt:ept,5:6)];
var3y = testData.pm_psi(:,4:6)/20;
var3z = (outputKnown.u_pm_tf(spt:ept,3) - outputKnown.mcg_array(3,spt:ept)');

var4x = [outputKnown.state_array_wire(spt:ept,7:8)];
var4y = testData.pm_psi(:,4:6)/20;
var4z = outputKnown.u_pm_tf(spt:ept,4) - outputKnown.mcg_array(4,spt:ept)';


xob1 = [var1x,var1y];
yob1 = var1z;
[ypred1,~,yint1] = predict(gprMdl1,xob1);

xob2 = [var2x,var2y];
yob2 = var2z;
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
[ypred3,~,yint3] = predict(gprMdl3,xob3);



xob4 = [var4x,var4y];
yob4 = var4z;
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')




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
%% RK4 simulation for pm regulator
testData = par_set.trial2;
alpha = -0.04; beta = 0.03768
h=1.0
x_pred = [];
x6x1 = testData.pm_psi(1,:)';
for i = 1:length(testData.pm_psi)  
    u6x1 = testData.pd_psi(i,:)';
    x_pred(i,:) = funcRK4pmODE_m(x6x1,alpha,beta,u6x1,h);
    x6x1 = x_pred(i,:)';
end
close all
figure(1)
for i  = 1:6 
subplot(3,2,i)

plot(testData.pm_psi(:,i),'k')
hold on
plot(x_pred(:,i),'r--')
hold on
ylim([0 20])
if i ==1
    legend('est','exp')
end
end

%% GP for a*pm-kx-ddotx - other
testData = par_set.trial2;

outputKnown = funcKnownTerm2seg_v3(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = [outputKnown.state_array_wire(spt:ept,1:2)];
var1y = testData.pm_psi(:,1:3)/20;
var1z = outputKnown.mcg_array(1,spt:ept)';

var2x = [outputKnown.state_array_wire(spt:ept,3:4)];
var2y = testData.pm_psi(:,1:3)/20;
var2z = outputKnown.mcg_array(2,spt:ept)';

var3x = [outputKnown.state_array_wire(spt:ept,5:6)];
var3y = testData.pm_psi(:,4:6)/20;
var3z = outputKnown.mcg_array(3,spt:ept)';

var4x = [outputKnown.state_array_wire(spt:ept,7:8)];
var4y = testData.pm_psi(:,4:6)/20;
var4z = outputKnown.mcg_array(4,spt:ept)';

xob1 = [var1x,var1y];
yob1 = var1z;
gprMdl1 = fitrgp(xob1,yob1);
[ypred1,~,yint1] = predict(gprMdl1,xob1);

xob2 = [var2x,var2y];
yob2 = var2z;
gprMdl2 = fitrgp(xob2,yob2);
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
gprMdl3 = fitrgp(xob3,yob3);
[ypred3,~,yint3] = predict(gprMdl3,xob3);

xob4 = [var4x,var4y];
yob4 = var4z;
gprMdl4 = fitrgp(xob4,yob4);
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')
%% validation GP with pm
testData = par_set.trial2;

outputKnown = funcKnownTerm2seg_v3(testData,par_set);
spt = 1; ept = length(outputKnown.state_array_wire);
var1x = [outputKnown.state_array_wire(spt:ept,1:2)];
var1y = testData.pm_psi(:,1:3)/20;
var1z = outputKnown.mcg_array(1,spt:ept)';

var2x = [outputKnown.state_array_wire(spt:ept,3:4)];
var2y = testData.pm_psi(:,1:3)/20;
var2z = outputKnown.mcg_array(2,spt:ept)';

var3x = [outputKnown.state_array_wire(spt:ept,5:6)];
var3y = testData.pm_psi(:,4:6)/20;
var3z = outputKnown.mcg_array(3,spt:ept)';

var4x = [outputKnown.state_array_wire(spt:ept,7:8)];
var4y = testData.pm_psi(:,4:6)/20;
var4z = outputKnown.mcg_array(4,spt:ept)';


xob1 = [var1x,var1y];
yob1 = var1z;
[ypred1,~,yint1] = predict(gprMdl1,xob1);

xob2 = [var2x,var2y];
yob2 = var2z;
[ypred2,~,yint2] = predict(gprMdl2,xob2);

xob3 = [var3x,var3y];
yob3 = var3z;
[ypred3,~,yint3] = predict(gprMdl3,xob3);



xob4 = [var4x,var4y];
yob4 = var4z;
[ypred4,~,yint4] = predict(gprMdl4,xob4);

close all
figure(1)
subplot(4,2,1)
plot(ypred1)
hold on
plot(var1z)
legend('pred','actul')
subplot(4,2,2)
plot(ypred1 - var1z)
hold on
title('e = pred - actul')

subplot(4,2,3)
plot(ypred2)
hold on
plot(var2z)
legend('pred','actul')
subplot(4,2,4)
plot(ypred2 - var2z)
hold on
title('e = pred - actul')


subplot(4,2,5)
plot(ypred3)
hold on
plot(var3z)
legend('pred','actul')
subplot(4,2,6)
plot(ypred3 - var3z)
hold on
title('e = pred - actul')

subplot(4,2,7)
plot(ypred4)
hold on
plot(var4z)
legend('pred','actul')
subplot(4,2,8)
plot(ypred4 - var4z)
hold on
title('e = pred - actul')

%% RK4 simulation for full order ode regulator
testData = par_set.trial2;
alpha = -0.04; beta = 0.03768;
h=1;
x_pred = [];
outputKnown = funcKnownTerm2seg_v3(testData,par_set);
x14x1 = [testData.pm_psi(1,:)';outputKnown.state_array_wire(1,:)';];
for i = 1:100  
    u6x1 = testData.pd_psi(i,:)';
    x_pred(i,:) = funcRK4fullODEv2_m(x14x1,alpha,beta,u6x1,h,gprMdl1,gprMdl2,gprMdl3,gprMdl4,par_set);
    x14x1 = x_pred(i,:)';
end
close all
figure(1)
for i  = 1:6 
subplot(3,2,i)

plot(testData.pm_psi(:,i),'k')
hold on
plot(x_pred(:,i),'r--')
hold on
ylim([0 20])
if i ==1
    legend('est','exp')
end
end

figure(2)
for i  = 1:8 
subplot(4,2,i)

plot(outputKnown.state_array_wire(:,i),'k')
hold on
plot(x_pred(:,i+6),'r--')
hold on
% ylim([0 20])
if i ==1
    legend('est','exp')
end
end




