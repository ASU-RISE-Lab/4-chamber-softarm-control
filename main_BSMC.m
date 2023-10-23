%% EKF main
% q8x1 = [theta1,lc1,theta2,lc2,pm1,pm2,pm3,pm4]';
% u4x1_psi = [pd1,pd2,pd3,pd4]';
% kk1 = -1.767*v4x1(1)^2 + 17.55*abs(v4x1(1))+33.471;
% kk2 = 10.25*v4x1(2)^2 - 325.1*v4x1(2)+3299;
% kk3 =-1.013*v4x1(3)^2+11.55*abs(v4x1(3))+4.419;
% kk4 = 15.34*v4x1(4)^2 - 474.1*v4x1(4)+4475;
% d1= 0.9725*v4x1(1)^2- 11.2*abs(v4x1(1))+38.471;
% d2= -0.9725*v4x1(2)^2+ 30.23*v4x1(2)+435.471;
% d3=  0.1125*v4x1(3)^2- 1.2*abs(v4x1(3))+14.471;
% d4= 4.34*v4x1(4)^2 - 155.21*v4x1(4)+2146;
%v4x1(1,1) = -(pm11 - pm12);
%v4x1(2,1) = (pm11 + pm12);
%v4x1(3,1) = -(pm21 - pm22);
%v4x1(4,1) = (pm21 + pm22);
% Kmat = diag([kk1,kk2,kk3,kk4]);
% Dmat = diag([d1,d2,d3,d4]);
% pm11 = x10x1(1); pm12 = x10x1(2); pm13 = x10x1(3);
% pm21 = x10x1(4); pm22 = x10x1(5); pm23 = x10x1(6);

clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
fprintf( 'Loading... \n' );
load('trainData4.mat','par_set');
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
            outputKnown.state_vel(i,1) = (s1.l_t(i)-s1.l_t(i-1))/par_set.Ts; 
            outputKnown.state_vel(i,2) = (s1.r_t(i)-s1.r_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,3) = (s2.l_t(i)-s2.l_t(i-1))/par_set.Ts;
            outputKnown.state_vel(i,4) = (s2.r_t(i)-s2.r_t(i-1))/par_set.Ts; 
        end
    end
    for i = 1:length(s1.l_t)
        if i ==1
            outputKnown.state_acc(i,1) = 0; 
            outputKnown.state_acc(i,2) = 0; 
            outputKnown.state_acc(i,3) = 0; 
            outputKnown.state_acc(i,4) = 0; 


        else
            outputKnown.state_acc(i,1) = (outputKnown.state_vel(i,1)-outputKnown.state_vel(i-1,1))/par_set.Ts; 
            outputKnown.state_acc(i,2) = (outputKnown.state_vel(i,2)-outputKnown.state_vel(i-1,2))/par_set.Ts;
            outputKnown.state_acc(i,3) = (outputKnown.state_vel(i,3)-outputKnown.state_vel(i-1,3))/par_set.Ts;
            outputKnown.state_acc(i,4) = (outputKnown.state_vel(i,4)-outputKnown.state_vel(i-1,4))/par_set.Ts; 
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
%%% Get Mddtdq %%%
for i  = 1:length(s1.l_t)
    Mati = [];
    [Mmati] = funcMCGcalv2(outputKnown.arc_state_wire);
    outputKnown.Mddtdq(i,:) = Mmati*outputKnown.arc_acc_wire(i,:)';
end
fprintf('State Var estimation done \n')
%%% End %%%
close all
figure(1)
for i =1:4
    subplot(4,1,i)
    yyaxis left
    plot(outputKnown.arc_state_wire(:,i))
    hold on
    yyaxis right
    plot(outputKnown.u_pm_tf(:,i))
    hold on
    plot(outputKnown.u_pm_tf(:,i)-outputKnown.Mddtdq(:,i))
    hold on
end
%% RK4 with mean model
h=1/30
spt =1;ept =length(testData.enco_volts);
x4x1 = [outputKnown.arc_state_wire(spt,1:4)]';
for i = spt:ept
    u4x1 = outputKnown.u_pm_psi(i,1:4)';
    x_pred(i,:) = funcRK4NoPmDynSegallpmpara_m(x4x1,u4x1,h,par_set);
    x4x1 = x_pred(i,:)';
end
h=1/30
spt =1;ept =length(testData.enco_volts);
x8x1 = [outputKnown.arc_state_wire(spt,1:8)]';
for i = spt:ept
    u4x1 = outputKnown.u_pm_tf(:,1:4)';
    x_pred2(i,:) = funcRK4_pm_minus_M(x8x1,u4x1,h,par_set);
    x8x1 = x_pred2(i,:)';
end

%%%%%%% Plot result
close all
clc
ylabelvec={'theta1';'lc1';'theta2';'lc2';'tau1';'f1';'tau2';'f2'};
figure(1)
    for i =1:4
    subplot(2,4,i)
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(spt:ept,i))
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(spt:ept,i))
    
    hold on
    ylabel(ylabelvec{i})
    % ylim([0 20])
    if i ==1
        legend('exp','rk4')
        title('rk4 state sim')
    end
    hold on
    end
    for i =5:8
    subplot(2,4,i)
    plot(testData.time_stamp(spt:ept),outputKnown.u_pm_psi(spt:ept,i-4))
    hold on
    ylabel(ylabelvec{i})
    % ylim([ 20])
    if i ==5
        legend('Input(psi)')
%         title('rk4 state sim')
    end
    hold on
    end
%% theta1
xold = [outputKnown.arc_state_wire(1,1)]';
h = 1/40
Dmax = 100; dkmax = 40; dcmax = 18;dxmax = 1; 
xd = [outputKnown.arc_state_wire(:,1)]';
dtdxd = [outputKnown.arc_state_wire(:,5)]';
kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;
eta0 = 1e4;
umax =15;umin=-15;
for i = 1:length(xd)
    e0 = xd(i)-xold;
    Dmax = 1/d1*(dxmax*abs(xold)+dcmax*dxmax);
    eta1 = Dmax+0.01;
    u = (1/d1)*(dtdxd(i)-kk1*xold+eta0*e0+eta1*sign(e0));
    if u <=umin
        u=umin;
    elseif u>=umax
        u=umax;
    end
    xnew = funcRK4_one_state(xold,u,kk1,d1,h);
    xold =xnew;
    xest(i,:) =xnew;
    u_unb(i,:)=u;
end
close all
figure(1)
subplot(2,1,1)
plot(xest)
hold on
plot(outputKnown.arc_state_wire(:,1))
subplot(2,1,2)
plot(u_unb)
%% lc1
xold = [outputKnown.arc_state_wire(1,2)]';
h = 1/40
Dmax = 1e7;dkmax = 852; dcmax = 1709;dxmax = 1; 
xd = [outputKnown.arc_state_wire(:,2)]';
dtdxd = [outputKnown.arc_state_wire(:,6)]';
kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;
eta0 = 5e7;
umax =20;umin=-20;
ki = kk2;
di = d2;
for i = 1:length(xd)
    e0 = xd(i)-xold;
    Dmax = 1/d1*(dkmax*abs(xold)+dcmax*dxmax);
    eta1 = Dmax+0.01;
    u = (1/di)*(dtdxd(i)-ki*xold+eta0*e0+eta1*sign(e0));
    if u <=umin
        u=umin;
    elseif u>=umax
        u=umax;
    end
    xnew = funcRK4_one_state(xold,u,ki,di,h);
    xold =xnew;
    xest(i,:) =xnew;
    u_unb(i,:)=u;
end
close all
figure(1)
subplot(2,1,1)
plot(xest)
hold on
plot(outputKnown.arc_state_wire(:,2))
subplot(2,1,2)
plot(u_unb)

%% theta1 and lc1
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';
dtdxd = [outputKnown.arc_state_wire(:,5:8)]';
h = 1/30
Dmax1 = 5e2;Dmax2 =2e4;
dkmax1 = 40; dcmax1 = 18;dxmax1 = 1;
dkmax2 = 852; dcmax2 = 1709;dxmax2 = 1; 
xest=[]
xd = [outputKnown.arc_state_wire(:,1:4)]';
pold1 = testData.pm_psi(1,1);
pold2 = testData.pm_psi(1,2);
zold1 = abs(testData.pm_psi(1,2)-testData.pm_psi(1,1));
zold2 = abs(testData.pm_psi(1,2)+testData.pm_psi(1,1));

kk1 = -1.767*zold1^2+17.55*zold1+8.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;


d1= 0.9725*zold1^2- 11.2*zold1+38.471;
d2= -0.9725*zold1^2+ 30.23*zold1+435.471;

eta01 = 4e3;eta02 = 9e7;
umax =20;umin=-20;
pdmax =20;pdmin=0;
kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;
for i = 1:length(xd)
kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
% kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
% kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
% d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
% d4= 4.34*zold4^2 - 155.21*zold4+2146;

    e01 = xd(1,i)-xold(1);
    e02 = xd(2,i)-xold(2);
Dmax1 = 1/d1*(dkmax1*abs(xold(1))+dcmax1*dxmax1);
Dmax2 = 1/d2*(dkmax2*abs(xold(2))+dcmax2*dxmax2);
    
    eta1 = Dmax1+0.01;
    eta2 = Dmax2+0.01;

    u1 = (1/d1)*(dtdxd(1,i)-kk1*xold(1)+eta01*e01+eta1*sign(e01));
    u2 = (1/d2)*(dtdxd(2,i)-kk2*xold(2)+eta02*e02+eta2*sign(e02));

    if u1 <=umin
        u1=umin;
    elseif u1>=umax
        u1=umax;
    end
    if u2 <=umin
        u2=umin;
    elseif u2>=umax
        u2=umax;
    end


    x1new = funcRK4_one_state(xold(1),u1,kk1,d1,h);
    x2new = funcRK4_one_state(xold(2),u2,kk2,d2,h);

    xold =[x1new;x2new;];
    xest(i,:) =[x1new;x2new;];
    pd1(i) = (u2-u1)/2;
    pd2(i) = (u2+u1)/2;

    if pd1(i)<=pdmin
        pd1(i) = pdmin;
    elseif pd1(i)>=pdmax
        pd1(i)=pdmax;
    end
    if pd2(i)<=pdmin
        pd2(i) = pdmin;
    elseif pd2(i)>=pdmax
        pd2(i)=pdmax;
    end

    pnew1 = funcRK4pm4ch_m(pold1,alpha,beta,pd1(i),h);
    pnew2 = funcRK4pm4ch_m(pold2,alpha,beta,pd2(i),h);
    
    zold1 =abs(pnew2-pnew1);
    zold2 =abs(pnew2+pnew1) ;
    pold1 = pnew1;
    pold2 = pnew2;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
end
close all
figure(1)
subplot(4,1,1)
plot(xest(:,1))
hold on
plot(outputKnown.arc_state_wire(:,1))
hold on

subplot(4,1,2)
plot(xest(:,2))
hold on
plot(outputKnown.arc_state_wire(:,2))

subplot(4,1,3)
plot(pd1)
hold on
plot(pm1)
subplot(4,1,4)
plot(pd2)
hold on
plot(pm2)

%% theta1, lc1, theta2, lc2 ASMC
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';

h = 1/30
Dmax1 = 5e2;Dmax2 =4e4;
dkmax1 = 40; dcmax1 = 18;dxmax1 = 1;
dkmax2 = 852; dcmax2 = 1709;dxmax2 = 1; 
dkmax3 = 40; dcmax3 = 18;dxmax3 = 1;
dkmax4 = 852; dcmax4 = 1709;dxmax4 = 1; 
xest=[]
xd = [outputKnown.arc_state_wire(:,1:4)]';
dtdxd = [outputKnown.arc_state_wire(:,5:8)]';
pold1 = testData.pm_psi(1,1);
pold2 = testData.pm_psi(1,2);
pold3 = testData.pm_psi(1,4);
pold4 = testData.pm_psi(1,5);

zold1 = testData.pm_psi(1,2)-testData.pm_psi(1,1);
zold2 = testData.pm_psi(1,2)+testData.pm_psi(1,1);
zold3 = testData.pm_psi(1,5)-testData.pm_psi(1,4);
zold4 = testData.pm_psi(1,5)+testData.pm_psi(1,4);

kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;
eta01 = 4e3;eta02 = 9e7;
eta03 = 4e3;eta04 = 9e7;
umax =20;umin=-20;
pdmax =20;pdmin=0;
for i = 1:length(xd)
kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;

    e01 = xd(1,i)-xold(1);
    e02 = xd(2,i)-xold(2);
    e03 = xd(3,i)-xold(3);
    e04 = xd(4,i)-xold(4);
    Dmax1 = 1/d1*(dkmax1*abs(xold(1))+dcmax1*dxmax1);
    Dmax2 = 1/d2*(dkmax2*abs(xold(2))+dcmax2*dxmax2);
    Dmax3 = 1/d3*(dkmax3*abs(xold(1))+dcmax3*dxmax3);
Dmax4 = 1/d4*(dkmax4*abs(xold(2))+dcmax4*dxmax4);
    eta1 = Dmax1+0.01;
    eta2 = Dmax2+0.01;
    eta3 = Dmax1+0.01;
    eta4 = Dmax2+0.01;
    u1 = (1/d1)*(dtdxd(1,i)-kk1*xold(1)+eta01*e01+eta1*sign(e01));
    u2 = (1/d2)*(dtdxd(2,i)-kk2*xold(2)+eta02*e02+eta2*sign(e02));
    u3 = (1/d3)*(dtdxd(3,i)-kk3*xold(1)+eta03*e03+eta3*sign(e03));
    u4 = (1/d4)*(dtdxd(4,i)-kk4*xold(2)+eta04*e04+eta4*sign(e04));
    if u1 <=umin
        u1=umin;
    elseif u1>=umax
        u1=umax;
    end
    if u2 <=umin
        u2=umin;
    elseif u2>=umax
        u2=umax;
    end
    if u3 <=umin
        u3=umin;
    elseif u3>=umax
        u3=umax;
    end
    if u4 <=umin
        u4=umin;
    elseif u4>=umax
        u4=umax;
    end

    x1new = funcRK4_one_state(xold(1),u1,kk1,d1,h);
    x2new = funcRK4_one_state(xold(2),u2,kk2,d2,h);
    x3new = funcRK4_one_state(xold(3),u3,kk3,d3,h);
    x4new = funcRK4_one_state(xold(4),u4,kk4,d4,h);
    xold =[x1new;x2new;x3new;x4new];
    xest(i,:) =[x1new;x2new;x3new;x4new];
    pd1(i) = (u2-u1)/2;
    pd2(i) = (u2+u1)/2;
    pd3(i) = (u4-u3)/2;
    pd4(i) = (u4+u3)/2;
    if pd1(i)<=pdmin
        pd1(i) = pdmin;
    elseif pd1(i)>=pdmax
        pd1(i)=pdmax;
    end
    if pd2(i)<=pdmin
        pd2(i) = pdmin;
    elseif pd2(i)>=pdmax
        pd2(i)=pdmax;
    end
    if pd3(i)<=pdmin
        pd3(i) = pdmin;
    elseif pd3(i)>=pdmax
        pd3(i)=pdmax;
    end
    if pd4(i)<=pdmin
        pd4(i) = pdmin;
    elseif pd4(i)>=pdmax
        pd4(i)=pdmax;
    end
    pnew1 = funcRK4pm4ch_m(pold1,alpha,beta,pd1(i),h);
    pnew2 = funcRK4pm4ch_m(pold2,alpha,beta,pd2(i),h);
    pnew3 = funcRK4pm4ch_m(pold3,alpha,beta,pd3(i),h);
    pnew4 = funcRK4pm4ch_m(pold4,alpha,beta,pd4(i),h);
    zold1 =pnew2-pnew1;
    zold2 =pnew2+pnew1 ;
    zold3 =pnew4-pnew3;
    zold4 =pnew4+pnew3;
    pold1=pnew1;
    pold2=pnew2;
    pold3=pnew3;
    pold4=pnew4;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
    pm3(i)=pnew3;
    pm4(i)=pnew4;
end
close all
figure(1)
subplot(4,2,1)
plot(xest(:,1))
hold on
plot(outputKnown.arc_state_wire(:,1))
hold on
title('$\theta_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,2)
plot(xest(:,3))
hold on
plot(outputKnown.arc_state_wire(:,3))
hold on
title('$L_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,3)
plot(xest(:,2))
hold on
plot(outputKnown.arc_state_wire(:,2))
hold on
title('$\theta_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,4)
plot(xest(:,4))
hold on
plot(outputKnown.arc_state_wire(:,4))
title('$L_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,5)
plot(pd1)
hold on
plot(pm1)
title('$p_1$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,6)
plot(pd2)
hold on
plot(pm2)
title('$p_2$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,7)
plot(pd3)
hold on
plot(pm3)
title(['$p_3$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,8)
plot(pd4)
hold on
plot(pm4)
title(['$p_4$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
sgtitle("ASMC LPV at 40Hz")

%% theta1, lc1, theta2, lc2 ANDOBSMC
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';
h = 1/30
Dmax1 = 5e2;Dmax2 =4e4;
dkmax1 = 40; dcmax1 = 18;dxmax1 = 1;
dkmax2 = 852; dcmax2 = 1709;dxmax2 = 1; 
dkmax3 = 40; dcmax3 = 18;dxmax3 = 1;
dkmax4 = 852; dcmax4 = 1709;dxmax4 = 1; 
xest=[]
xd = [outputKnown.arc_state_wire(:,1:4)]';
dtdxd = [outputKnown.arc_state_wire(:,5:8)]';
pold1 = testData.pm_psi(1,1);
pold2 = testData.pm_psi(1,2);
pold3 = testData.pm_psi(1,4);
pold4 = testData.pm_psi(1,5);

zold1 = testData.pm_psi(1,2)-testData.pm_psi(1,1);
zold2 = testData.pm_psi(1,2)+testData.pm_psi(1,1);
zold3 = testData.pm_psi(1,5)-testData.pm_psi(1,4);
zold4 = testData.pm_psi(1,5)+testData.pm_psi(1,4);



kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;
eta02 = 9e7;
eta03 = 4e3;eta04 = 9e7;
    eta2 = Dmax2+0.01;
    eta3 = Dmax1+0.01;
    eta4 = Dmax2+0.01;
umax =20;umin=-20;
pdmax =20;pdmin=0;
%%%%%   NDOB  %%%%%%%
%%%% optimal
l1 = -0.1;eta01 = 3e3;    eta1 = 1e1;
destold1 =0;
destnew1=0;
intvar1 =0;intvar1old = 0;
pxold1=0;

l2 = -0.1;eta02 = 1e7;    eta2 = 1e2;
destold2 =0;
destnew2=0;
intvar2 =0;intvar2old= 0;
pxold2=0;

l3 = -0.1;eta03 = 3e3;    eta3 = 1e1;
destold3 =0;
destnew3=0;
intvar3 =0;intvar3old = 0;
pxold3=0;

l4 = -0.1;eta04 = 1e7;    eta4 = 1e2;
destold4 =0;
destnew4=0;
intvar4 =0;intvar4old = 0;
pxold4=0;

% l1 = -0.1;eta01 = 3e4;    eta1 = 1e1;
% destold1 =0;
% destnew1=0;
% intvar1new =0;dtdintvar1 = 0;
% pxold1=0;
% 
% l2 = -0.1;eta02 = 1e8;    eta2 = 1e2;
% destold2 =0;
% destnew2=0;
% intvar2new =0;dtdintvar2 = 0;
% pxold2=0;
% 
% l3 = -0.1;eta03 = 3e4;    eta3 = 1e1;
% destold3 =0;
% destnew3=0;
% intvar3new =0;dtdintvar3 = 0;
% pxold3=0;
% 
% l4 = -0.1;eta04 = 1e8;    eta4 = 1e2;
% destold4 =0;
% destnew4=0;
% intvar4new =0;dtdintvar4 = 0;
% pxold4=0;
%%%%%%%%%%%
for i = 1:length(xd)
kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;

    e01 = xd(1,i)-xold(1);
    e02 = xd(2,i)-xold(2);
    e03 = xd(3,i)-xold(3);
    e04 = xd(4,i)-xold(4);

    pxold1 = l1*xold(1);
    intvar1new = funcRK4fintvar_onestate(h,l1,e01,eta1);
    destnew1 = intvar1new + pxold1;


    pxold2 = l2*xold(2);
    intvar2new = funcRK4fintvar_onestate(h,l2,e02,eta2);
    destnew2 = intvar2new + pxold2;

        pxold3 = l3*xold(3);
    intvar3new = funcRK4fintvar_onestate(h,l3,e03,eta3);
    destnew3 = intvar3new + pxold3;


    pxold4 = l4*xold(4);
    intvar4new = funcRK4fintvar_onestate(h,l4,e04,eta4);
    destnew4 = intvar4new + pxold4;

%     Dmax1 = 1/d1*(dkmax1*abs(xold(1))+dcmax1*dxmax1);
%     Dmax2 = 1/d2*(dkmax2*abs(xold(2))+dcmax2*dxmax2);
    Dmax3 = 1/d3*(dkmax3*abs(xold(1))+dcmax3*dxmax3);
Dmax4 = 1/d4*(dkmax4*abs(xold(2))+dcmax4*dxmax4);

    u1 = 1/d1*(dtdxd(1,i) - kk1*xold(1) +eta01*e01+eta1*sign(e01)-destold1);
    u2 = 1/d2*(dtdxd(2,i) - kk2*xold(2) +eta02*e02+eta2*sign(e02)-destold2);
    u3 = 1/d3*(dtdxd(3,i) - kk3*xold(1) +eta03*e03+eta3*sign(e03)-destold3);
    u4 = 1/d4*(dtdxd(4,i) - kk4*xold(2) +eta04*e04+eta4*sign(e04)-destold4);
% %     u1 = (1/d1)*(xd(1,i)-kk1*xold(1)+eta01*e01+eta1*sign(e01));
% %     u2 = (1/d2)*(xd(2,i)-kk2*xold(2)+eta02*e02+eta2*sign(e02));
%     u3 = (1/d3)*(xd(3,i)-kk3*xold(1)+eta03*e03+eta3*sign(e03));
%     u4 = (1/d4)*(xd(4,i)-kk4*xold(2)+eta04*e04+eta4*sign(e04));
    if u1 <=umin
        u1=umin;
    elseif u1>=umax
        u1=umax;
    end
    if u2 <=umin
        u2=umin;
    elseif u2>=umax
        u2=umax;
    end
    if u3 <=umin
        u3=umin;
    elseif u3>=umax
        u3=umax;
    end
    if u4 <=umin
        u4=umin;
    elseif u4>=umax
        u4=umax;
    end

    x1new = funcRK4_one_state(xold(1),u1,kk1,d1,h);
    x2new = funcRK4_one_state(xold(2),u2,kk2,d2,h);
    x3new = funcRK4_one_state(xold(3),u3,kk3,d3,h);
    x4new = funcRK4_one_state(xold(4),u4,kk4,d4,h);
    xold =[x1new;x2new;x3new;x4new];
    xest(i,:) =[x1new;x2new;x3new;x4new];
    pd1(i) = (u2-u1)/2;
    pd2(i) = (u2+u1)/2;
    pd3(i) = (u4-u3)/2;
    pd4(i) = (u4+u3)/2;
    if pd1(i)<=pdmin
        pd1(i) = pdmin;
    elseif pd1(i)>=pdmax
        pd1(i)=pdmax;
    end
    if pd2(i)<=pdmin
        pd2(i) = pdmin;
    elseif pd2(i)>=pdmax
        pd2(i)=pdmax;
    end
    if pd3(i)<=pdmin
        pd3(i) = pdmin;
    elseif pd3(i)>=pdmax
        pd3(i)=pdmax;
    end
    if pd4(i)<=pdmin
        pd4(i) = pdmin;
    elseif pd4(i)>=pdmax
        pd4(i)=pdmax;
    end
    pnew1 = funcRK4pm4ch_m(pold1,alpha,beta,pd1(i),h);
    pnew2 = funcRK4pm4ch_m(pold2,alpha,beta,pd2(i),h);
    pnew3 = funcRK4pm4ch_m(pold3,alpha,beta,pd3(i),h);
    pnew4 = funcRK4pm4ch_m(pold4,alpha,beta,pd4(i),h);
    zold1 =pnew2-pnew1;
    zold2 =pnew2+pnew1 ;
    zold3 =pnew4-pnew3;
    zold4 =pnew4+pnew3;
    pold1=pnew1;
    pold2=pnew2;
    pold3=pnew3;
    pold4=pnew4;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
    pm3(i)=pnew3;
    pm4(i)=pnew4;
    destold1 = destnew1;
    destold2 = destnew2;
    destold3 = destnew3;
    destold4 = destnew4;
end
close all
figure(1)
subplot(4,2,1)
plot(xest(:,1))
hold on
plot(outputKnown.arc_state_wire(:,1))
hold on
title('$\theta_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,3)
plot(xest(:,2))
hold on
plot(outputKnown.arc_state_wire(:,2))
hold on
title('$L_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,2)
plot(xest(:,3))
hold on
plot(outputKnown.arc_state_wire(:,3))
hold on
title('$\theta_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,4)
plot(xest(:,4))
hold on
plot(outputKnown.arc_state_wire(:,4))
title('$L_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,5)
plot(pd1)
hold on
plot(pm1)
title('$p_1$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,7)
plot(pd2)
hold on
plot(pm2)
title('$p_2$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,6)
plot(pd3)
hold on
plot(pm3)
title(['$p_3$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,8)
plot(pd4)
hold on
plot(pm4)
title(['$p_4$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
sgtitle("NDOBASMC LPV at 40Hz")

%% theta1, lc1, theta2, lc2 INDOBASMC
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';
h = 1/30

xest=[]
xd = [outputKnown.arc_state_wire(:,1:4)]';
dxdxd = [outputKnown.arc_state_wire(:,5:8)]';
pold1 = testData.pm_psi(1,1);
pold2 = testData.pm_psi(1,2);
pold3 = testData.pm_psi(1,4);
pold4 = testData.pm_psi(1,5);

zold1 = testData.pm_psi(1,2)-testData.pm_psi(1,1);
zold2 = testData.pm_psi(1,2)+testData.pm_psi(1,1);
zold3 = testData.pm_psi(1,5)-testData.pm_psi(1,4);
zold4 = testData.pm_psi(1,5)+testData.pm_psi(1,4);

umax =20;umin=-20;
pdmax =20;pdmin=0;
%%%%%   NDOB  %%%%%%%
%%%% optimal
% l1 = -0.1;eta01 = 3e3;    eta1 = 1e1;
% destold1 =0;
% destnew1=0;
% intvar1 =0;dtdintvar1 = 0;
% pxold1=0;
% 
% l2 = -0.1;eta02 = 1e7;    eta2 = 1e2;
% destold2 =0;
% destnew2=0;
% intvar2 =0;dtdintvar2 = 0;
% pxold2=0;
% 
% l3 = -0.1;eta03 = 3e3;    eta3 = 1e1;
% destold3 =0;
% destnew3=0;
% intvar3 =0;dtdintvar3 = 0;
% pxold3=0;
% 
% l4 = -0.1;eta04 = 1e7;    eta4 = 1e2;
% destold4 =0;
% destnew4=0;
% intvar4 =0;dtdintvar4 = 0;
% pxold4=0;

l1x = 0.1;eta01 = 3e3; etalx1 =1e2;
dtdestmax1 = 0.01;
destold1 =0;
destnew1=0;
intvar1new =0;dtdintvar1 = 0;
pxold1=0;

l2x = 0.1;eta02 = 1e8;    eta2 = 1e2;
destold2 =0;
destnew2=0;
intvar2new =0;dtdintvar2 = 0;
pxold2=0;

l3x = 0.1;eta03 = 3e4;    eta3 = 1e1;
destold3 =0;
destnew3=0;
intvar3new =0;dtdintvar3 = 0;
pxold3=0;

l4x = 0.1;eta04 = 1e8;    eta4 = 1e2;
destold4 =0;
destnew4=0;
intvar4new =0;dtdintvar4 = 0;
pxold4=0;
%%%%%%%%%%%
for i = 1:length(xd)
kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;

    e01 = xd(1,i)-xold(1);
    e02 = xd(2,i)-xold(2);
    e03 = xd(3,i)-xold(3);
    e04 = xd(4,i)-xold(4);

    pxold1 = l1x*xold(1) + etalx1*xold(1)^2*sign(xold(1));
    l1 = l1x + 2*etalx1*abs(xold(1));
    intvar1new = funcRK4fintvar_onestate_improve(h,l1,pxold1,kk1,d1,xold(1),zold1,intvar1old);
    destnew1 = intvar1new + pxold1;


    pxold2 = l2*xold(2);
    intvar2new = funcRK4fintvar_onestate(h,l2,e02,eta2);
    destnew2 = intvar2new + pxold2;

    pxold3 = l3*xold(3);
    intvar3new = funcRK4fintvar_onestate(h,l3,e03,eta3);
    destnew3 = intvar3new + pxold3;


    pxold4 = l4*xold(4);
    intvar4new = funcRK4fintvar_onestate(h,l4,e04,eta4);
    destnew4 = intvar4new + pxold4;
    eta1 = abs(dtdxd(1,i)) + dtdestmax1/l1; 


    u1 = 1/d1*(dtdxd(1,i) - kk1*xold(1) +eta01*e01+eta1*sign(e01)-destold1);
    u2 = 1/d2*(dtdxd(2,i) - kk2*xold(2) +eta02*e02+eta2*sign(e02)-destold2);
    u3 = 1/d3*(dtdxd(3,i) - kk3*xold(1) +eta03*e03+eta3*sign(e03)-destold3);
    u4 = 1/d4*(dtdxd(4,i) - kk4*xold(2) +eta04*e04+eta4*sign(e04)-destold4);

    if u1 <=umin
        u1=umin;
    elseif u1>=umax
        u1=umax;
    end
    if u2 <=umin
        u2=umin;
    elseif u2>=umax
        u2=umax;
    end
    if u3 <=umin
        u3=umin;
    elseif u3>=umax
        u3=umax;
    end
    if u4 <=umin
        u4=umin;
    elseif u4>=umax
        u4=umax;
    end

    x1new = funcRK4_one_state(xold(1),u1,kk1,d1,h);
    x2new = funcRK4_one_state(xold(2),u2,kk2,d2,h);
    x3new = funcRK4_one_state(xold(3),u3,kk3,d3,h);
    x4new = funcRK4_one_state(xold(4),u4,kk4,d4,h);
    xold =[x1new;x2new;x3new;x4new];
    xest(i,:) =[x1new;x2new;x3new;x4new];
    pd1(i) = (u2-u1)/2;
    pd2(i) = (u2+u1)/2;
    pd3(i) = (u4-u3)/2;
    pd4(i) = (u4+u3)/2;
    if pd1(i)<=pdmin
        pd1(i) = pdmin;
    elseif pd1(i)>=pdmax
        pd1(i)=pdmax;
    end
    if pd2(i)<=pdmin
        pd2(i) = pdmin;
    elseif pd2(i)>=pdmax
        pd2(i)=pdmax;
    end
    if pd3(i)<=pdmin
        pd3(i) = pdmin;
    elseif pd3(i)>=pdmax
        pd3(i)=pdmax;
    end
    if pd4(i)<=pdmin
        pd4(i) = pdmin;
    elseif pd4(i)>=pdmax
        pd4(i)=pdmax;
    end
    pnew1 = funcRK4pm4ch_m(pold1,alpha,beta,pd1(i),h);
    pnew2 = funcRK4pm4ch_m(pold2,alpha,beta,pd2(i),h);
    pnew3 = funcRK4pm4ch_m(pold3,alpha,beta,pd3(i),h);
    pnew4 = funcRK4pm4ch_m(pold4,alpha,beta,pd4(i),h);
    zold1 =pnew2-pnew1;
    zold2 =pnew2+pnew1 ;
    zold3 =pnew4-pnew3;
    zold4 =pnew4+pnew3;
    pold1=pnew1;
    pold2=pnew2;
    pold3=pnew3;
    pold4=pnew4;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
    pm3(i)=pnew3;
    pm4(i)=pnew4;
    destold1 = destnew1;
    destold2 = destnew2;
    destold3 = destnew3;
    destold4 = destnew4;
end
close all
figure(1)
subplot(4,2,1)
plot(xest(:,1))
hold on
plot(outputKnown.arc_state_wire(:,1))
hold on
title('$\theta_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,3)
plot(xest(:,2))
hold on
plot(outputKnown.arc_state_wire(:,2))
hold on
title('$L_1$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,2)
plot(xest(:,3))
hold on
plot(outputKnown.arc_state_wire(:,3))
hold on
title('$\theta_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('rad')
subplot(4,2,4)
plot(xest(:,4))
hold on
plot(outputKnown.arc_state_wire(:,4))
title('$L_2$','Interpreter','latex')
legend('exp','sim',Location='south')
ylabel('m')
subplot(4,2,5)
plot(pd1)
hold on
plot(pm1)
title('$p_1$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,7)
plot(pd2)
hold on
plot(pm2)
title('$p_2$','Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,6)
plot(pd3)
hold on
plot(pm3)
title(['$p_3$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
subplot(4,2,8)
plot(pd4)
hold on
plot(pm4)
title(['$p_4$'],'Interpreter','latex')
legend('pd','pm')
ylabel('psi')
sgtitle("INDOBASMC LPV at 40Hz")
%% bsmc for 1 state
h = 1/30;alpha = -0.9665; beta = 0.9698;
clc
output_x=[];
xold = [outputKnown.arc_state_wire(1,1)]';
zold = [testData.pm_psi(1,2)-testData.pm_psi(1,1)]';
xd = [outputKnown.arc_state_wire(:,1)]';
dtdxd = [outputKnown.arc_state_wire(:,5)'];
ddtdxd = [outputKnown.arc_acc_wire(:,1)'];
xdi = xd(:,1);
destold = zeros(1,1);
dtdest = zeros(1,1);
dtdintvar = zeros(1,1);
eta0 = 1000;eta1 = 100;eta2 = 100; l = 100;
alpha = -0.9665; beta = 0.9698;
%%%%%%
pold = Lx*xold; epsilon = 0.1;D = ones(1,1)*0.001;
%%%%%%
for j =1:length(xd)
xdi = xd(:,j);
dtdxdi = dtdxd(:,j);
ddtdxdi = ddtdxd(:,j);    
vi = zold;

kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;


Fmat = diag([kk1/d1]);
Bmat = diag([1/d1]);
Cmat = diag([d1]);
Kmat = diag([kk1]);
%%%%% NDOB
dtdintvar = l*eye(length(Fmat))*(-Fmat*xold -Bmat*vi - destold);
intvar = funcRK4fintvar(xold,vi,destold,h,Lx,Fmat,Bmat);
destnew = intvar + pold;
dtdest= (destnew-destold)*h;
%%%% END

e0 = xdi-xold;
vdn = Cmat*(dtdxdi - Fmat*xdi-eta0*e0);
vd = vdn -Cmat*destold;
e1 = vd-vi;
hv = Cmat*(ddtdxdi + eta0mat*dtdxdi-(eta0mat + Kmat)*(Fmat*xold+Bmat*vi));
u = 1/(beta)*(hv - Cmat*dtdest - (eta0mat + Kmat)*destold...
    -alpha*zold+eta2mat*sign(e1)+Bmat*e0-eta1mat*e1);
for i =1:4
    if u<=-20
        u=-20;
    elseif u>=20
        u=20;
    end
end
znew = funcRK4pm4ch_m(zold,alpha,beta,u,h);
xnew = funcRK4xdyn_m(xold,vi,h,Fmat,Bmat,D);

output_x(:,j) = xnew(1);
output_z(:,j) = znew(1);
xold = xnew;
zold =znew;
destold = destnew;
end
close all
figure(1)
plot(xd)
hold on
plot(output_x(1,:))
ylim([-0.4 0])
%%
clc
xold = [outputKnown.arc_state_wire(1,1:4)]';
zold = [testData.pm_psi(1,1),testData.pm_psi(1,2),testData.pm_psi(1,4),testData.pm_psi(1,5)]';
xd4xn = [outputKnown.arc_state_wire(:,1:4)]';
dtdxd4xn = zeros(4,length(xd4xn));
ddtdxd4xn = zeros(4,length(xd4xn));
xd4x1 = xd4xn(:,1);
dtdxd4x1 = dtdxd4xn(:,1);
ddtdxd4x1 = ddtdxd4xn(:,1);
destold = zeros(4,1);
destnew = zeros(4,1);
dtdintvar = zeros(4,1);
eta0 = 1;eta1 = 1;eta2 = 1;phi = 0.01; 
freq = 1/40
H = [[-1 1;1 1],zeros(2,2);
    zeros(2,2),[-1 1;1 1]]; 
alpha = -0.9665; beta = 0.9698;
%%%%%%
l = 0.01; Lx = l*eye(length(Fmat)); pold = Lx*xold; epsilon = 0.1;D = ones(4,1)*0.001;
%%%%%%
for j =1:length(xd4xn)
xd4x1 = xd4xn(:,j);
dtdxd4x1 = dtdxd4xn(:,j);
ddtdxd4x1 = ddtdxd4xn(:,j);    
v4x1 = H*zold;

kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;
Fmat = diag([kk1/d1,kk2/d2,kk3/d3,kk4/d4]);
Bmat = diag([1/d1,1/d2,1/d3,1/d4]);
Cmat = diag([d1,d2,d3,d4]);
Kmat = diag([kk1,kk2,kk3,kk4]);

dtdintvar = l*eye(length(Fmat))*(-Fmat*xold -Bmat*v4x1 - destold);
intvar = funcRK4fintvar(xold,v4x1,destold,freq,Lx,Fmat,Bmat);
destnew = intvar + pold;
e0 = xd4x1-x4x1;
vdn = Cmat*(dtdxd4x1 - Fmat*xd4x1-eta0*e0);
vd4x1 = vdn -Cmat*destold;
e1 = vd4x1-v4x1;
eta0mat = eta0*eye(length(Fmat));
eta1mat = eta1*eye(length(Fmat));
eta2mat = eta2*eye(length(Fmat));
for i =1:4
    e0i = e0(i);
    e1i = e1(i);
if e0i ==0 && e1i ==0
    eta0mat(i,i)= eta0;eta1mat(i,i) = eta1;eta2mat(i,i) = eta2;
elseif e0i ==0 && e1i ~=0
eta0mat(i,i)= eta0;eta1mat(i,i) = eta1;eta2mat(i,i) = eta2;
elseif e0i ~=0 && e1i ==0
eta0mat(i,i)= eta0;eta1mat(i,i) = (phi + epsilon/l)/(abs(e0i));
eta2mat(i,i) = eta2;   
else
eta0mat(i,i)= eta0;
eta1mat(i,i) = max([eta1,(-phi -eta0mat(i,i)*e0i^2 +abs(e0i)*epsilon/l -eta2mat(i,i)*abs(e1i))/e1i^2]);
eta2mat(i,i) = eta2;     
end
end
hv = Cmat*ddtdxd4x1-eta0mat*dtdxd4x1+(eta0mat-Kmat)*(Fmat*xold+Bmat*v4x1);
u4x1 = inv(beta*H)*(hv - Cmat*dtdintvar -(eta0mat-Kmat)*destold...
    -alpha*H*zold+eta2mat*sign(e1)+Bmat*e0-eta1mat*e1);
% for i =1:4
%     if u4x1(i)<=0
%         u4x1(i)=0;
%     elseif u4x1(i)>=15
%         u4x1(i)=15;
%     end
% end
znew = funcRK4pm4ch_m(zold,alpha,beta,u4x1,h);
xnew = funcRK4xdyn_m(xold,v4x1,h,Fmat,Bmat,D);

output_x(1:4,j) = xnew;
output_z(1:4,j) = znew;
xold = xnew;
zold =znew;
destold = destnew;
end
close all
figure(1)
plot(output_x(1,:))
