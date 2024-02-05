%%%
% Initialization
%%%
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
% fprintf( 'Loading... \n' );
load('trainData4.mat','par_set');
% 
% fprintf( 'Data loaded \n' );

par_set.flag_read_exp =1;
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
%% Mean std for data
%%%
% Change line 28 for different cases
% Use Oct26 rmse asmc/new/nsmc for figure 
%%%
ctrl_flag =2;% 1:asmc 2:nsmc 3:inasmc
if par_set.flag_read_exp==1
    for i = 1:7
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
%% Plot for exp result 3 set
spt =1;ept =1500;
close all
theta1 =[];L1 =[];theta2=[];L2=[];

j=1;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=2;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=3;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

theta1_mean = mean(theta1,2);
L1_mean = mean(L1,2);
theta2_mean = mean(theta2,2);
L2_mean = mean(L2,2);
x_mean = [theta1_mean,L1_mean,theta2_mean,L2_mean];

theta1_upper = theta1_mean + std(theta1,0,2);
L1_upper = L1_mean + std(L1,0,2);
theta2_upper = theta2_mean + std(theta2,0,2);
L2_upper = L2_mean + std(L2,0,2);

theta1_lower = theta1_mean - std(theta1,0,2);
L1_lower = L1_mean - std(L1,0,2);
theta2_lower = theta2_mean - std(theta2,0,2);
L2_lower = L2_mean - std(L2,0,2);
close all
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
% spt =1;ept =1500;
% ctrl_flag = 1
testData = par_set.trial1;
figure(1)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),testData.xd(spt:ept,i),LineStyle="-",LineWidth=2,Color='b')
    hold on
    plot(testData.time_stamp(spt:ept),x_mean(spt:ept,i),'r')
    hold on
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
    if ctrl_flag ==1
    legend('Ref','ASMC',Location='southeast')
elseif ctrl_flag==2
legend('Ref','NSMC',Location='southeast')
else
legend('Ref','INASMC',Location='southeast')
    end
%              
        
    hold on
    end
    % Get RMSE
j =1;
testData = par_set.trial1;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

j =2;
testData = par_set.trial2;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

j =3;
testData = par_set.trial3;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

    if ctrl_flag ==1
    asmc_rmse = rmse;
elseif ctrl_flag==2
nsmc_rmse = rmse;
else
inasmc_rmse = rmse;
end
%% 7 set
spt =1;ept =1500;
close all
theta1 =[];L1 =[];theta2=[];L2=[];

j=1;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=2;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=3;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=4;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=5;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=6;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

j=7;
testData = par_set.trial1;
theta1(:,j) = testData.xm(spt:ept,1);
L1(:,j) = testData.xm(spt:ept,2);
theta2(:,j) = testData.xm(spt:ept,3);
L2(:,j) = testData.xm(spt:ept,4);

theta1_mean = mean(theta1,2);
L1_mean = mean(L1,2);
theta2_mean = mean(theta2,2);
L2_mean = mean(L2,2);
x_mean = [theta1_mean,L1_mean,theta2_mean,L2_mean];

theta1_upper = theta1_mean + std(theta1,0,2);
L1_upper = L1_mean + std(L1,0,2);
theta2_upper = theta2_mean + std(theta2,0,2);
L2_upper = L2_mean + std(L2,0,2);

theta1_lower = theta1_mean - std(theta1,0,2);
L1_lower = L1_mean - std(L1,0,2);
theta2_lower = theta2_mean - std(theta2,0,2);
L2_lower = L2_mean - std(L2,0,2);

close all
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
spt =1;ept =1500;
testData = par_set.trial1;
figure(1)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),testData.xd(spt:ept,i),LineStyle="-",LineWidth=2,Color='b')
    hold on
    plot(testData.time_stamp(spt:ept),x_mean(spt:ept,i),'r')
    hold on
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
    if ctrl_flag ==1
    legend('Ref','ASMC',Location='southeast')
elseif ctrl_flag==2
legend('Ref','NSMC',Location='southeast')
else
legend('Ref','INASMC',Location='southeast')
    end
%         
        
        
    hold on
    end
% Get RMSE
j =1;
testData = par_set.trial1;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

j =2;
testData = par_set.trial2;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

j =3;
testData = par_set.trial3;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end

j =4;
testData = par_set.trial4;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end
j =5;
testData = par_set.trial5;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end
j =6;
testData = par_set.trial6;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end
j =7;
testData = par_set.trial7;
for i =1:4
ydi = testData.xd(spt:ept,i);
y = testData.xm(spt:ept,i);
rmse(j,i) = sqrt((ydi-y)'*(ydi-y)/ept);
end
%%%%%%%%%%%%%%%%%%%
if ctrl_flag ==1
    asmc_rmse = rmse;
elseif ctrl_flag==2
nsmc_rmse = rmse;
else
inasmc_rmse = rmse;
end
%% Plot bar plot
%%%
% Need to run previous section 
% Or directly load rmse.mat
%%%
close all
mean_asmc = mean(asmc_rmse,1);
mean_nsmc = mean(nsmc_rmse,1);
mean_inasmc = mean(inasmc_rmse,1);
mean_all = [mean_asmc;mean_nsmc;mean_inasmc];
std_all = [std(asmc_rmse,1);std(nsmc_rmse,1);std(inasmc_rmse,1);];
low_all = mean_all
x_bar_pos=categorical({'ASMC','NSMC','INASMC'});
x_bar_pos=reordercats(x_bar_pos,{'ASMC','NSMC','INASMC'});
figure(1)
    for i =1:4
    subplot(2,2,i)
    bar_obj = bar(x_bar_pos,mean_all(:,i));
    bar_obj(1).FaceColor = 'flat';
    bar_obj.CData(1,:)= [211 211 211]/255;
    bar_obj.CData(2,:)= [128 128 128]/255;
    bar_obj.CData(3,:)= [129 133 137]/255;
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
%     xlim([0,4])       
    hold on
    er =errorbar(x_bar_pos,mean_all(:,i),-std_all(:,i), ...
        +std_all(:,i));
    er.Color = [0 0 0];                            
    er.LineStyle = 'none'; 
    hold on
    end
%% Plot bar plot
%%%
% Need to run previous section 
% Or directly load rmse.mat
%%%
close all
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
mean_asmc = mean(asmc_rmse,1);
mean_nsmc = mean(nsmc_rmse,1);
mean_inasmc = mean(inasmc_rmse,1);
mean_all = [mean_asmc;mean_nsmc;mean_inasmc];
std_all = [std(asmc_rmse,1);std(nsmc_rmse,1);std(inasmc_rmse,1);];
low_all = mean_all
x_bar_pos=categorical({'0','100','0.01'});
x_bar_pos=reordercats(x_bar_pos,{'0','100','0.01'});
figure(1)
    for i =1:4
    subplot(2,2,i)
    bar_obj = bar(x_bar_pos,mean_all(:,i));
    bar_obj(1).FaceColor = 'flat';
    bar_obj.CData(1,:)= [211 211 211]/255;
    bar_obj.CData(2,:)= [128 128 128]/255;
    bar_obj.CData(3,:)= [129 133 137]/255;
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
%     xlim([0,4])       
    hold on
    er =errorbar(x_bar_pos,mean_all(:,i),-std_all(:,i), ...
        +std_all(:,i));
    er.Color = [0 0 0];                            
    er.LineStyle = 'none'; 
    hold on
    end
%% Plot bar plot with only ASMC and INASMC
close all
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
mean_asmc = mean(asmc_rmse,1);
mean_nsmc = mean(nsmc_rmse,1);
mean_inasmc = mean(inasmc_rmse,1);
mean_all = [mean_asmc;mean_inasmc];
std_all = [std(asmc_rmse,1);std(inasmc_rmse,1);];
low_all = mean_all;
x_bar_pos=categorical({'ASMC','INASMC'});
x_bar_pos=reordercats(x_bar_pos,{'ASMC','INASMC'});
figure(1)
    for i =1:4
    subplot(2,2,i)
    bar_obj = bar(x_bar_pos,mean_all(:,i));
    bar_obj(1).FaceColor = 'flat';
    bar_obj.CData(1,:)= [211 211 211]/255;
    bar_obj.CData(2,:)= [128 128 128]/255;
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
%     xlim([0,4])       
    hold on
    er =errorbar(x_bar_pos,mean_all(:,i),-std_all(:,i), ...
        +std_all(:,i));
    er.Color = [0 0 0];                            
    er.LineStyle = 'none'; 
    hold on
    end

%% RK4 simulation with mean model and LPV
h=1/30
% testData = par_set.trial1;
spt =1;ept =length(testData.enco_volts);
x4x1 = [outputKnown.arc_state_wire(spt,1:4)]';
x4x1mean=[outputKnown.arc_state_wire(spt,1:4)]';
x_pred=[];x_pred2 = [];
for i = spt:ept
    u4x1 = outputKnown.u_pm_psi(i,1:4)';
    x_pred(i,:) = funcRK4NoPmDynSegallpmpara_m(x4x1,u4x1,h,par_set);
    x_pred2(i,:) = funcRK4NoPmDynSegallpmmean_m(x4x1mean,u4x1,h,par_set);
    x4x1 = x_pred(i,:)';
    x4x1mean = x_pred2(i,:)';
end

% spt =1;ept =length(testData.enco_volts);
% x8x1 = [outputKnown.arc_state_wire(spt,1:8)]';
% for i = spt:ept
%     u4x1 = outputKnown.u_pm_tf(:,1:4)';
%     x_pred2(i,:) = funcRK4_pm_minus_M(x8x1,u4x1,h,par_set);
%     x8x1 = x_pred2(i,:)';
% end

%%%%%%% Plot result
close all
clc
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
spt =1;ept =1600;
figure(1)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(spt:ept,i),LineStyle="-",LineWidth=1,Color='b')
    hold on
    plot(testData.time_stamp(spt:ept),x_pred2(spt:ept,i),LineStyle="-",LineWidth=2,Color='k')
    hold on
    plot(testData.time_stamp(spt:ept),x_pred(spt:ept,i),LineStyle="-",LineWidth=2,Color='r')
    hold on
    rmse_sim_mean(i) = ...
        sqrt(sum((outputKnown.arc_state_wire(spt:ept,i)-x_pred2(spt:ept,i)).^2) ... 
        /length(x_pred2(spt:ept,i))); 
    rmse_sim_lpv(i) = ...
        sqrt(sum((outputKnown.arc_state_wire(spt:ept,i)-x_pred(spt:ept,i)).^2) ... 
        /length(x_pred(spt:ept,i))); 
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
        legend('Exp','Mean','LPV',Location='southeast')
    hold on
    end

%% simulation of ASMC controller
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';
h = 1/30
Dmax1 = 5e2;Dmax2 =4e4;
dkmax1 = 40; dcmax1 = 18*2;dxmax1 = 1;
dkmax2 = 1000; dcmax2 = 1709*2;dxmax2 = 1; 
dkmax3 = 40; dcmax3 = 18*2;dxmax3 = 1;
dkmax4 = 852*2; dcmax4 = 1709*2;dxmax4 = 1; 
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
eta01 = 1;eta02 = 1;
eta03 = 1;eta04 = 1;
umaxt =30;umint=-30;
umaxf =30;uminf=0;
pdmax =15;pdmin=0;
for i = 1:length(xd)
kk1 = -1.767*zold1^2 + 17.55*abs(zold1)+33.471;
kk2 = 10.25*zold2^2 - 325.1*zold2+3299;
kk3 =-1.013*zold3^2+11.55*abs(zold3)+4.419;
kk4 = 15.34*zold4^2 - 474.1*zold4+4475;

d1= 0.9725*zold1^2- 11.2*abs(zold1)+38.471;
d2= -0.9725*zold2^2+ 30.23*zold2+435.471;
d3=  0.1125*zold3^2- 1.2*abs(zold3)+14.471;
d4= 4.34*zold4^2 - 155.21*zold4+2146;

    e01 = -(xd(1,i)-xold(1));
    e02 = -(xd(2,i)-xold(2));
    e03 = -(xd(3,i)-xold(3));
    e04 = -(xd(4,i)-xold(4));
    Dmax1 = 1/d1*(dkmax1*abs(xold(1))+dcmax1*dxmax1);
    Dmax2 = 1/d2*(dkmax2*abs(xold(2))+dcmax2*dxmax2);
    Dmax3 = 1/d3*(dkmax3*abs(xold(1))+dcmax3*dxmax3);
    Dmax4 = 1/d4*(dkmax4*abs(xold(2))+dcmax4*dxmax4);
    eta1 = Dmax1+eta01*abs(e01);
    eta2 = Dmax2+eta01*abs(e02);
    eta3 = Dmax3+eta01*abs(e03);
    eta4 = Dmax4+eta01*abs(e04);
    u1 = (d1)*(dtdxd(1,i) + kk1/d1*xold(1) - (eta01*e01+eta1*sign(e01)));
    u2 = (d2)*(dtdxd(2,i) + kk2/d2*xold(2) - (eta02*e02+eta2*sign(e02)));
    u3 = (d3)*(dtdxd(3,i) + kk3/d3*xold(1) - (eta03*e03+eta3*sign(e03)));
    u4 = (d4)*(dtdxd(4,i) + kk4/d4*xold(2) - (eta04*e04+eta4*sign(e04)));
    if u1 <=umint
        u1=umint;
    elseif u1>=umaxt
        u1=umaxt;
    end
    if u2 <=uminf
        u2=uminf;
    elseif u2>=umaxf
        u2=umaxf;
    end
    if u3 <=umint
        u3=umint;
    elseif u3>=umaxt
        u3=umaxt;
    end
    if u4 <=uminf
        u4=uminf;
    elseif u4>=umaxf
        u4=umaxf;
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
    if zold1<=-20
        zold1 = -20;
    elseif zold1>=20
        zold1 =20;
    end
    if zold2<=0
        zold2 = 0;
    elseif zold2>=20
        zold2 =20;
    end
    if zold3<=-20
        zold3 = -20;
    elseif zold3>=20
        zold3 =20;
    end
    if zold4<=0
        zold4 = 0;
    elseif zold4>=20
        zold4 =20;
    end
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

close all
clc
ylabelvec={'rad';'m';'rad';'m';};
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
spt =1;ept =1600;

figure(2)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),xest(spt:ept,i),LineStyle="-",LineWidth=1,Color='r')
    hold on
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(spt:ept,i),LineStyle="-",LineWidth=2,Color='b')
    hold on
    rmse_sim_asmc(i) = ...
        sqrt(sum((outputKnown.arc_state_wire(spt:ept,i)-xest(spt:ept,i)).^2) ... 
        /length(xest(spt:ept,i))); 
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
        legend('ASMC','Ref',Location='southeast')
    hold on
    end
   
%% simulation of SMART (Frontier AI work)
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
umaxt =30;umint=-30;
umaxf =30;uminf=0;
pdmax =15;pdmin=0;
%%%%%   NDOB  %%%%%%%
%%%% optimal
l1 = 1e1;eta01 = 1e1;    eta1 = 1e0;
destold1 =0;
destnew1=0;
intvar1 =0;intvar1old = 0;
pxold1=0;

l2 = 1e1;eta02 = 1e0;    eta2 = 1e0;
destold2 =0;
destnew2=0;
intvar2 =0;intvar2old= 0;
pxold2=0;

l3 = 1e1;eta03 = 1e1;    eta3 = 1e0;
destold3 =0;
destnew3=0;
intvar3 =0;intvar3old = 0;
pxold3=0;

l4 = 1e1;eta04 = 1e0;    eta4 = 1e0;
destold4 =0;
destnew4=0;
intvar4 =0;intvar4old = 0;
% pxold4=0;

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

    e01 = -xd(1,i) + xold(1);
    e02 = -xd(2,i) + xold(2);
    e03 = -xd(3,i) + xold(3);
    e04 = -xd(4,i) + xold(4);

    pxold1 = l1*xold(1);
%     intvar1new = funcRK4fintvar_onestate(h,l1,e01,eta1);
    intvar1new = funcRK4fintvar_onestate_v2(h,l1,e01,eta1,eta01);
    destnew1 = intvar1new + pxold1;


    pxold2 = l2*xold(2);
%     intvar2new = funcRK4fintvar_onestate(h,l2,e02,eta2);
    intvar2new = funcRK4fintvar_onestate_v2(h,l2,e02,eta2,eta02);
    destnew2 = intvar2new + pxold2;

        pxold3 = l3*xold(3);
%     intvar3new = funcRK4fintvar_onestate(h,l3,e03,eta3);
    intvar3new = funcRK4fintvar_onestate_v2(h,l3,e03,eta3,eta03);
    destnew3 = intvar3new + pxold3;


    pxold4 = l4*xold(4);
%     intvar4new = funcRK4fintvar_onestate(h,l4,e04,eta4);
    intvar4new = funcRK4fintvar_onestate_v2(h,l4,e04,eta4,eta04);
    destnew4 = intvar4new + pxold4;

%     Dmax1 = 1/d1*(dkmax1*abs(xold(1))+dcmax1*dxmax1);
%     Dmax2 = 1/d2*(dkmax2*abs(xold(2))+dcmax2*dxmax2);
%     Dmax3 = 1/d3*(dkmax3*abs(xold(1))+dcmax3*dxmax3);
% Dmax4 = 1/d4*(dkmax4*abs(xold(2))+dcmax4*dxmax4);

    u1 = d1*(dtdxd(1,i) + kk1/d1*xold(1) - (eta01*e01+eta1*sign(e01))-destnew1);
    u2 = d2*(dtdxd(2,i) + kk2/d2*xold(2) - (eta02*e02+eta2*sign(e02))-destnew2);
    u3 = d3*(dtdxd(3,i) + kk3/d3*xold(1) - (eta03*e03+eta3*sign(e03))-destnew3);
    u4 = d4*(dtdxd(4,i) + kk4/d4*xold(2) - (eta04*e04+eta4*sign(e04))-destnew4);
% %     u1 = (1/d1)*(xd(1,i)-kk1*xold(1)+eta01*e01+eta1*sign(e01));
% %     u2 = (1/d2)*(xd(2,i)-kk2*xold(2)+eta02*e02+eta2*sign(e02));
%     u3 = (1/d3)*(xd(3,i)-kk3*xold(1)+eta03*e03+eta3*sign(e03));
%     u4 = (1/d4)*(xd(4,i)-kk4*xold(2)+eta04*e04+eta4*sign(e04));
if u1 <=umint
        u1=umint;
    elseif u1>=umaxt
        u1=umaxt;
    end
    if u2 <=uminf
        u2=uminf;
    elseif u2>=umaxf
        u2=umaxf;
    end
    if u3 <=umint
        u3=umint;
    elseif u3>=umaxt
        u3=umaxt;
    end
    if u4 <=uminf
        u4=uminf;
    elseif u4>=umaxf
        u4=umaxf;
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
    if zold1<=-20
        zold1 = -20;
    elseif zold1>=20
        zold1 =20;
    end
    if zold2<=0
        zold2 = 0;
    elseif zold2>=20
        zold2 =20;
    end
    if zold3<=-20
        zold3 = -20;
    elseif zold3>=20
        zold3 =20;
    end
    if zold4<=0
        zold4 = 0;
    elseif zold4>=20
        zold4 =20;
    end
    pold1=pnew1;
    pold2=pnew2;
    pold3=pnew3;
    pold4=pnew4;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
    pm3(i)=pnew3;
    pm4(i)=pnew4;
    dest1(i) = destnew1;
    dest2(i) = destnew2;
    dest3(i) = destnew3;
    dest4(i) = destnew4;
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
sgtitle("NDOBSMC LPV at 40Hz")

figure(2)
subplot(4,1,1)
plot(dest1)
hold on
title('$\theta_1$','Interpreter','latex')
legend('d1',Location='south')
subplot(4,1,2)
plot(dest2)
hold on
title('$L_1$','Interpreter','latex')
legend('d2',Location='south')
subplot(4,1,3)
plot(dest3)
hold on
title('$\theta_2$','Interpreter','latex')
legend('d3',Location='south')
subplot(4,1,4)
plot(dest4)
hold on
title('$L_2$','Interpreter','latex')
legend('d4',Location='south')
sgtitle("NDOBSMC LPV at 40Hz")

figure(2)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),xest(spt:ept,i),LineStyle="-",LineWidth=1,Color='r')
    hold on
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(spt:ept,i),LineStyle="-",LineWidth=2,Color='b')
    hold on
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
        legend('ASMC','Ref',Location='southeast')
    hold on
    end

%% simulation of INASMC
title_array = {'$\theta_1$';'$L_1$';'$\theta_2$';'$L_2$'};
alpha = -0.9665; beta = 0.9698;
xold = [outputKnown.arc_state_wire(1,1:4)]';
h = 1/30

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

umaxt =30;umint=-30;
umaxf =30;uminf=0;
pdmax =20;pdmin=0;
%%%%%   NDOB  %%%%%%%

l1x = 1;eta01 = 20; etalx1 =0.1;
dtdestmax1 = 0.1;
destold1 =0;
destnew1=0;
intvar1new =0;dtdintvar1 = 0;
pxold1=0;

l2x = 0;eta02 = 1e0;   etalx2 =0.1;
dtdestmax2 = 0.1;
destold2 =0;
destnew2=0;
intvar2new =0;dtdintvar2 = 0;
pxold2=0;

l3x = 1;eta03 = 20;   etalx3 =0.1;
dtdestmax3 = 0.1;
destold3 =0;
destnew3=0;
intvar3new =0;dtdintvar3 = 0;
pxold3=0;

l4x = 0;eta04 = 1e0;    etalx4 =0.1;
dtdestmax4 = 0.1;
destold4 =0;
destnew4=0;
intvar4new =0;dtdintvar4 = 0;
pxold4=0;
l1 = 1;
l2 = 1;
l3 = 1;
l4 = 1;

        d1t0 = 1e-2; 
        d2t0 = 1e-2; 
        d3t0 = 1e-2; 
        d4t0 = 1e-2;
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

    e01 = -xd(1,i) + xold(1);
    e02 = -xd(2,i) + xold(2);
    e03 = -xd(3,i) + xold(3);
    e04 = -xd(4,i) + xold(4);

    pxold1 = l1x*xold(1) + etalx1*xold(1)^2*sign(xold(1));
    l1 = l1x + 2*etalx1*abs(xold(1));
    eta1 = d1t0*exp(-l1*h*i) + dtdestmax1/l1; 
%     intvar1new = funcRK4fintvar_onestate_improve_v2( h,l1,dtdxd(1,i),eta01,eta1,e01);
    intvar1new = funcRK4fintvar_onestate_improve_v3( h,l1,dtdxd(1,i),eta01,eta1,e01);
    destnew1 = intvar1new + pxold1;



    pxold2 = l2x*xold(2) + etalx2*xold(2)^2*sign(xold(2));
    l2 = l2x + 2*etalx1*abs(xold(2));
    eta2 = d2t0*exp(-l2*h*i) + dtdestmax2/l2; 
%     intvar2new = funcRK4fintvar_onestate_improve_v2( h,l2,dtdxd(2,i),eta02,eta2,e02);
    intvar2new = funcRK4fintvar_onestate_improve_v3( h,l2,dtdxd(2,i),eta02,eta2,e02);
    destnew2 = intvar2new + pxold2;

    pxold3 = l3x*xold(3) + etalx3*xold(3)^2*sign(xold(3));
    l3 = l3x + 2*etalx3*abs(xold(3));
     eta3 = d3t0*exp(-l3*h*i) + dtdestmax3/l3; 
%     intvar3new = funcRK4fintvar_onestate_improve_v2( h,l3,dtdxd(3,i),eta03,eta3,e03);
    intvar3new = funcRK4fintvar_onestate_improve_v3( h,l3,dtdxd(3,i),eta03,eta3,e03);
    destnew3 = intvar3new + pxold3;


    pxold4 = l4x*xold(4) + etalx4*xold(4)^2*sign(xold(4));
    l4 = l4x + 2*etalx4*abs(xold(4));
     eta4 = d4t0*exp(-l4*h*i) + dtdestmax4/l4;  
%     intvar4new = funcRK4fintvar_onestate_improve_v2( h,l4,dtdxd(4,i),eta04,eta4,e04);
    intvar4new = funcRK4fintvar_onestate_improve_v3( h,l4,dtdxd(4,i),eta04,eta4,e04);
    destnew4 = intvar4new + pxold4;


    u1 = -d1*(-dtdxd(1,i) + kk1/d1*xold(1) + (eta01*e01+eta1*sign(e01)) + destnew1);
    u2 = -d2*(-dtdxd(2,i) + kk2/d2*xold(2) + (eta02*e02+eta2*sign(e02))+ destnew2);
    u3 = -d3*(-dtdxd(3,i) + kk3/d3*xold(1) + (eta03*e03+eta3*sign(e03))+ destnew3);
    u4 = -d4*(-dtdxd(4,i) + kk4/d4*xold(2) + (eta04*e04+eta4*sign(e04))+ destnew4);

if u1 <=umint
        u1=umint;
    elseif u1>=umaxt
        u1=umaxt;
    end
    if u2 <=uminf
        u2=uminf;
    elseif u2>=umaxf
        u2=umaxf;
    end
    if u3 <=umint
        u3=umint;
    elseif u3>=umaxt
        u3=umaxt;
    end
    if u4 <=uminf
        u4=uminf;
    elseif u4>=umaxf
        u4=umaxf;
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
        if zold1<=-20
        zold1 = -20;
    elseif zold1>=20
        zold1 =20;
    end
    if zold2<=0
        zold2 = 0;
    elseif zold2>=20
        zold2 =20;
    end
    if zold3<=-20
        zold3 = -20;
    elseif zold3>=20
        zold3 =20;
    end
    if zold4<=0
        zold4 = 0;
    elseif zold4>=20
        zold4 =20;
    end
    pold1=pnew1;
    pold2=pnew2;
    pold3=pnew3;
    pold4=pnew4;
    pm1(i)=pnew1;
    pm2(i)=pnew2;
    pm3(i)=pnew3;
    pm4(i)=pnew4;

    dest1(i) = destnew1;
    dest2(i) = destnew2;
    dest3(i) = destnew3;
    dest4(i) = destnew4;
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

figure(2)
subplot(4,1,1)
plot(dest1)
hold on
title('$\theta_1$','Interpreter','latex')
legend('d1',Location='south')
subplot(4,1,2)
plot(dest2)
hold on
title('$L_1$','Interpreter','latex')
legend('d2',Location='south')
subplot(4,1,3)
plot(dest3)
hold on
title('$\theta_2$','Interpreter','latex')
legend('d3',Location='south')
subplot(4,1,4)
plot(dest4)
hold on
title('$L_2$','Interpreter','latex')
legend('d4',Location='south')
sgtitle("INDOBASMC LPV at 40Hz")

figure(3)
    for i =1:4
    subplot(2,2,i)
    plot(testData.time_stamp(spt:ept),xest(spt:ept,i),LineStyle="-",LineWidth=1,Color='r')
    hold on
    plot(testData.time_stamp(spt:ept),outputKnown.arc_state_wire(spt:ept,i),LineStyle="-",LineWidth=2,Color='b')
    hold on
    ylabel(ylabelvec{i})
    title(title_array{i},Interpreter="latex",FontSize=12)
    xlim([0,50])
    % ylim([0 20])
        legend('INASMC','Ref',Location='southeast')
    hold on
    end
%% open asmc fig file manully

obj_all = findobj(gcf,'Type','line');
ref_l2 = obj_all(1).YData;
asmc_l2 = obj_all(2).YData;

ref_theta2 = obj_all(3).YData;
asmc_theta2 = obj_all(4).YData;

ref_l1 = obj_all(5).YData;
asmc_l1 = obj_all(6).YData;

ref_theta1 = obj_all(7).YData;
asmc_theta1 = obj_all(8).YData;

rmse_sim_inasmc(1) = ...
        sqrt(sum((outputKnown.arc_state_wire(spt:ept,i)-xest(spt:ept,i)).^2) ... 
        /length(xest(spt:ept,i))); 