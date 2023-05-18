%% Greybox main
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
%% Rounge-Kuka 4th order
h = 0.0001; %prediction size
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
gp_input_vec = X_temp(1,:);
x0 = output_struct.state_array(1,:);
u_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0];
result_mat = [];
for ti = 1:10*3
    result_mat(ti,:) = x0;
%     temp_tspan = testData.time_stamp(ti):dt:testData.time_stamp(ti+1);
    temp_gp = zeros(4,1);
%     temp_gp= [predict(gpy1_1,gp_input_vec);...
%             predict(gpy2_1,gp_input_vec);...
%             predict(gpy3_1,gp_input_vec);
%             predict(gpy4_1,gp_input_vec)];
%     u = u_array(ti,:);
    u = u_array(1,:);
    temp_y = funcRK4GP2segODE_m(x0, u,temp_gp,h);
%     [temp_t,temp_y] = ode45(@(t,x) funcGP2segODE_m(t, x, u,temp_gp),temp_tspan,x0);
    % Update 
    temp_x = temp_y(:,1:2:end);
%     temp_roll = gp_input_vec(1,1:8);
%     gp_input_vec = [mean(temp_x,1),temp_roll];
    x0 = mean(temp_y,1);
    end
    close all

figure(1)
subplot(4,1,1)
plot(result_mat(:,1))
hold on
plot(output_struct.state_array(1:10,1))
subplot(4,1,2)
plot(result_mat(:,3))
hold on
% plot(output_struct.state_array(1:10,3))
subplot(4,1,3)
plot(result_mat(:,5))
hold on
% plot(output_struct.state_array(1:10,5))
subplot(4,1,4)
plot(result_mat(:,7))
hold on
% plot(output_struct.state_array(1:10,7))
%% Apr 26 x1 only
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,1)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,1)];
gpy1_1 = fitrgp(X_temp,y_temp,"FitMethod","exact","BasisFunction","constant");
gpy1_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy1_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);

y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,2)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,3);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,3)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,3)];
gpy2_1 = fitrgp(X_temp,y_temp,"FitMethod","exact","BasisFunction","constant");
gpy2_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy2_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);

y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,3)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,5);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,5)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,5)];
gpy3_1 = fitrgp(X_temp,y_temp,"FitMethod","exact","BasisFunction","constant");
gpy3_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy3_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);

y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,4)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,7);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,7)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,7)];
gpy4_1 = fitrgp(X_temp,y_temp,"FitMethod","exact","BasisFunction","constant");
gpy4_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy4_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
%%
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,1)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,1)];

close all
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,1)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,1)];
predy1_1 = predict(gpy1_1,X_temp);
predy1_2 = predict(gpy1_2,X_temp);
predy1_3 = predict(gpy1_3,X_temp);
figure(1)
plot(y_temp,'r')
hold on
plot(predy1_1,'b.')
hold on
plot(predy1_2,'k.')
hold on
plot(predy1_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta1')


y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,2)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,3);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,3)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,3)];
predy2_1 = predict(gpy2_1,X_temp);
predy2_2 = predict(gpy2_2,X_temp);
predy2_3 = predict(gpy2_3,X_temp);
figure(2)
plot(y_temp,'r')
hold on
plot(predy2_1,'b.')
hold on
plot(predy2_2,'k.')
hold on
plot(predy2_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L1')

y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,3)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,5);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,5)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,5)];
predy3_1 = predict(gpy3_1,X_temp);
predy3_2 = predict(gpy3_2,X_temp);
predy3_3 = predict(gpy3_3,X_temp);
figure(3)
plot(y_temp,'r')
hold on
plot(predy3_1,'b.')
hold on
plot(predy3_2,'k.')
hold on
plot(predy3_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta2')


y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,4)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,7);
X_temp(:,2) = [0;output_struct.state_array(st_pt:ed_pt-1,7)];
X_temp(:,3) = [0; 0; output_struct.state_array(st_pt:ed_pt-2,7)];
predy4_1 = predict(gpy4_1,X_temp);
predy4_2 = predict(gpy4_2,X_temp);
predy4_3 = predict(gpy4_3,X_temp);
figure(4)
plot(y_temp,'r')
hold on
plot(predy4_1,'b.')
hold on
plot(predy4_2,'k.')
hold on
plot(predy4_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L2')
%% Apr 26 full state estimation
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
%%
gpy1_1 = fitrgp(X_temp,y_temp,"FitMethod","exact","BasisFunction","constant");
gpy1_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy1_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
beep;
%%
predy1_1 = resubPredict(gpy1_1);
predy1_2 = resubPredict(gpy1_2);
predy1_3 = resubPredict(gpy1_3);
figure(1)
plot(y_temp,'r')
hold on
plot(predy1_1,'b.')
hold on
plot(predy1_2,'k.')
hold on
plot(predy1_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta1')
%% validataion
testData =par_set.trial2;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
predy1_1 = predict(gpy1_1,X_temp);
predy1_2 = predict(gpy1_2,X_temp);
predy1_3 = predict(gpy1_3,X_temp);
figure(1)
plot(y_temp,'r')
hold on
plot(predy1_1,'b.')
hold on
plot(predy1_2,'k.')
hold on
plot(predy1_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta1')
%%
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,2)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
gpy2_1 = fitrgp(X_temp,y_temp);
gpy2_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy2_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
predy2_1 = resubPredict(gpy2_1);
predy2_2 = resubPredict(gpy2_2);
predy2_3 = resubPredict(gpy2_3);
y_temp = Y_id_array(:,2);
figure(2)
plot(y_temp,'r.')
hold on
plot(predy2_1,'b.')
hold on
plot(predy2_2,'k.')
hold on
plot(predy2_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L1')
%%
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,3)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
gpy3_1 = fitrgp(X_temp,y_temp);
gpy3_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy3_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
predy3_1 = resubPredict(gpy3_1);
predy3_2 = resubPredict(gpy3_2);
predy3_3 = resubPredict(gpy3_3);
y_temp = Y_id_array(:,3);
figure(3)
plot(y_temp,'r')
hold on
plot(predy3_1,'b.')
hold on
plot(predy3_2,'k.')
hold on
plot(predy3_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta2')
%%
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,4)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
gpy4_1 = fitrgp(X_temp,y_temp);
gpy4_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy4_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
predy4_1 = resubPredict(gpy4_1);
predy4_2 = resubPredict(gpy4_2);
predy4_3 = resubPredict(gpy4_3);
y_temp = Y_id_array(:,4);
close all

figure(4)
plot(y_temp,'r')
hold on
plot(predy4_1,'b.')
hold on
plot(predy4_2,'k.')
hold on
plot(predy4_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L2')
%%
testData =par_set.trial2;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
%%
close all
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
predy1_1 = predict(gpy1_1,X_temp);
predy1_2 = predict(gpy1_2,X_temp);
predy1_3 = predict(gpy1_3,X_temp);
figure(1)
plot(y_temp,'r')
hold on
plot(predy1_1,'b.')
hold on
plot(predy1_2,'k.')
hold on
plot(predy1_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta1')


y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,2)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
predy2_1 = predict(gpy2_1,X_temp);
predy2_2 = predict(gpy2_2,X_temp);
predy2_3 = predict(gpy2_3,X_temp);
figure(2)
plot(y_temp,'r')
hold on
plot(predy2_1,'b.')
hold on
plot(predy2_2,'k.')
hold on
plot(predy2_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L1')

y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,3)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
predy3_1 = predict(gpy3_1,X_temp);
predy3_2 = predict(gpy3_2,X_temp);
predy3_3 = predict(gpy3_3,X_temp);
figure(3)
plot(y_temp,'r')
hold on
plot(predy3_1,'b.')
hold on
plot(predy3_2,'k.')
hold on
plot(predy3_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('theta2')


y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,4)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
predy4_1 = predict(gpy4_1,X_temp);
predy4_2 = predict(gpy4_2,X_temp);
predy4_3 = predict(gpy4_3,X_temp);
figure(4)
plot(y_temp,'r')
hold on
plot(predy4_1,'b.')
hold on
plot(predy4_2,'k.')
hold on
plot(predy4_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L2')
%% Simulation 
dt = 0.001; %sec
testData =par_set.trial1;
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = int64(par_set.train_ratio * length(testData.pm_psi));
Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0] - output_struct.mcg_array(st_pt:ed_pt)';
y_temp = [];X_temp = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y_temp =filter(b,a,Y_id_array(:,1)) ;
X_temp = output_struct.state_array(st_pt:ed_pt,1:2:end);
X_temp(:,5:8) = [0, 0, 0, 0;output_struct.state_array(st_pt:ed_pt-1,1:2:end)];
X_temp(:,9:12) = [0, 0, 0, 0; 0, 0, 0, 0; output_struct.state_array(st_pt:ed_pt-2,1:2:end)];
gp_input_vec = X_temp(1,:);
x0 = output_struct.state_array(1,:);
u_array= [output_struct.u_pm_pa(st_pt:ed_pt,1)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,2)*par_set.fz_a0,...
    output_struct.u_pm_pa(st_pt:ed_pt,3)*par_set.fz_a0*par_set.tau_l0,...
    output_struct.u_pm_pa(st_pt:ed_pt,4)*par_set.fz_a0];
% result_mat =zeros(length(X_temp)-1,8);
result_mat =[];
% for ti = 1:length(testData.time_stamp)-1
    for ti = 1:10
    result_mat(ti,:) = x0;
    temp_tspan = testData.time_stamp(ti):dt:testData.time_stamp(ti+1);
    temp_gp = [];
    temp_gp= [predict(gpy1_1,gp_input_vec);...
            predict(gpy2_1,gp_input_vec);...
            predict(gpy3_1,gp_input_vec);
            predict(gpy4_1,gp_input_vec)];
    u = u_array(ti,:);
    [temp_t,temp_y] = ode45(@(t,x) funcGP2segODE_m(t, x, u,temp_gp),temp_tspan,x0);
    % Update 
    temp_x = temp_y(:,1:2:end);
    temp_roll = gp_input_vec(1,1:8);
    gp_input_vec = [mean(temp_x,1),temp_roll];
    x0 = mean(temp_y,1);
    end
    close all

figure(1)
subplot(4,1,1)
plot(result_mat(:,1))
hold on
plot(output_struct.state_array(1:10,1))
subplot(4,1,2)
plot(result_mat(:,3))
hold on
plot(output_struct.state_array(1:10,3))
subplot(4,1,3)
plot(result_mat(:,5))
hold on
plot(output_struct.state_array(1:10,5))
subplot(4,1,4)
plot(result_mat(:,7))
hold on
plot(output_struct.state_array(1:10,7))
