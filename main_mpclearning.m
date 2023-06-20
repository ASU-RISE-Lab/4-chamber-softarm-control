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
%% ode fix 1 seg 3 link
par_set.EOM=1;
par_set = funcEOMbaseFrame1seg_v2(par_set);

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
%% x xdot u 
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
fkResult = funcCompuFK_v1(mocapResult.state_array,par_set.Ti);
temp_a(1,:) = zeros(1,8);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,:);
nn_pred = [temp_a,testData.pd_psi];
nn_resp = mocapResult.state_array;
%% x xdot u 
testData = par_set.trial1;
mocapResult = funcComputeStateVar_v1(testData,par_set);

temp_a(1,:) = zeros(1,8);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,:);
val_pred = [temp_a,testData.pd_psi];
val_resp = mocapResult.state_array;
%%% chose diff nnMode
obj_test = nnMod1.Network;
yn = sim(obj_test,val_pred');
close all
figure(1)
subplot(4,1,1)
plot(yn(1,:))
hold on
plot(val_resp(:,1))
legend('sim','tru')
subplot(4,1,2)
plot(yn(3,:))
hold on
plot(val_resp(:,3))
subplot(4,1,3)
plot(yn(5,:))
hold on
plot(val_resp(:,5))
subplot(4,1,4)
plot(yn(7,:))
hold on
plot(val_resp(:,7))
%% x pm u
nn_pred =[]; nn_resp =[];temp_a = [];
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
temp_a(1,:) = zeros(1,4);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,1:2:end);
nn_pred = [temp_a,testData.pd_psi];
nn_resp = [mocapResult.state_array(:,1:2:end),testData.pm_psi];

testData = par_set.trial1;
mocapResult = funcComputeStateVar_v1(testData,par_set);temp_a(1,:) = zeros(1,4);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,1:2:end);
val_pred = [temp_a,testData.pd_psi];
val_resp = [mocapResult.state_array(:,1:2:end),testData.pm_psi];
%%% chose diff mode
obj_test = nnMod3_1.Network;
yn = sim(obj_test,val_pred');
close all
figure(1)
title1 = {'theta1','l1','theta2','l2'};
for i =1:4
subplot(4,1,i)
plot(yn(i,:))
hold on
plot(val_resp(:,i))
legend('sim','tru')
title(title1{i})
end
figure(2)
for i =1:6
subplot(6,1,i)
plot(yn(i+4,:))
hold on
plot(val_resp(:,i+4))
legend('sim','tru')
end
title('pm')
%% Script testing
nn_pred =[]; nn_resp =[];temp_a = [];
testData = par_set.trial2;
mocapResult = funcComputeStateVar_v1(testData,par_set);
temp_a(1,:) = zeros(1,4);
temp_a(2:length(mocapResult.state_array),:)= mocapResult.state_array(1:end-1,1:2:end);
nn_pred = [temp_a,testData.pd_psi];
nn_resp = [mocapResult.state_array(:,1:2:end),testData.pm_psi];

TsNSS = 0;
obj = idNeuralStateSpace(NumInputs=10,NumOutputs=10,Ts=TsNSS);
obj.StateNetwork = createMLPNetwork(obj,'state',LayerSizes = [64 64 64]);
trainData = iddata
options = nssTrainingOptions('adam');
options.MaxEpochs = 1000;


%%

obj = nlssest(U,M,obj,options,'UseLastExperimentForValidation',true);
