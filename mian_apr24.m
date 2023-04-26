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
par_set.Ts=1/60;

par_set.fz_a0 = (25/1000)*(60/1000);%m^2
par_set.tau_l0 =48/1000;%m

par_set.R1_stand_off = 0.05;% m
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
%%
%%% Use mocap for angle estimation  
%%% Use wire encoder for arc length estimation
%%% End %%%
testData =par_set.trial1;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z1=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial2;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z2=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial3;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z3=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial4;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z4=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial5;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z5=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial6;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z6=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');

testData =par_set.trial7;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,:);
z7=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');
z_total = merge(z1,z2,z3,z4,z5,z6,z7);
%%
st_pt = 1; ed_pt = length(testData.pm_psi);

Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0]';
Y_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

z = iddata(Y_id_array(:,1:2:end),U_id_array',par_set.Ts,'Name','2-segArm');
z.InputName = {'$\tau_1$';'$\tau_2$'};
z.InputUnit = {'$Nm$';'$Nm$'};
z.OutputName = {'$\theta_1$';'$\theta_2$'};
z.OutputUnit = {'$rad$','$rad$'};
present(z)
return
%%
close all
figure('Name', [z.Name ': tau input 1 -> Angular position output 1']);
plot(z1(:,1,1))
figure('Name', [z.Name ': f input 1 ->  arc length 1']);
plot(z1(:,2,2))
figure('Name', [z.Name ': tau input 2 -> Angular position output 2']);
plot(z1(:,3,3))
figure('Name', [z.Name ': f input 2 ->  arc length 2']);
plot(z1(:,4,4))
%% apr18 classical gprfit

testData =par_set.trial1;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

y_temp = [];X_temp = [];
y_temp = Y_id_array(:,1); 
X_temp = [output_struct.state_array(:,1:2)];
X_temp = output_struct.state_array(:,1);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
%% train the model
gpy1_1 = fitrgp(X_temp,y_temp);
gpy1_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy1_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
%%
predy1_1 = resubPredict(gpy1_1);
predy1_2 = resubPredict(gpy1_2);
predy1_3 = resubPredict(gpy1_3);
y_temp = Y_id_array(:,1); 
close all

figure(1)
plot(y_temp,'r.')
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
y_temp = [];X_temp = [];
y_temp = Y_id_array(:,2); 
X_temp = output_struct.state_array(:,3);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,2)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,2)];
%% 
gpy2_1 = fitrgp(X_temp,y_temp);
gpy2_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy2_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
%%
predy2_1 = resubPredict(gpy2_1);
predy2_2 = resubPredict(gpy2_2);
predy2_3 = resubPredict(gpy2_3);
y_temp = Y_id_array(:,2); 
close all

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
y_temp = [];X_temp = [];
y_temp = Y_id_array(:,3); 
X_temp = output_struct.state_array(:,5);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
%% train the model
gpy3_1 = fitrgp(X_temp,y_temp);
gpy3_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy3_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
%%
predy3_1 = resubPredict(gpy3_1);
predy3_2 = resubPredict(gpy3_2);
predy3_3 = resubPredict(gpy3_3);
y_temp = Y_id_array(:,3); 
close all

figure(3)
plot(y_temp,'r.')
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
y_temp = [];X_temp = [];
y_temp = Y_id_array(:,4); 
X_temp = output_struct.state_array(:,7);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,2)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,2)];
%% 
gpy4_1 = fitrgp(X_temp,y_temp);
gpy4_2 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.02);
gpy4_3 = fitrgp(X_temp,y_temp,'SigmaLowerBound',0.001);
%%
predy4_1 = resubPredict(gpy4_1);
predy4_2 = resubPredict(gpy4_2);
predy4_3 = resubPredict(gpy4_3);
y_temp = Y_id_array(:,4); 
close all

figure(4)
plot(y_temp,'r.')
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
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);
U_id_array =[]; Y_id_array = [];
Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array';
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

%%
close all
y_temp = [];X_temp = [];
y_temp = Y_id_array(:,1); 
X_temp = output_struct.state_array(:,1);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
predy1_1 = predict(gpy1_1,X_temp);
predy1_2 = predict(gpy1_2,X_temp);
predy1_3 = predict(gpy1_3,X_temp);
figure(1)
plot(y_temp,'r.')
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
y_temp = Y_id_array(:,2); 
X_temp = output_struct.state_array(:,3);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
predy2_1 = predict(gpy2_1,X_temp);
predy2_2 = predict(gpy2_2,X_temp);
predy2_3 = predict(gpy2_3,X_temp);
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

y_temp = [];X_temp = [];
y_temp = Y_id_array(:,3); 
X_temp = output_struct.state_array(:,5);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
predy3_1 = predict(gpy3_1,X_temp);
predy3_2 = predict(gpy3_2,X_temp);
predy3_3 = predict(gpy3_3,X_temp);
figure(3)
plot(y_temp,'r.')
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
y_temp = Y_id_array(:,4); 
X_temp = output_struct.state_array(:,7);
X_temp(:,2) = [0;output_struct.state_array(1:end-1,1)];
X_temp(:,3) = [0;0;output_struct.state_array(1:end-2,1)];
predy4_1 = predict(gpy4_1,X_temp);
predy4_2 = predict(gpy4_2,X_temp);
predy4_3 = predict(gpy4_3,X_temp);
figure(4)
plot(y_temp,'r.')
hold on
plot(predy4_1,'b.')
hold on
plot(predy4_2,'k.')
hold on
plot(predy4_3,'c.')
hold on
legend('exp','sig1e-2','sig2e-2','sig1e-3')
title('L2')