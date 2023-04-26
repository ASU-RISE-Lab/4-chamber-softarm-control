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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
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
U_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);
z7=iddata(Y_id_array,U_id_array,par_set.Ts,'Name','2-segArm');
z_total_2 = merge(z1,z2,z3,z4,z5,z6,z7);
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
%%
FileName      = 'func_base2segODE_m';       % File describing the model structure.
Order         = [2 2 4];           % Model orders [ny nu nx].
Parameters    = [9.182;11.79; 6.263;4.7];         % Initial parameters. Np = 3*4
InitialStates = zeros(4,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'Arm');
present(nlgr)
compare(nlgr,z)
return

%%
testData =par_set.trial2;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);

Y_id_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;               output_struct.u_pm_pa(:,2)*par_set.fz_a0,...              output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...              output_struct.u_pm_pa(:,4)*par_set.fz_a0] - output_struct.mcg_array;
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]';
Y_id_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

z = iddata(Y_id_array(:,1:2:end),U_id_array',par_set.Ts,'Name','2-segArm');
z.InputName = {'$\tau_1$','$f1$','$\tau_2$','$f2$'};
z.InputUnit = {'$Nm$';'$N$';'$Nm$';'$N$'};
z.OutputName = {'$\theta_1$';'$\theta_2$'};
z.OutputUnit = {'$rad$','$rad$'};
present(z)
FileName      = 'func_staticlc2segODE_m';       % File describing the model structure.
Order         = [2 4 4];           % Model orders [ny nu nx].
Parameters    = [9.182;11.79; 6.263;4.7];         % Initial parameters. Np = 3*4
InitialStates = zeros(4,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'Arm');
present(nlgr)
compare(nlgr,z)
return
%%
opt = nlgreyestOptions('Display', 'on');
nlgr1 = nlgreyest(z, nlgr, opt);
return
%%
Parameters    = [9.182;11.79;9.182;11.79;...
                6.263;4.7;9.182;11.79];         % Initial parameters. Np = 3*4
FileName      = 'func2segODE_m';       % File describing the model structure.
Order         = [2 2 4];           % Model orders [ny nu nx].
Parameters    = [9.182;11.79;9.182;11.79;...
                6.263;4.7;9.182;11.79];         % Initial parameters. Np = 3*4
InitialStates = zeros(8,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.

present(nlgr);
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(z, nlgr, opt);
%% Learning part
testData =par_set.trial1;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);

U_id_array = [];Y_id_array=[];
y_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
U_id_array = output_struct.state_array(:,1:2:end);

z1 = iddata(y_array(:,1),U_id_array(:,1),par_set.Ts,'Name','2-segArm');
nn_pred1 = U_id_array(:,1);
nn_resp1 = y_array(:,1);

nn_pred2 = U_id_array(:,2)/0.2;
nn_resp2 = y_array(:,2)/(40*6894.76*par_set.fz_a0);

nn_pred3 = U_id_array(:,3);
nn_resp3 = y_array(:,3);
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
close all

figure(1)
plot(y_temp,'r.')
hold on
plot(predy1_1,'b')
hold on
plot(predy1_2,'k')
hold on
plot(predy1_3,'c')
hold on