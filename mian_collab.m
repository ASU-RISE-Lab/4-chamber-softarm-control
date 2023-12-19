
clc;close all;clear all;
%%%% Initialize the system %%%
par_set=[];
par_set.flag_read_exp=1;
par_set.Ts=1/30;
par_set.fz_a0 = (25/1000)*(60/1000);%m^2 contact area of pillow
par_set.tau_l0 =48/1000;%m distance between center of pillow to rotation axis
par_set.R1_stand_off = 0.05;% m
par_set.enco_volt_p0 = [1.4612    1.4975    1.8306    1.8261];% V wire encoder readings at mid p=2 psi Dec 1st;
% par_set.enco_volt_p0 = [1.4692    1.5103    1.8416    1.8475];% V wire encoder readings at mid p=2 psi;
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
close all
testData =[];
testData = par_set.trial1;
figure(1)
plot(testData.pd_psi(:,1))
xyz0=testData.rigid_1_pose(1,:)
par_set.xyz0_2psi = xyz0;

s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;

figure(2)
scatter3(s1.pose(:,1),s1.pose(:,2),s1.pose(:,3))
hold on 
scatter3(s2.pose(:,1),s2.pose(:,2),s2.pose(:,3))
%%
testData =[];
testData = par_set.trial1;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp1 = temp_struct;

testData = par_set.trial2;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp2 = temp_struct;

testData = par_set.trial3;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp3 = temp_struct;

testData = par_set.trial4;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp4 = temp_struct;

testData = par_set.trial5;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp5 = temp_struct;

testData = par_set.trial6;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp6 = temp_struct;

testData = par_set.trial7;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp7 = temp_struct;

testData = par_set.trial8;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp8 = temp_struct;

testData = par_set.trial9;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp9 = temp_struct;

testData = par_set.trial10;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp10 = temp_struct;

testData = par_set.trial11;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp11= temp_struct;

testData = par_set.trial12;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp12 = temp_struct;

testData = par_set.trial13;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp13 = temp_struct;

testData = par_set.trial14;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp14 = temp_struct;

testData = par_set.trial15;
temp_struct=[];
temp_struct.s1.pose = testData.rigid_2_pose-par_set.xyz0_2psi;
temp_struct.s2.pose = testData.rigid_3_pose-par_set.xyz0_2psi;
temp_struct.s1.input_ref_unit_psi = testData.pd_psi(:,1:2);
temp_struct.s2.input_ref_unit_psi = testData.pd_psi(:,4:5);
exp15 = temp_struct;