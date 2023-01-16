%% GP for sysid 0 order
close all;clear all;clc
load("opensim.mat")
testData=opensim;
%%
timestamp=[];
timestamp=testData.pd_MPa(:,1); % sec
Ts=timestamp(2)-timestamp(1);
input1=testData.pd_psi(:,2); % psi
input2=testData.pd_psi(:,3); % psi
input3=testData.pd_psi(:,4); % psi
inputs_raw=testData.pd_MPa(:,2:4);
% top xyz with respect to local base mm
output_x=testData.tip_exp(:,2)*1000;
output_y=testData.tip_exp(:,3)*1000;
output_z=testData.tip_exp(:,4)*1000;
outputs_mm = testData.tip_exp(:,2:4)*1000;
outputs_mm_per_s = testData.tip_exp(:,2:4);
outputs_mm_per_s(2:end,:) = (testData.tip_exp(2:end,2:4)-testData.tip_exp(1:end-1,2:4))*1000/Ts;
outputs_mm_per_s(1,:)=[0,0,0];
%% 0 order gp X = f(U), X= [x,y,z], U= [pd1,pd2,pd3] works fine
gp_y=outputs_mm;
gp_u=inputs_raw;
gp_obj_0_order=iddata(gp_y,gp_u,Ts);
set(gp_obj_0_order,'InputName',{'pd1';'pd2';'pd3'},'OutputName',{'x','y','z'});
gp_obj_0_order.InputUnit = {'psi';'psi';'psi'};
gp_obj_0_order.OutputUnit = {'mm';'mm';'mm'};
get(gp_obj_0_order)
%% 1st order gp  X(k) = f(X(k-1),U), X= [x,y,z,vx,vy,vz], U= [pd1,pd2,pd3]
%%%% main part
gp_y=[outputs_mm,outputs_mm_per_s];
gp_u=[outputs_mm,outputs_mm_per_s,inputs_raw];
gp_obj_1st_order=iddata(gp_y,gp_u,Ts);
% set(gp_obj_1st_order,'InputName',{'x';'y';'z';'pd1';'pd2';'pd3'},'OutputName',{'vx','vy','vz'});
% gp_obj_1st_order.InputUnit = {'mm';'mm';'mm';'psi';'psi';'psi'};
% gp_obj_1st_order.OutputUnit = {'mm/s';'mm/s';'mm/s'};
get(gp_obj_1st_order)

%% 2nd order gp  dot(X) = f(X,U), X= [x,y,z,vx,vy,vz], U= [pd1,pd2,pd3]
gp_y=[outputs_mm_per_s];
gp_u=[outputs_mm,inputs_raw];
gp_obj_1st_order=iddata(gp_y,gp_u,Ts);
set(gp_obj_1st_order,'InputName',{'x';'y';'z';'pd1';'pd2';'pd3'},'OutputName',{'vx','vy','vz'});
gp_obj_1st_order.InputUnit = {'mm';'mm';'mm';'psi';'psi';'psi'};
gp_obj_1st_order.OutputUnit = {'mm/s';'mm/s';'mm/s'};
get(gp_obj_1st_order)
%%
clc
output= testData.theta_deg(1:1000);
data_set=table([1:1:1000]',input1(1:1000),output+180,'VariableNames',{'Time Steps','Input pressure setpoint(psi)','Output Angle (deg)'})


