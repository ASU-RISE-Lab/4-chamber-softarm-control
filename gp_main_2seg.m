%% GP for sysid 0 order
% run ml_ral_main to get testdata
%%
timestamp=testData.time_stamp;%sec
Ts=timestamp(2)-timestamp(1);
%%%%% calculate tau, fz, theta and lc
s1.l_t0 = 205.7796;s1.r_t0 = 209.0839; 
s2.l_t0 = 223.6281;s2.r_t0 = 222.3360;
r0 = 43;
green_t0 = 31.92;yellow_t0 = 46.33; purple_t0 =21.99;
s1.red_l = s1.l_t0 - green_t0;
s1.red_r = s1.r_t0 - green_t0;
s2.red_l = s2.l_t0 - yellow_t0 - purple_t0 - green_t0;
s2.red_r = s2.r_t0 - yellow_t0 - purple_t0 - green_t0;

s1.l_t = testData.enco_volts(:,1)/5*1000 - s1.red_l;
s1.r_t = testData.enco_volts(:,2)/5*1000 - s1.red_r;

s2.l_t = testData.enco_volts(:,4)/5*1000 - purple_t0 - s1.l_t - s2.red_l;
s2.r_t = testData.enco_volts(:,3)/5*1000 - purple_t0 - s1.r_t - s2.red_r;

s1.theta_wire_rad = (s1.r_t - s1.l_t)/r0;
s1.l_wire_mm = (s1.r_t + s1.l_t)/2;

s2.theta_wire_rad = (s2.r_t - s2.l_t)/r0;
s2.l_wire_mm = (s2.r_t + s2.l_t)/2;

tauy1 = testData.pd_psi(:,1) - testData.pd_psi(:,2);
fz1 = testData.pd_psi(:,1) + testData.pd_psi(:,2) + testData.pd_psi(:,3);
tauy2 = testData.pd_psi(:,4) - testData.pd_psi(:,5);
fz2 = testData.pd_psi(:,4) + testData.pd_psi(:,5) + testData.pd_psi(:,6);

input1 = tauy1; % psi
input2 = fz1; % psi
input3 = tauy2; % psi
input4 = fz2;
% top xyz with respect to local base mm
output1 =s1.theta_wire_rad;
output2 =s1.l_wire_mm;
output3 =s2.theta_wire_rad;
output4 =s2.l_wire_mm;
% outputs_mm_per_s = testData.tip_exp(:,2:4);
% outputs_mm_per_s(2:end,:) = (testData.tip_exp(2:end,2:4)-testData.tip_exp(1:end-1,2:4))*1000/Ts;
% outputs_mm_per_s(1,:)=[0,0,0];
return
%% 0 order gp X = f(U), X= [x,y,z], U= [pd1,pd2,pd3] works fine
gp_y=[output1,output2, output3, output4];
gp_u=[input1, input2, input3,input4];
gp_obj_0_order=iddata(gp_y,gp_u,Ts);
set(gp_obj_0_order,'InputName',{'tauy1';'fz1';'tauy2';'fz2'},'OutputName',{'a1','l1','a2','l2'});
gp_obj_0_order.InputUnit = {'psi';'psi';'psi';'psi'};
gp_obj_0_order.OutputUnit = {'rad';'mm';'rad';'mm'};
get(gp_obj_0_order)
%% Validation set
testData = par_set.trial2;
timestamp=testData.time_stamp%sec
Ts=timestamp(2)-timestamp(1);
%%%%% calculate tau, fz, theta and lc
s1.l_t0 = 205.7796;s1.r_t0 = 209.0839; 
s2.l_t0 = 223.6281;s2.r_t0 = 222.3360;
r0 = 43;
green_t0 = 31.92;yellow_t0 = 46.33; purple_t0 =21.99;
s1.red_l = s1.l_t0 - green_t0;
s1.red_r = s1.r_t0 - green_t0;
s2.red_l = s2.l_t0 - yellow_t0 - purple_t0 - green_t0;
s2.red_r = s2.r_t0 - yellow_t0 - purple_t0 - green_t0;

s1.l_t = testData.enco_volts(:,1)/5*1000 - s1.red_l;
s1.r_t = testData.enco_volts(:,2)/5*1000 - s1.red_r;

s2.l_t = testData.enco_volts(:,4)/5*1000 - purple_t0 - s1.l_t - s2.red_l;
s2.r_t = testData.enco_volts(:,3)/5*1000 - purple_t0 - s1.r_t - s2.red_r;

s1.theta_wire_rad = (s1.r_t - s1.l_t)/r0;
s1.l_wire_mm = (s1.r_t + s1.l_t)/2;

s2.theta_wire_rad = (s2.r_t - s2.l_t)/r0;
s2.l_wire_mm = (s2.r_t + s2.l_t)/2;

tauy1 = testData.pd_psi(:,1) - testData.pd_psi(:,2);
fz1 = testData.pd_psi(:,1) + testData.pd_psi(:,2) + testData.pd_psi(:,3);
tauy2 = testData.pd_psi(:,4) - testData.pd_psi(:,5);
fz2 = testData.pd_psi(:,4) + testData.pd_psi(:,5) + testData.pd_psi(:,6);

input1 = tauy1; % psi
input2 = fz1; % psi
input3 = tauy2; % psi
input4 = fz2;
% top xyz with respect to local base mm
output1 =s1.theta_wire_rad;
output2 =s1.l_wire_mm;
output3 =s2.theta_wire_rad;
output4 =s2.l_wire_mm;
% outputs_mm_per_s = testData.tip_exp(:,2:4);
% outputs_mm_per_s(2:end,:) = (testData.tip_exp(2:end,2:4)-testData.tip_exp(1:end-1,2:4))*1000/Ts;
% outputs_mm_per_s(1,:)=[0,0,0];
% 0 order gp X = f(U), X= [x,y,z], U= [pd1,pd2,pd3] works fine
gp_y=[output1,output2, output3, output4];
gp_u=[input1, input2, input3,input4];
gp_val_0_order=iddata(gp_y,gp_u,Ts);
set(gp_val_0_order,'InputName',{'tauy1';'fz1';'tauy2';'fz2'},'OutputName',{'a1','l1','a2','l2'});
gp_val_0_order.InputUnit = {'psi';'psi';'psi';'psi'};
gp_val_0_order.OutputUnit = {'rad';'mm';'rad';'mm'};
get(gp_val_0_order)
return
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


