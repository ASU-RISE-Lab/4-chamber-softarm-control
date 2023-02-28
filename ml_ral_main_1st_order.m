%% Main function use data in 2023
%%% Major chanages:
%%%
clear all
close all
clc
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
fprintf('System initialization done \n')
%% EOM per element
par_set.EOM = 0
if par_set.EOM ==1
    par_set = funcEOMbaseFrame2seg_v2(par_set);
end
simplify(par_set.rigid_3_htm)
return
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
%% PLot raw data
testData = par_set.trial1;
if par_set.flag_plot_rawData == 1
    funcPlotRawData(testData)
end
%% EOM deriviation with 6 joints virtual rigid robot
% par_set.EOM = 1
if par_set.EOM ==1
    par_set = funcEOMbaseFrame2seg_v2(par_set);
end

%% filtering velocity and acc
testData =par_set.trial1;
output_struct = funcKnownTerm_v3(testData);
st_pt = 500; ed_pt = length(testData.pm_psi);


tauy1 = testData.pm_Pa(st_pt:ed_pt,1) - testData.pm_Pa(st_pt:ed_pt,2);
fz1 = testData.pm_Pa(st_pt:ed_pt,1) + testData.pm_Pa(st_pt:ed_pt,2) + 2*testData.pm_Pa(st_pt:ed_pt,3);
tauy2 = testData.pm_Pa(st_pt:ed_pt,4) - testData.pm_Pa(st_pt:ed_pt,5);
fz2 = testData.pm_Pa(st_pt:ed_pt,4) + testData.pm_Pa(st_pt:ed_pt,5) + 2*testData.pm_Pa(st_pt:ed_pt,6);

input_array= [tauy1*par_set.fz_a0*par_set.tau_l0,fz1*par_set.fz_a0...
             ,tauy2*par_set.fz_a0*par_set.tau_l0,fz2*par_set.fz_a0]';
output_array = output_struct.output_array(:,st_pt:ed_pt);
state_array = output_struct.state_array(st_pt:ed_pt,:);

output_array_1st = output_struct.Gi(:,st_pt:ed_pt) + output_struct.Ci(:,st_pt:ed_pt);
close all
for i  =1 :4
figure(i)
subplot(4,1,1)
plot(output_array(i,:))
hold on
plot((output_array(i,:)))
ylabel('y_i')
title("i =" + i)
hold on
subplot(4,1,2)
plot(input_array(i,:))
ylabel('u_i')
hold on
subplot(4,1,3)
plot(state_array(:,2*i-1))
hold on
plot((state_array(:,2*i-1)))
ylabel('xi')
hold on
subplot(4,1,4)
plot(state_array(:,2*i))
hold on
plot((state_array(:,2*i)))
ylabel('xi+1')
hold on
end
figure(5)
subplot(3,1,1)
title("i =" + i)
plot(output_struct.Mi(i,st_pt:ed_pt))
ylabel('Mi')
hold on
title("i =" + i)
subplot(3,1,2)
plot(output_struct.Ci(i,st_pt:ed_pt))
ylabel('Ci')
hold on
subplot(3,1,3)
plot(output_struct.Gi(i,st_pt:ed_pt))
ylabel('Gi')
hold on
%% 2nd order LG
Ts = par_set.Ts;
st_pt = 1;
i =1
temp_y=output_array(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj1 =iddata(temp_y,temp_u,Ts);
lg1 = regress(temp_y,temp_u)
i =2
temp_y=output_array(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj2 =iddata(temp_y,temp_u,Ts);
lg2 = regress(temp_y,temp_u)
i =3
temp_y=output_array(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj3 =iddata(temp_y,temp_u,Ts);
lg3 = regress(temp_y,temp_u)
i =4
temp_y=output_array(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj4 =iddata(temp_y,temp_u,Ts);
lg4 = regress(temp_y,temp_u)
%% 1st order LG
Ts = par_set.Ts;
st_pt = 1;
i =1

temp_y=output_array_1st(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj1 =iddata(temp_y,temp_u,Ts);
lg1 = regress(temp_y,temp_u)
i =2
temp_y=output_array_1st(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj2 =iddata(temp_y,temp_u,Ts);
lg2 = regress(temp_y,temp_u)
i =3
temp_y=output_array_1st(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj3 =iddata(temp_y,temp_u,Ts);
lg3 = regress(temp_y,temp_u)
i =4
temp_y=output_array_1st(i,st_pt:end)';
temp_u=[input_array(i,st_pt:end)',state_array(st_pt:end,2*i-1),state_array(st_pt:end,2*i)];
obj4 =iddata(temp_y,temp_u,Ts);
lg4 = regress(temp_y,temp_u)
%% 1st order LG no alpha term
Ts = par_set.Ts;
st_pt = 1;

i =1
temp_u=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y=state_array(st_pt:end,2*i-1);
temp_y2 = state_array(st_pt:end,2*i);
obj1 =iddata(temp_y,temp_u,Ts);

i =2
temp_u=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y=state_array(st_pt:end,2*i-1);
obj2 =iddata(temp_y,temp_u,Ts);

i =3
temp_u=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y=state_array(st_pt:end,2*i-1);
obj3 =iddata(temp_y,temp_u,Ts);

i =4
temp_u=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y=state_array(st_pt:end,2*i-1);
obj4 =iddata(temp_y,temp_u,Ts);

%% curve fitting
i =4
temp_y=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y2=0.25*input_array(i,st_pt:end)'-output_array(i,st_pt:end)';
temp_x=state_array(st_pt:end,2*i-1);
temp_xdot = state_array(st_pt:end,2*i);
temp_u = [temp_x,temp_xdot];
obj4 =iddata(temp_y,temp_u,Ts);
lg4 = regress(temp_y,temp_u)
%%
i =2
temp_y=input_array(i,st_pt:end)'-output_array_1st(i,st_pt:end)';
temp_y2=0.25*input_array(i,st_pt:end)'-output_array(i,st_pt:end)';
temp_x=state_array(st_pt:end,2*i-1);
temp_xdot = state_array(st_pt:end,2*i);
temp_u = [temp_x,temp_xdot];
obj2 =iddata(temp_y,temp_u,Ts);
lg2 = regress(temp_y,temp_u)
%%
close all
figure(1)
i =4
subplot(4,1,1)
plot(output_array(i,:))
hold on
plot((output_array(i,:)))
ylabel('y_i')
title("i =" + i)
hold on
subplot(4,1,2)
plot(input_array(i,:))
ylabel('u_i')
hold on
subplot(4,1,3)
plot(state_array(:,2*i-1))
hold on
plot((state_array(:,2*i-1)))
ylabel('xi')
hold on
subplot(4,1,4)
plot(state_array(:,2*i))
hold on
plot((state_array(:,2*i)))
ylabel('xi+1')
hold on

figure(2)
subplot(3,1,1)
title("i =" + i)
plot(output_struct.Mi(i,:))
ylabel('Mi')
hold on
title("i =" + i)
subplot(3,1,2)
plot(output_struct.Ci(i,:))
ylabel('Ci')
hold on
subplot(3,1,3)
plot(output_struct.Gi(i,:))
ylabel('Gi')
hold on
Ts = par_set.Ts;
gp_y=-input_array'-output_array';
gp_u=[input_array',state_array];
gp_obj=iddata(gp_y,gp_u,Ts);

gp_y1=-input_array'-output_array';
gp_u1=[input_array',state_array(:,1:4)];
gp_obj1=iddata(gp_y1,gp_u1,Ts);

%% 3pt filter
d1 = designfilt("lowpassiir",FilterOrder=3, ...
    HalfPowerFrequency=0.15,DesignMethod="butter");

close all
figure(1)
i =4
subplot(4,1,1)
plot(output_array(i,:))
hold on
plot(filtfilt(d1,output_array(i,:)))
ylabel('y_i')
title("i =" + i)
hold on
subplot(4,1,2)
plot(input_array(i,:))
ylabel('u_i')
hold on
subplot(4,1,3)
plot(state_array(:,2*i-1))
hold on
plot(filtfilt(d1,state_array(:,2*i-1)))
ylabel('xi')
hold on
subplot(4,1,4)
plot(state_array(:,2*i))
hold on
plot(filtfilt(d1,state_array(:,2*i)))
ylabel('xi+1')
hold on

figure(2)
subplot(3,1,1)
title("i =" + i)
plot(output_struct.Mi(i,:))
ylabel('Mi')
hold on
title("i =" + i)
subplot(3,1,2)
plot(output_struct.Ci(i,:))
ylabel('Ci')
hold on
subplot(3,1,3)
plot(output_struct.Gi(i,:))
ylabel('Gi')
hold on
Ts = par_set.Ts;
gp_y=-input_array'-output_array';
gp_u=[input_array',state_array];
gp_obj=iddata(gp_y,gp_u,Ts);

gp_y1=-input_array'-output_array';
gp_u1=[input_array',state_array(:,1:4)];
gp_obj1=iddata(gp_y1,gp_u1,Ts);
%% Linear curve fit yi = ai*ui -ki*qi -di*dot_qi
d1 = designfilt("lowpassiir",FilterOrder=10, ...
    HalfPowerFrequency=0.15,DesignMethod="butter");

i =1
temp_y=filtfilt(d1,output_array(i,:));
temp_u=[input_array(i,:)',filtfilt(d1,state_array(:,2*i-1)),filtfilt(d1,state_array(:,2*i))];
obj1f =iddata(temp_y',temp_u,Ts);
% xx = input_array(i,:)';q
% yy = filtfilt(d1,state_array(:,2*i-1));
% zz = filtfilt(d1,output_array(i,:));

i =2
temp_y=filtfilt(d1,output_array(i,:));
temp_u=[input_array(i,:)',filtfilt(d1,state_array(:,2*i-1)),filtfilt(d1,state_array(:,2*i))];
obj2f =iddata(temp_y',temp_u,Ts);
xx = input_array(i,220:250)';
yy = filtfilt(d1,state_array(220:250,2*i-1));
zz = filtfilt(d1,output_array(i,220:250));
obj2f_seg = iddata(zz',[xx,yy])
i =3
temp_y=filtfilt(d1,output_array(i,:));
temp_u=[input_array(i,:)',filtfilt(d1,state_array(:,2*i-1)),filtfilt(d1,state_array(:,2*i))];
obj3f =iddata(temp_y',temp_u,Ts);

i =4
temp_y=filtfilt(d1,output_array(i,:));
temp_u=[input_array(i,:)',filtfilt(d1,state_array(:,2*i-1)),filtfilt(d1,state_array(:,2*i))];
obj4f =iddata(temp_y',temp_u,Ts);

%% Linear curve fit yi = ai*ui -ki*qi -di*dot_qi
i =1
temp_y=output_array(i,:)';
temp_u=[input_array(i,:)',state_array(:,2*i-1),state_array(:,2*i)];
obj1 =iddata(temp_y,temp_u,Ts);

i =2
temp_y=output_array(i,:)';
temp_u=[input_array(i,:)',state_array(:,2*i-1),state_array(:,2*i)];
obj2 =iddata(temp_y,temp_u,Ts);

i =3
temp_y=output_array(i,:)';
temp_u=[input_array(i,:)',state_array(:,2*i-1),state_array(:,2*i)];
obj3 =iddata(temp_y,temp_u,Ts);

i =4
temp_y=output_array(i,:)';
temp_u=[input_array(i,:)',state_array(:,2*i-1),state_array(:,2*i)];
obj4 =iddata(temp_y,temp_u,Ts);
%%
gp_y=output_array';
gp_u=[input_array',state_array];
gp_obj=iddata(gp_y,gp_u,Ts);
%% Calculate bending angle
% Encoder reading based theta = (si-r - si-l)/ r0
testData = par_set.trial1;
s1.l_t0 = 205.7796;s1.r_t0 = 209.0839; 
s2.l_t0 = 223.6281;s2.r_t0 = 222.3360;
r0 = 62.2254*0.75
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

% Mocap based theta = 2 * (pi/2 - atan2(R2.pose.x - R1.pos.x,R1.pose.z -
% 0.05 - R2.pos.z))

testData = par_set.trial2;
par_set.R1_stand_off = 0.03;% m
% testData.theta_mocap = 2 * (pi/2 - atan2(testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1),...
%     testData.rigid_1_pose(:,3) - par_set.R1_stand_off - testData.rigid_2_pose(:,3))); 
% triangle cal
d_x = testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1);
d_y = testData.rigid_2_pose(:,2) - testData.rigid_1_pose(:,2);
d_z = testData.rigid_2_pose(:,3) - testData.rigid_1_pose(:,3) + par_set.R1_stand_off;
testData.s1.theta_mocap = 2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2));

d_x = testData.rigid_3_pose(:,1) - testData.rigid_1_pose(:,1);
d_y = testData.rigid_3_pose(:,2) - testData.rigid_1_pose(:,2);
d_z = testData.rigid_3_pose(:,3) - testData.rigid_1_pose(:,3)  + 0.004;
testData.s2.theta_mocap = 2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2)).*sign(d_x);
% testData.theta_mocap =2 * (atan2(testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1),...
%     testData.rigid_1_pose(:,3) - par_set.R1_stand_off - testData.rigid_2_pose(:,3)));  
close all;

for i = 1:length(testData.rigid_3_rot)
R3_mat{i} = quat2rotm(testData.rigid_3_rot(i,:));
R3_i = R3_mat{i};
R3_euler_angle(i,1) = atan2(-R3_i(3,1),sqrt(R3_i(3,2)^2 + R3_i(3,3)^2));  
end

figure(1)
subplot(2,1,1)
plot(rad2deg(testData.s1.theta_mocap))
hold on
plot(rad2deg(s1.theta_wire_rad))
hold on
plot(rad2deg(testData.s2.theta_mocap))
hold on
plot(rad2deg(s2.theta_wire_rad))
hold on
plot(rad2deg(R3_euler_angle))
hold on
plot(rad2deg(s2.theta_wire_rad)+rad2deg(s1.theta_wire_rad))
ylabel('rad')
legend('s1-m','s1-w','s2-m','s2-w','s12-m','s12-w')

subplot(2,1,2)
plot(testData.rigid_1_pose(:,3) - testData.rigid_2_pose(:,3) - 0.04 -0.02199/2)
hold on
plot(s1.l_wire_mm/1000)
hold on
plot(testData.rigid_2_pose(:,3) - testData.rigid_3_pose(:,3) -0.02199)
hold on
plot(s2.l_wire_mm/1000)
hold on
ylabel('m')

figure(2)
subplot(2,1,1)
plot(testData.enco_volts(:,1))
hold on
plot(testData.enco_volts(:,2))
hold on
plot(testData.enco_volts(:,3))
hold on
plot(testData.enco_volts(:,4))
hold on
legend('')
%%
for i = 1:length(testData.rigid_3_rot)
R3_mat{i} = quat2rotm(testData.rigid_3_rot(i,:));
R3_i = R3_mat{i};
R3_euler_angle(i,1) = atan2(-R3_i(3,1),sqrt(R3_i(3,2)^2 + R3_i(3,3)^2));  
end