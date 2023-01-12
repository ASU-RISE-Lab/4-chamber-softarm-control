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
par_set.flag_plot_rawData =1;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;

%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 0;
% Check data readme.txt for detail input reference
par_set.Ts=1/30;

par_set.L=0.185;%actuator length
par_set.n=4;% # of joints for augmented rigid arm
par_set.m0=0.35;%kg segment weight
par_set.g=9.8;%% gravity constant
par_set.R1_stand_off = 0.05;% m
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set= funcLoadExp2Seg(par_set,1);
    par_set= funcLoadExp2Seg(par_set,2);
    par_set= funcLoadExp2Seg(par_set,3);

    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
%% PLot raw data
testData = par_set.trial2;
if par_set.flag_plot_rawData == 1
    funcPlotRawData(testData)
end
%%
par_set.EOM = 1
if par_set.EOM ==1
    par_set = funcEOMbaseFrame2seg(par_set);
end
return
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