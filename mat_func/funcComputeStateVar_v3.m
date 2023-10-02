function [output] = funcComputeStateVar_v3(testData,par_set)
%Fix log:
%Aug 11th: change theta_mocap_rad calculation make it match with the
%pressure input setup.
%Step 28: change pm force calculation to match the direction of angle and
%arclength changes,such that both K,D para. are positive.
% Use mocap for angle but encoder for arclength
% State equations.
%Oct 2: change l(t) to delta_l = l(p>0)-l(p=0)
    m0 = (100 + 34*2 + 25*5)/1000; %kg
    a1 = 0.05; %m
    g = 9.8; %N/kg
    timestamp=testData.time_stamp;%sec
    Ts=timestamp(2)-timestamp(1);

    %%%%% calculate tau, fz, theta and lc %%%%%%
    s1.l_t0 = 205.7796;s1.r_t0 = 209.0839; 
    s2.l_t0 = 223.6281;s2.r_t0 = 223.3360;%222.3360;
    r0 = 43;
    green_t0 = 31.92;yellow_t0 = 46.33; purple_t0 =21.99;
    s1.red_l = s1.l_t0 - green_t0;
    s1.red_r = s1.r_t0 - green_t0;
     s1.red_l = 169.7486;
    s1.red_r = 173.1282;
    s2.red_l = s2.l_t0 - yellow_t0 - purple_t0 - green_t0;
    s2.red_r = s2.r_t0 - yellow_t0 - purple_t0 - green_t0;
%     offL2 = mean(testData.enco_volts(:,4)/5*1000 ...
%         -1000*(abs(testData.rigid_2_pose(:,3) - testData.rigid_1_pose(:,3) + par_set.R1_stand_off))...
%         -1000*(abs(testData.rigid_3_pose(:,3) - testData.rigid_2_pose(:,3) + 0.004)));
%        offR2 = mean(testData.enco_volts(:,3)/5*1000 ...
%         -1000*(abs(testData.rigid_2_pose(:,3) - testData.rigid_1_pose(:,3) + par_set.R1_stand_off))...
%         -1000*(abs(testData.rigid_3_pose(:,3) - testData.rigid_2_pose(:,3) + 0.004)))

s1.red_r = s1.r_t0 - green_t0;

    s2.offset_l =(purple_t0 + s2.red_l)*1.0;
    s2.offset_r =(purple_t0 + s2.red_r)*1.0;
    s2.offset_l =126.3435;
    s2.offset_r =124.7783;

    s1.l_t = (testData.enco_volts(:,1)-par_set.enco_volt_p0(1))/5*1000;
    s1.r_t = (testData.enco_volts(:,2)-par_set.enco_volt_p0(2))/5*1000;
    
    s2.l_t = (testData.enco_volts(:,4)-par_set.enco_volt_p0(4))/5*1000;
    s2.r_t = (testData.enco_volts(:,3)-par_set.enco_volt_p0(3))/5*1000;
    
    s1.theta_wire_rad = (s1.r_t - s1.l_t)/r0;
    s1.l_wire_mm = (s1.r_t + s1.l_t)/2;
%     d_x = testData.rigid_2_pose(:,1) - testData.rigid_1_pose(:,1);
%     d_y = testData.rigid_2_pose(:,2) - testData.rigid_1_pose(:,2);
%     d_z = testData.rigid_2_pose(:,3) - testData.rigid_1_pose(:,3) + par_set.R1_stand_off;
%     s1.theta_mocap_rad = -(2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2)));
    quat_array(:,1) = testData.rigid_2_rot(:,4);
    quat_array(:,2:4) = testData.rigid_2_rot(:,1:3);
    rpy = quat2eul(quat_array,'XYZ');
    s1.theta_mocap_rad = rpy(:,2);

%     d_x = testData.rigid_3_pose(:,1) - testData.rigid_1_pose(:,1);
%     d_y = testData.rigid_3_pose(:,2) - testData.rigid_1_pose(:,2);
%     d_z = testData.rigid_3_pose(:,3) - testData.rigid_1_pose(:,3)  + 0.004;
%     s2.theta_mocap_rad = -(2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2))) - s1.theta_mocap_rad;
%     s2.theta_mocap_rad = 2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2));

    quat_array(:,1) = testData.rigid_3_rot(:,4);
    quat_array(:,2:4) = testData.rigid_3_rot(:,1:3);
    rpy = quat2eul(quat_array,'XYZ');
    s2.theta_mocap_rad = rpy(:,2);
    s2.theta_wire_rad = (s2.r_t - s2.l_t)/(r0);
    s2.l_wire_mm = (s2.r_t + s2.l_t)/2;
% alpha_r2 = 0.7825;
% alpha_l2 = 0.716;
% alpha_l1 = 1.084;
% alpha_r1 = 1.09;

% s1.theta_wire_rad = (alpha_r1*s1.r_t - alpha_l1*s1.l_t)/r0;
% s1.l_wire_mm = (alpha_r1*s1.r_t + alpha_l1*s1.l_t)/2;
% 
% s2.theta_wire_rad = (alpha_r2*s2.r_t - alpha_l2*s2.l_t)/r0;
% s2.l_wire_mm = (alpha_r2*s2.r_t + alpha_l2*s2.l_t)/2;

%     theta1_array = s1.theta_wire_rad;
%     theta2_array = s2.theta_wire_rad;
    theta1_array = s1.theta_mocap_rad;
    theta2_array = (s2.theta_mocap_rad - s1.theta_mocap_rad); 
    lc1_array = s1.l_wire_mm/1000;
    lc2_array = s2.l_wire_mm/1000;
    dtheta1_array = zeros(length(theta1_array),1);
    dtheta2_array = zeros(length(theta1_array),1);
    ddtheta1_array = zeros(length(theta1_array),1);
    ddtheta2_array = zeros(length(theta1_array),1);
    dlc1_array = zeros(length(theta1_array),1);
    dlc2_array = zeros(length(theta1_array),1);
    ddlc1_array = zeros(length(theta1_array),1);
    ddlc2_array = zeros(length(theta1_array),1);

    fprintf( 'Filtering pos... \n' );
    windowSize = 1;
    filt_lc1_array = filter((1/windowSize)*ones(1,windowSize),1,lc1_array);
    filt_lc2_array = filter((1/windowSize)*ones(1,windowSize),1,lc2_array);
    filt_theta1_array = filter((1/windowSize)*ones(1,windowSize),1,theta1_array);
    filt_theta2_array = filter((1/windowSize)*ones(1,windowSize),1,theta2_array);

%     return
    fprintf( 'Filtering vel... \n' );
    for i = 1:length(s1.theta_wire_rad)
        if i ==1
            dtheta1_array(i,1) = 0; 
            dtheta2_array(i,1) = 0; 
            dlc1_array(i,1) = 0; 
            dlc2_array(i,1) = 0;

        else
            dtheta1_array(i,1) = (filt_theta1_array(i,1)-filt_theta1_array(i-1))/Ts; 
            dtheta2_array(i,1) = (filt_theta2_array(i,1)-filt_theta2_array(i-1))/Ts; 

            dlc1_array(i,1) = (filt_lc1_array(i)-filt_lc1_array(i-1))/Ts;
            dlc2_array(i,1) = (filt_lc2_array(i)-filt_lc2_array(i-1))/Ts;
        end
    end
    
%     windowSize = 3;
%     filt_dlc1_array = filter((1/windowSize)*ones(1,windowSize),1,dlc1_array);
%     filt_dlc2_array = filter((1/windowSize)*ones(1,windowSize),1,dlc2_array);
%     filt_dtheta1_array = filter((1/windowSize)*ones(1,windowSize),1,dtheta1_array);
%     filt_dtheta2_array = filter((1/windowSize)*ones(1,windowSize),1,dtheta2_array);
    fit_length =21;
    filt_dlc1_array = sgolayfilt(dlc1_array,1,fit_length);
    filt_dlc2_array = sgolayfilt(dlc2_array,1,fit_length);
    filt_dtheta1_array = sgolayfilt(dtheta1_array,1,fit_length);
    filt_dtheta2_array = sgolayfilt(dtheta2_array,1,fit_length);

%     return

    for i = 1:length(s1.theta_wire_rad)
        if i ==1
            ddtheta1_array(i,1) = 0; ddtheta2_array(i,1) = 0;

            ddlc1_array(i,1) = 0; ddlc2_array(i,1) =0;
        else
        ddtheta1_array(i,1) =  (filt_dtheta1_array(i,1)-filt_dtheta1_array(i-1))/Ts; 
        ddtheta2_array(i,1) =  (filt_dtheta2_array(i,1)-filt_dtheta2_array(i-1))/Ts;

        ddlc1_array(i,1) = (filt_dlc1_array(i)-filt_dlc1_array(i-1))/Ts; 
        ddlc2_array(i,1) =(filt_dlc2_array(i,1)-filt_dlc2_array(i-1))/Ts;
        end
    end
     fprintf( 'Filtering acc... \n' );

%       windowSize = 20;
%     filt_ddlc1_array = filter((1/windowSize)*ones(1,windowSize),1,ddlc1_array);
%     filt_ddlc2_array = filter((1/windowSize)*ones(1,windowSize),1,ddlc2_array);
%     filt_ddtheta1_array = filter((1/windowSize)*ones(1,windowSize),1,ddtheta1_array);
%     filt_ddtheta2_array = filter((1/windowSize)*ones(1,windowSize),1,ddtheta2_array);

    fit_length =21;
    filt_ddlc1_array = sgolayfilt(ddlc1_array,1,fit_length);
    filt_ddlc2_array = sgolayfilt(ddlc2_array,1,fit_length);
    filt_ddtheta1_array = sgolayfilt(ddtheta1_array,1,fit_length);
    filt_ddtheta2_array = sgolayfilt(ddtheta2_array,1,fit_length);
fprintf( 'Finishing calculation... \n' );
%     theta1 = filt_theta1_array(i);lc1 = filt_lc1_array(i);
%     dtheta1 = filt_dtheta1_array(i);dlc1 = filt_dlc1_array(i);
%     ddtheta1 = filt_ddtheta1_array(i);ddlc1 = filt_ddlc1_array(i);
%     theta2 = filt_theta2_array(i);lc2 = filt_lc2_array(i);
%     dtheta2 = filt_dtheta2_array(i);dlc2 = filt_dlc2_array(i);
%     ddtheta2 = filt_ddtheta2_array(i);ddlc2 = filt_ddlc2_array(i);

state_array = [filt_theta1_array,filt_dtheta1_array,filt_lc1_array,filt_dlc1_array,...
               filt_theta2_array,filt_dtheta2_array,filt_lc2_array,filt_dlc2_array];
output.state_array = state_array;
%%
    theta1_array = -s1.theta_wire_rad;
    theta2_array = -s2.theta_wire_rad - theta1_array; 
    lc1_array = s1.l_wire_mm/1000;
    lc2_array = s2.l_wire_mm/1000;
    dtheta1_array = zeros(length(theta1_array),1);
    dtheta2_array = zeros(length(theta1_array),1);
    ddtheta1_array = zeros(length(theta1_array),1);
    ddtheta2_array = zeros(length(theta1_array),1);
    dlc1_array = zeros(length(theta1_array),1);
    dlc2_array = zeros(length(theta1_array),1);
    ddlc1_array = zeros(length(theta1_array),1);
    ddlc2_array = zeros(length(theta1_array),1);

    fprintf( 'Filtering pos... \n' );
    windowSize = 1;
    filt_lc1_array = filter((1/windowSize)*ones(1,windowSize),1,lc1_array);
    filt_lc2_array = filter((1/windowSize)*ones(1,windowSize),1,lc2_array);
    filt_theta1_array = filter((1/windowSize)*ones(1,windowSize),1,theta1_array);
    filt_theta2_array = filter((1/windowSize)*ones(1,windowSize),1,theta2_array);

%     return
    fprintf( 'Filtering vel... \n' );
    for i = 1:length(s1.theta_wire_rad)
        if i ==1
            dtheta1_array(i,1) = 0; 
            dtheta2_array(i,1) = 0; 
            dlc1_array(i,1) = 0; 
            dlc2_array(i,1) = 0;

        else
            dtheta1_array(i,1) = (filt_theta1_array(i,1)-filt_theta1_array(i-1))/Ts; 
            dtheta2_array(i,1) = (filt_theta2_array(i,1)-filt_theta2_array(i-1))/Ts; 

            dlc1_array(i,1) = (filt_lc1_array(i)-filt_lc1_array(i-1))/Ts;
            dlc2_array(i,1) = (filt_lc2_array(i)-filt_lc2_array(i-1))/Ts;
        end
    end
    
%     windowSize = 3;
%     filt_dlc1_array = filter((1/windowSize)*ones(1,windowSize),1,dlc1_array);
%     filt_dlc2_array = filter((1/windowSize)*ones(1,windowSize),1,dlc2_array);
%     filt_dtheta1_array = filter((1/windowSize)*ones(1,windowSize),1,dtheta1_array);
%     filt_dtheta2_array = filter((1/windowSize)*ones(1,windowSize),1,dtheta2_array);
    fit_length =21;
    filt_dlc1_array = sgolayfilt(dlc1_array,1,fit_length);
    filt_dlc2_array = sgolayfilt(dlc2_array,1,fit_length);
    filt_dtheta1_array = sgolayfilt(dtheta1_array,1,fit_length);
    filt_dtheta2_array = sgolayfilt(dtheta2_array,1,fit_length);

%     return

    for i = 1:length(s1.theta_wire_rad)
        if i ==1
            ddtheta1_array(i,1) = 0; ddtheta2_array(i,1) = 0;

            ddlc1_array(i,1) = 0; ddlc2_array(i,1) =0;
        else
        ddtheta1_array(i,1) =  (filt_dtheta1_array(i,1)-filt_dtheta1_array(i-1))/Ts; 
        ddtheta2_array(i,1) =  (filt_dtheta2_array(i,1)-filt_dtheta2_array(i-1))/Ts;

        ddlc1_array(i,1) = (filt_dlc1_array(i)-filt_dlc1_array(i-1))/Ts; 
        ddlc2_array(i,1) =(filt_dlc2_array(i,1)-filt_dlc2_array(i-1))/Ts;
        end
    end
     fprintf( 'Filtering acc... \n' );

%       windowSize = 20;
%     filt_ddlc1_array = filter((1/windowSize)*ones(1,windowSize),1,ddlc1_array);
%     filt_ddlc2_array = filter((1/windowSize)*ones(1,windowSize),1,ddlc2_array);
%     filt_ddtheta1_array = filter((1/windowSize)*ones(1,windowSize),1,ddtheta1_array);
%     filt_ddtheta2_array = filter((1/windowSize)*ones(1,windowSize),1,ddtheta2_array);

    fit_length =21;
    filt_ddlc1_array = sgolayfilt(ddlc1_array,1,fit_length);
    filt_ddlc2_array = sgolayfilt(ddlc2_array,1,fit_length);
    filt_ddtheta1_array = sgolayfilt(ddtheta1_array,1,fit_length);
    filt_ddtheta2_array = sgolayfilt(ddtheta2_array,1,fit_length);
fprintf( 'Finishing calculation... \n' );
%     theta1 = filt_theta1_array(i);lc1 = filt_lc1_array(i);
%     dtheta1 = filt_dtheta1_array(i);dlc1 = filt_dlc1_array(i);
%     ddtheta1 = filt_ddtheta1_array(i);ddlc1 = filt_ddlc1_array(i);
%     theta2 = filt_theta2_array(i);lc2 = filt_lc2_array(i);
%     dtheta2 = filt_dtheta2_array(i);dlc2 = filt_dlc2_array(i);
%     ddtheta2 = filt_ddtheta2_array(i);ddlc2 = filt_ddlc2_array(i);

state_array = [filt_theta1_array,filt_dtheta1_array,filt_lc1_array,filt_dlc1_array,...
               filt_theta2_array,filt_dtheta2_array,filt_lc2_array,filt_dlc2_array];
output.state_array_wire = state_array;


output.wire_angle_rad = [s1.theta_wire_rad,s2.theta_wire_rad];
output.mean_u_pm_psi = mean(testData.pm_psi,2);
output.u_pm_psi(:,1) = -(testData.pm_psi(:,1) - testData.pm_psi(:,2));
output.u_pm_psi(:,2) = testData.pm_psi(:,1) + testData.pm_psi(:,2) + 2*testData.pm_psi(:,3);
output.u_pm_psi(:,3) = -(testData.pm_psi(:,4) - testData.pm_psi(:,5));
output.u_pm_psi(:,4) = testData.pm_psi(:,4) + testData.pm_psi(:,5) + 2*testData.pm_psi(:,6);
output.u_pm_pa = output.u_pm_psi * 6894.76;
output.u_pm_tf(:,1) = output.u_pm_pa(:,1) * par_set.fz_a0 * par_set.tau_l0;
output.u_pm_tf(:,2) = output.u_pm_pa(:,2) * par_set.fz_a0;
output.u_pm_tf(:,3) = output.u_pm_pa(:,3) * par_set.fz_a0 * par_set.tau_l0;
output.u_pm_tf(:,4) = output.u_pm_pa(:,4) * par_set.fz_a0;
output.acc_array = [filt_ddtheta1_array,filt_ddlc1_array,filt_ddtheta2_array,filt_ddlc2_array];


    figure(1)
    subplot(4,1,1)
    plot(filt_theta1_array)
    hold on
    plot(s1.theta_mocap_rad)
    legend('wire','mocap')
    ylabel('$\theta_1$',Interpreter='latex')
    subplot(4,1,2)
    plot(filt_lc1_array)
    ylabel('$lc_1$',Interpreter='latex')
    subplot(4,1,3)
    plot(filt_theta2_array)
    hold on
    plot(s2.theta_mocap_rad-s1.theta_mocap_rad)
    ylabel('$\theta_2$',Interpreter='latex')
    subplot(4,1,4)
    plot(filt_lc2_array)
    ylabel('$lc_2$',Interpreter='latex')

    figure(2)
    subplot(4,1,1)
    plot(filt_dtheta1_array)
    ylabel('$\dot{\theta}_1$',Interpreter='latex')
    subplot(4,1,2)
    plot(filt_dlc1_array)
    hold on
    plot(sgolayfilt(dlc1_array,1,17))
    ylabel('$\dot{lc}_1$',Interpreter='latex')
    subplot(4,1,3)
    plot(filt_dtheta2_array)
    ylabel('$\dot{\theta}_2$',Interpreter='latex')
    subplot(4,1,4)
    plot(filt_dlc2_array)
    ylabel('$\dot{lc}_2$',Interpreter='latex')

    figure(3)
    subplot(4,1,1)
    plot(output.u_pm_psi(:,1))
    ylabel('$\tau_1$',Interpreter='latex')
    subplot(4,1,2)
    plot(output.u_pm_psi(:,2))
    ylabel('$f_1$',Interpreter='latex')
    subplot(4,1,3)
    plot(output.u_pm_psi(:,3))
    ylabel('$\tau_2$',Interpreter='latex')
    subplot(4,1,4)
    plot(output.u_pm_psi(:,4))
    ylabel('$f_2$',Interpreter='latex')
end 