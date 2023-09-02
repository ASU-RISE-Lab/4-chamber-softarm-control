function [output] = funcComputeNNInputOutputPair_v1(testData,par_set)
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

    
    theta1 = s1.theta_wire_rad;
    theta2 = s2.theta_wire_rad;
    lc1 = s1.l_wire_mm/1000;
    lc2 = s2.l_wire_mm/1000;
    b_theta1 = lc1./(theta1).*sin(theta1/2);
    b_theta2 = lc2./(theta2).*sin(theta2/2);
    a1 = 0.05;
    endEffector2x = (2*b_theta1).*sin(theta1/2) + (2*b_theta2 + a1).*sin(theta1 + theta2/2);
    endEffector2y = -(2*b_theta1).*cos(theta1/2) - (2*b_theta2 + a1).*cos(theta1 + theta2/2);

    endEffectorPhiz = theta1+theta2;

    Tbase2mocap = [1 0 0 0.0068; 
                0 0 -1 -0.2111;
                0 1 0  0.5086; 
                0 0 0 1];
   xyzCam = testData.rigid_3_pose;
   d_x = testData.rigid_3_pose(:,1) - testData.rigid_1_pose(:,1);
    d_y = testData.rigid_3_pose(:,2) - testData.rigid_1_pose(:,2);
    d_z = testData.rigid_3_pose(:,3) - testData.rigid_1_pose(:,3)  + 0.004;
   theta_mocap_rad = -(2 *asin(d_x./sqrt(d_x.^2 + d_y.^2 + d_z.^2)));
   for i =1:length(xyzCam)
   xyzArm(:,i) = inv(Tbase2mocap)*[xyzCam(i,:),1]';
   end
   output.x_measured = xyzArm(1,:)';
   output.y_measured = xyzArm(2,:)';
   output.phiz_measured = theta_mocap_rad;
   output.x_est = endEffector2x;
   output.y_est = endEffector2y;
   output.phiz_est = -endEffectorPhiz;
   close all
   figure(1)
    subplot(3,1,1)
    plot(output.x_measured)
    hold on
    plot(output.x_est)
    ylabel('$x(m)$',Interpreter='latex')
    legend('mocap','model est.')
    subplot(3,1,2)
    plot(output.y_measured)
    hold on
    plot(output.y_est)
    ylabel('$y(m)$',Interpreter='latex')
    subplot(3,1,3)
    plot(output.phiz_measured)
    hold on
    plot(output.phiz_est)
    ylabel('$/phi_z(rad)$',Interpreter='latex')

end