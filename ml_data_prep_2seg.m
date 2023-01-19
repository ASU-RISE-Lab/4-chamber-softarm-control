%% run ml_ral_main 1st to collect all data
for i =1:11
    if i ==1
        testData = par_set.trial1;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];

    elseif i==2
        testData = par_set.trial2;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
     elseif i==3
        testData = par_set.trial3;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
     elseif i==4
        testData = par_set.trial4;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==5
        testData = par_set.trial5;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==6
        testData = par_set.trial6;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==7
        testData = par_set.trial7;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==8
        testData = par_set.trial8;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==9
        testData = par_set.trial9;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==10
        testData = par_set.trial10;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
         elseif i==11
        testData = par_set.trial11;
        timestamp = testData.time_stamp;Ts=timestamp(2)-timestamp(1);
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
        gp_y=[output1,output2, output3, output4];
        gp_u=[input1, input2, input3,input4];
    end
    exp_t{i}=iddata(gp_y,gp_u,Ts);
%     set(exp_t{i},'InputName',{'tauy1';'fz1';'tauy2';'fz2'},'OutputName',{'a1','l1','a2','l2'});
%     exp_t{i}.InputUnit = {'psi';'psi';'psi';'psi'};
%     exp_t{i}.OutputUnit = {'rad';'mm';'rad';'mm'};
end
traindata = merge(exp_t{1:end});
    set(traindata,'InputName',{'tauy1';'fz1';'tauy2';'fz2'},'OutputName',{'a1','l1','a2','l2'});
    traindata.InputUnit = {'psi';'psi';'psi';'psi'};
    traindata.OutputUnit = {'rad';'mm';'rad';'mm'};
present(traindata)