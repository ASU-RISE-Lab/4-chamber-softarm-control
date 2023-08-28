function [output] = funcKnownTerm2seg_v2(testData,par_set)
output =[];
    % Use mocap for angle but encoder for arclength
% State equations.
    m0 = (100 + 34*2 + 25*5)/1000; %kg
    a1 = 0.05; %m
    g = 9.8; %N/kg
    angle_th = deg2rad(5);
    timestamp=testData.time_stamp;%sec
    Ts=timestamp(2)-timestamp(1);
    mocapResult = funcComputeStateVar_v1(testData,par_set);
    stateVarArr = mocapResult.state_array;
    for i = 1: length(stateVarArr)
    theta1 = stateVarArr(i,1);
    lc1 = stateVarArr(i,3);
    theta2 = stateVarArr(i,5);
    lc2 = stateVarArr(i,7);

    dtheta1 = stateVarArr(i,2);
    dlc1 = stateVarArr(i,4);
    dtheta2 = stateVarArr(i,6);
    dlc2 = stateVarArr(i,8);

    ddtheta1 = mocapResult.acc_array(i,1);
    ddlc1 = mocapResult.acc_array(i,2);
    ddtheta2 = mocapResult.acc_array(i,3);
    ddlc2 = mocapResult.acc_array(i,4);

    if abs(theta1)<= angle_th && abs(theta2) <= angle_th
        simpM4x4 = [(lc2^2*m0)/4 + (m0*(3*lc1^2 + 4*lc1*lc2 + 2*lc2^2))/8,0, (lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0;
0, (5*m0)/4, 0, m0/2;
(lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0,(lc2^2*m0)/8,0;
0, m0/2, 0, m0/4;
 ];

        simpC4x4 = [(m0*(3*dlc1*lc1 + 2*dlc1*lc2 + 2*dlc2*lc1 + 4*dlc2*lc2))/8, -(m0*(3*dtheta1*lc1 + 4*dtheta1*lc2 + 2*dtheta2*lc2))/8, (lc2*m0*(dlc1 + 2*dlc2))/8, -(m0*((dtheta1*lc1)/2 + 2*dtheta1*lc2 + dtheta2*lc2))/4;
(dtheta1*m0*(3*lc1 + 2*lc2))/8, 0, (dtheta1*lc2*m0)/8, 0;
 (m0*(2*dlc2*lc1 + 4*dlc2*lc2))/16, -(m0*(2*dtheta1*lc2 + dtheta2*lc2))/8,(dlc2*lc2*m0)/8, -(lc2*m0*(2*dtheta1 + dtheta2))/8;
(m0*(2*dtheta1 + dtheta2)*(lc1/2 + lc2))/4, 0, (lc2*m0*(2*dtheta1 + dtheta2))/8, 0;];

        simpG4x1 = [         0;
(3*g*m0)/2;
         0;
  (g*m0)/2;];
    elseif abs(theta1)<= angle_th && abs(theta2) > angle_th
        simpM4x4 = [(m0*(4*lc2^2 - 4*lc2^2*cos(theta2) + 3*lc1^2*theta2^2 + 4*lc1*lc2*theta2*sin(theta2)))/(8*theta2^2) + (lc2^2*m0*sin(theta2/2)^2)/theta2^2, (lc2*m0*(cos(theta2)/2 - 1/2))/theta2, (lc2^2*m0*sin(theta2/2)^2)/(2*theta2^2) + (lc2*m0*(lc2 - lc1 + lc1*cos(theta2) - lc2*cos(theta2) + lc1*theta2*sin(theta2)))/(4*theta2^2), (lc1*m0*sin(theta2/2)^2)/(2*theta2);
(lc2*m0*(cos(theta2)/2 - 1/2))/theta2,(5*m0)/4,-(lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2), (m0*cos(theta2/2)*sin(theta2/2))/theta2;
 (lc2^2*m0*sin(theta2/2)^2)/(2*theta2^2) + (lc2*m0*(lc2 - lc1 + lc1*cos(theta2) - lc2*cos(theta2) + lc1*theta2*sin(theta2)))/(4*theta2^2), -(lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2),(lc2^2*m0*sin(theta2/2)^2)/(4*theta2^2) - (lc2^2*m0*(2*cos(theta2) + 2*theta2*sin(theta2) - theta2^2 - 2))/(4*theta2^4), (m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/theta2;
 -(lc1*m0*(cos(theta2) - 1))/(4*theta2), (m0*cos(theta2/2)*sin(theta2/2))/theta2,(m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/theta2, (m0*sin(theta2/2)^2)/theta2^2;
 ];

        simpC4x4 = [ (m0*(3*dlc1*lc1*theta2^3 - 8*dtheta2*lc2^2 + 8*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 - dtheta1*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc1*lc2*theta2^2*sin(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 4*dtheta2*lc2^2*theta2*sin(theta2) + dtheta1*lc1*lc2*theta2^2*cos(theta2) + 2*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 2*dtheta2*lc1*lc2*theta2*sin(theta2)))/(8*theta2^3),-(m0*(4*dlc2*theta2 - 4*dtheta2*lc2 + 3*dtheta1*lc1*theta2^2 + 4*dtheta2*lc2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 4*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*(8*dtheta1*lc2 - 2*dlc1*theta2 + 4*dlc2*theta2 - 8*dtheta1*lc2*cos(theta2) + 2*dlc1*theta2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 2*dlc1*theta2^2*sin(theta2) + dtheta1*lc1*theta2*sin(theta2) - 4*dtheta1*lc2*theta2*sin(theta2) - dtheta1*lc1*theta2^2*cos(theta2)))/(8*theta2^3), -(m0*sin(theta2/2)*(8*dtheta1*lc2*sin(theta2/2) + 4*dtheta2*lc2*sin(theta2/2) - 2*dlc1*theta2*sin(theta2/2) + dtheta1*lc1*theta2*cos(theta2/2)))/(4*theta2^2);
 (dtheta1*m0*(3*lc1*theta2 + 2*lc2*sin(theta2)))/(8*theta2),0,-(dtheta1*lc2*m0*sin(theta2/2)*(sin(theta2/2) - theta2*cos(theta2/2)))/(2*theta2^2), (dtheta1*m0*sin(theta2/2)^2)/(2*theta2);
(m0*(16*dtheta1*lc2^2*cos(theta2) - 16*dtheta2*lc2^2 - 16*dtheta1*lc2^2 + 16*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 + dtheta2*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 8*dtheta1*lc2^2*theta2*sin(theta2) + 8*dtheta2*lc2^2*theta2*sin(theta2) + 4*dtheta1*lc1*lc2*theta2^2*cos(theta2) + 3*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 4*dtheta1*lc1*lc2*theta2*sin(theta2) - 4*dtheta2*lc1*lc2*theta2*sin(theta2)))/(16*theta2^3), -(m0*(2*dlc2*theta2 - 4*dtheta2*lc2 - 4*dtheta1*lc2 + 4*dtheta1*lc2*cos(theta2) + 4*dtheta2*lc2*cos(theta2) - 2*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 3*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*sin(theta2/2)*(2*dlc2*theta2*sin(theta2/2) - 2*dtheta2*lc2*sin(theta2/2) + dtheta2*lc2*theta2*cos(theta2/2)))/(4*theta2^3),-(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2);
 (m0*sin(theta2/2)*(2*dtheta1 + dtheta2)*(4*lc2*sin(theta2/2) + lc1*theta2*cos(theta2/2)))/(4*theta2^2), -(m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2),(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2), 0;];

        simpG4x1 = [                    (g*lc2*m0*(cos(theta2)/2 - 1/2))/theta2;
                                                 (3*g*m0)/2;
-(g*lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2);
                  (g*m0*cos(theta2/2)*sin(theta2/2))/theta2];

    elseif abs(theta1)> angle_th && abs(theta2) <= angle_th
        simpM4x4 = [(m0*(20*lc1^2 - 20*lc1^2*cos(theta1) + 9*lc1^2*theta1^2 + 4*lc2^2*theta1^4 + 8*lc1*lc2*theta1^2 - 20*lc1^2*theta1*sin(theta1) + lc1^2*theta1^2*cos(theta1) - 8*lc1*lc2*theta1^2*cos(theta1)))/(8*theta1^4) + (lc1^2*m0*sin(theta1/2)^2)/(4*theta1^2) + (lc1^2*m0*sin(theta1/2)^4)/(4*theta1^2) + (lc1^2*m0*cos(theta1/2)^2*sin(theta1/2)^2)/(4*theta1^2),(m0*(10*lc1*cos(theta1) - 2*lc2*theta1^2 - 10*lc1 + 5*lc1*theta1*sin(theta1) + 2*lc2*theta1^2*cos(theta1)))/(4*theta1^3), (lc2*m0*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2), (lc1*m0*(theta1 - sin(theta1)))/(2*theta1^2);
((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*((sin(theta1/2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1 + (sin(theta1/2)*(2*m0*cos(theta1/2)^2 + 2*m0*sin(theta1/2)^2))/theta1) + (lc2*m0*(cos(theta1) - 1))/(2*theta1) + (2*sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1, (2*sin(theta1/2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1^2 + (sin(theta1/2)*((sin(theta1/2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1 + (sin(theta1/2)*(2*m0*cos(theta1/2)^2 + 2*m0*sin(theta1/2)^2))/theta1))/theta1,(lc2*m0*(cos(theta1) - 1))/(4*theta1),(m0*cos(theta1/2)*sin(theta1/2))/theta1;
(lc2*m0*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2),-(lc2*m0*sin(theta1/2)^2)/(2*theta1), (lc2^2*m0)/8,0;
(lc1*m0*(theta1 - sin(theta1)))/(2*theta1^2), (m0*cos(theta1/2)*sin(theta1/2))/theta1,0, m0/4;
 ];
        
        simpC4x4 = [(m0*(4*dlc2*lc2*theta1^3 - 6*dtheta1*lc1^2 + 6*dtheta1*lc1^2*cos(theta1) + 6*dlc1*lc1*theta1 + 4*dlc2*lc1*theta1 - dtheta1*lc1*lc2*theta1^2 - 2*dtheta2*lc1*lc2*theta1^2 - 6*dlc1*lc1*theta1*cos(theta1) - 4*dlc2*lc1*theta1*cos(theta1) + 2*dlc1*lc2*theta1^2*sin(theta1) + 3*dtheta1*lc1^2*theta1*sin(theta1) + dtheta1*lc1*lc2*theta1^2*cos(theta1) + 2*dtheta2*lc1*lc2*theta1*sin(theta1)))/(8*theta1^3), -(m0*(3*dtheta1*lc1 + 2*dlc2*theta1 - 3*dtheta1*lc1*cos(theta1) - 2*dlc2*theta1*cos(theta1) + 2*dtheta1*lc2*theta1*sin(theta1) + dtheta2*lc2*theta1*sin(theta1)))/(4*theta1^2), (lc2*m0*(4*dlc2*theta1^2 - 4*dtheta1*lc1*sin(theta1) + 2*dlc1*theta1*sin(theta1) + 3*dtheta1*lc1*theta1 + dtheta1*lc1*theta1*cos(theta1)))/(16*theta1^2), -(m0*(4*dtheta1*lc1 - 2*dlc1*theta1 + 4*dtheta1*lc2*theta1^2 + 2*dtheta2*lc2*theta1^2 - 4*dtheta1*lc1*cos(theta1) + 2*dlc1*theta1*cos(theta1) - dtheta1*lc1*theta1*sin(theta1)))/(8*theta1^2);
(dtheta1*m0*(3*lc1 - 3*lc1*cos(theta1) + lc2*theta1*sin(theta1)))/(4*theta1^2),0,(dtheta1*lc2*m0*sin(theta1))/(8*theta1), (dtheta1*m0*sin(theta1/2)^2)/(2*theta1);
 (m0*(2*dlc2*lc1 + 2*dlc2*lc2*theta1^2 - 2*dlc2*lc1*cos(theta1) - 2*dtheta1*lc1*lc2*theta1 - dtheta2*lc1*lc2*theta1 + 2*dtheta1*lc1*lc2*sin(theta1) + dtheta2*lc1*lc2*sin(theta1)))/(8*theta1^2),-(m0*sin(theta1/2)*(2*dlc2*sin(theta1/2) + 2*dtheta1*lc2*cos(theta1/2) + dtheta2*lc2*cos(theta1/2)))/(4*theta1),(dlc2*lc2*m0)/8, -(lc2*m0*(2*dtheta1 + dtheta2))/8;
(m0*(2*dtheta1 + dtheta2)*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2), -(m0*sin(theta1/2)^2*(2*dtheta1 + dtheta2))/(2*theta1), (lc2*m0*(2*dtheta1 + dtheta2))/8, 0;];
        
        simpG4x1 = [3*g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (g*m0*(lc1 - lc1*cos(theta1) + lc2*theta1*sin(theta1)))/(2*theta1) - (g*lc1*m0*sin(theta1/2)^2)/(2*theta1);
(3*g*m0*cos(theta1/2)*sin(theta1/2))/theta1;
-(g*lc2*m0*sin(theta1))/4;
 (g*m0*cos(theta1))/2;];
    else
    simpM4x4 = [(m0*(20*lc1^2*theta2^2 + 8*lc2^2*theta1^4 + 11*lc1^2*theta1^2*theta2^2 - 20*lc1^2*theta2^2*cos(theta1) - 8*lc2^2*theta1^4*cos(theta2) - 20*lc1^2*theta1*theta2^2*sin(theta1) - 8*lc1*lc2*theta1^3*theta2 - lc1^2*theta1^2*theta2^2*cos(theta1) - 8*lc1*lc2*theta1^2*theta2*sin(theta1 + theta2) + 8*lc1*lc2*theta1^3*theta2*cos(theta2) + 8*lc1*lc2*theta1^2*theta2*sin(theta1) + 8*lc1*lc2*theta1^2*theta2*sin(theta2)))/(8*theta1^4*theta2^2), -(m0*(10*lc1*theta2 - 10*lc1*theta2*cos(theta1) - 2*lc2*theta1^2*sin(theta1 + theta2) + 2*lc2*theta1^2*sin(theta1) + 2*lc2*theta1^2*sin(theta2) - 5*lc1*theta1*theta2*sin(theta1)))/(4*theta1^3*theta2), -(lc2*m0*(lc1 + lc1*cos(theta1 + theta2) - lc2*theta1^2 - lc1*cos(theta1) - lc1*cos(theta2) + lc1*theta1*sin(theta2) - lc1*theta2*sin(theta2) + lc2*theta1^2*cos(theta2) + lc1*theta2*sin(theta1 + theta2) - lc1*theta1*theta2*cos(theta2)))/(2*theta1^2*theta2^2), (lc1*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta1*sin(theta2) + 1))/(2*theta1^2*theta2);
-(m0*(10*lc1*theta2 - 10*lc1*theta2*cos(theta1) - 2*lc2*theta1^2*sin(theta1 + theta2) + 2*lc2*theta1^2*sin(theta1) + 2*lc2*theta1^2*sin(theta2) - 5*lc1*theta1*theta2*sin(theta1)))/(4*theta1^3*theta2),-(5*m0*(cos(theta1) - 1))/(2*theta1^2), (lc2*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta2*sin(theta1 + theta2) - theta2*sin(theta2) + 1))/(2*theta1*theta2^2), -(m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + 1))/(2*theta1*theta2);
 -(lc2*m0*(lc1 + lc1*cos(theta1 + theta2) - lc2*theta1^2 - lc1*cos(theta1) - lc1*cos(theta2) + lc1*theta1*sin(theta2) - lc1*theta2*sin(theta2) + lc2*theta1^2*cos(theta2) + lc1*theta2*sin(theta1 + theta2) - lc1*theta1*theta2*cos(theta2)))/(2*theta1^2*theta2^2),(lc2*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta2*sin(theta1 + theta2) - theta2*sin(theta2) + 1))/(2*theta1*theta2^2),-(lc2^2*m0*(4*cos(theta2) + theta2^2*cos(theta2) + 4*theta2*sin(theta2) - 3*theta2^2 - 4))/(8*theta2^4), (lc2*m0*(2*cos(theta2) + theta2*sin(theta2) - 2))/(4*theta2^3);
 (lc1*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta1*sin(theta2) + 1))/(2*theta1^2*theta2),-(m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + 1))/(2*theta1*theta2), (lc2*m0*(2*cos(theta2) + theta2*sin(theta2) - 2))/(4*theta2^3), -(m0*(cos(theta2) - 1))/(2*theta2^2);];
    
    simpC4x4 = [-(m0*(6*dtheta1*lc1^2*theta2^3 + 8*dtheta2*lc2^2*theta1^3 - 6*dtheta1*lc1^2*theta2^3*cos(theta1) - 8*dtheta2*lc2^2*theta1^3*cos(theta2) - 6*dlc1*lc1*theta1*theta2^3 - 8*dlc2*lc2*theta1^3*theta2 + 2*dlc1*lc2*theta1^2*theta2^2 + 4*dlc2*lc1*theta1^2*theta2^2 + 4*dlc2*lc1*theta1*theta2^2*sin(theta1 + theta2) + 6*dlc1*lc1*theta1*theta2^3*cos(theta1) + 8*dlc2*lc2*theta1^3*theta2*cos(theta2) - 4*dlc2*lc1*theta1*theta2^2*sin(theta1) - 4*dlc2*lc1*theta1*theta2^2*sin(theta2) + 2*dlc1*lc2*theta1^2*theta2^2*cos(theta1 + theta2) - 2*dlc1*lc2*theta1^2*theta2^2*cos(theta1) - 2*dlc1*lc2*theta1^2*theta2^2*cos(theta2) - 4*dlc2*lc1*theta1^2*theta2^2*cos(theta2) - 3*dtheta1*lc1^2*theta1*theta2^3*sin(theta1) - 4*dtheta2*lc2^2*theta1^3*theta2*sin(theta2) - 4*dtheta2*lc1*lc2*theta1^2*theta2 - dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta1 + theta2) + dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta1) + dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta2) + 4*dtheta2*lc1*lc2*theta1^2*theta2^2*sin(theta2) - 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta1 + theta2) + 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta1) + 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta2) + 4*dtheta2*lc1*lc2*theta1*theta2^2*cos(theta1 + theta2) - 4*dtheta2*lc1*lc2*theta1*theta2^2*cos(theta2) + 4*dtheta2*lc1*lc2*theta1^2*theta2*cos(theta2)))/(8*theta1^3*theta2^3), -(m0*(3*dtheta1*lc1*theta2^2 - 2*dtheta1*lc2*theta1*theta2 + 2*dtheta2*lc2*theta1*sin(theta1 + theta2) - 2*dlc2*theta1*theta2*sin(theta1 + theta2) - 2*dtheta2*lc2*theta1*sin(theta1) - 2*dtheta2*lc2*theta1*sin(theta2) + 2*dlc2*theta1*theta2*sin(theta1) + 2*dlc2*theta1*theta2*sin(theta2) - 3*dtheta1*lc1*theta2^2*cos(theta1) - 2*dtheta1*lc2*theta1*theta2*cos(theta1 + theta2) - 2*dtheta2*lc2*theta1*theta2*cos(theta1 + theta2) + 2*dtheta1*lc2*theta1*theta2*cos(theta1) + 2*dtheta1*lc2*theta1*theta2*cos(theta2) + 2*dtheta2*lc2*theta1*theta2*cos(theta2)))/(4*theta1^2*theta2^2), (lc2*m0*(8*dtheta1*lc2*theta1^2 + 4*dlc2*theta1^2*theta2 - 3*dtheta1*lc1*theta1*theta2 - 4*dtheta1*lc1*theta2*sin(theta1 + theta2) + 2*dlc1*theta1*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*theta2*sin(theta1) + 4*dtheta1*lc1*theta2*sin(theta2) - 2*dlc1*theta1*theta2*sin(theta1) - 2*dlc1*theta1*theta2*sin(theta2) + 4*dtheta1*lc1*theta2^2*cos(theta1 + theta2) - 2*dlc1*theta1*theta2^2*cos(theta1 + theta2) - 4*dtheta1*lc1*theta2^2*cos(theta2) - 8*dtheta1*lc2*theta1^2*cos(theta2) + 2*dlc1*theta1*theta2^2*cos(theta2) - 4*dlc2*theta1^2*theta2*cos(theta2) + dtheta1*lc1*theta1*theta2^2*sin(theta1 + theta2) + 3*dtheta1*lc1*theta1*theta2^2*sin(theta2) - 4*dtheta1*lc2*theta1^2*theta2*sin(theta2) + dtheta1*lc1*theta1*theta2*cos(theta1 + theta2) - dtheta1*lc1*theta1*theta2*cos(theta1) + 3*dtheta1*lc1*theta1*theta2*cos(theta2)))/(8*theta1^2*theta2^3), -(m0*(8*dtheta1*lc2*theta1^2 + 4*dtheta2*lc2*theta1^2 - 3*dtheta1*lc1*theta1*theta2 - 4*dtheta1*lc1*theta2*sin(theta1 + theta2) + 2*dlc1*theta1*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*theta2*sin(theta1) + 4*dtheta1*lc1*theta2*sin(theta2) - 2*dlc1*theta1*theta2*sin(theta1) - 2*dlc1*theta1*theta2*sin(theta2) - 8*dtheta1*lc2*theta1^2*cos(theta2) - 4*dtheta2*lc2*theta1^2*cos(theta2) + dtheta1*lc1*theta1*theta2*cos(theta1 + theta2) - dtheta1*lc1*theta1*theta2*cos(theta1) + 3*dtheta1*lc1*theta1*theta2*cos(theta2)))/(8*theta1^2*theta2^2);
(dtheta1*m0*(3*lc1*theta2 - lc2*theta1 - 3*lc1*theta2*cos(theta1) + lc2*theta1*cos(theta1) + lc2*theta1*cos(theta2) - lc2*theta1*cos(theta1 + theta2)))/(4*theta1^2*theta2), 0,-(dtheta1*lc2*m0*(sin(theta1) - sin(theta1 + theta2) + sin(theta2) + theta2*cos(theta1 + theta2) - theta2*cos(theta2)))/(4*theta1*theta2^2),(dtheta1*m0*(sin(theta1) - sin(theta1 + theta2) + sin(theta2)))/(4*theta1*theta2);
 -(m0*(8*dtheta1*lc2^2*theta1^2 + 8*dtheta2*lc2^2*theta1^2 - 8*dtheta1*lc2^2*theta1^2*cos(theta2) - 8*dtheta2*lc2^2*theta1^2*cos(theta2) - dtheta2*lc1*lc2*theta2^2 + 2*dlc2*lc1*theta1*theta2^2 - 4*dlc2*lc2*theta1^2*theta2 + 2*dlc2*lc1*theta2^2*sin(theta1 + theta2) - 2*dlc2*lc1*theta2^2*sin(theta1) - 2*dlc2*lc1*theta2^2*sin(theta2) + 4*dtheta1*lc1*lc2*theta2^2*cos(theta1 + theta2) + 3*dtheta2*lc1*lc2*theta2^2*cos(theta1 + theta2) - 4*dtheta1*lc1*lc2*theta2^2*cos(theta2) + dtheta2*lc1*lc2*theta2^2*cos(theta1) - 3*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 2*dlc2*lc1*theta1*theta2^2*cos(theta2) + 4*dlc2*lc2*theta1^2*theta2*cos(theta2) - 4*dtheta1*lc1*lc2*theta1*theta2 - 4*dtheta2*lc1*lc2*theta1*theta2 - 4*dtheta1*lc2^2*theta1^2*theta2*sin(theta2) - 4*dtheta2*lc2^2*theta1^2*theta2*sin(theta2) - 4*dtheta1*lc1*lc2*theta2*sin(theta1 + theta2) - 4*dtheta2*lc1*lc2*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*lc2*theta2*sin(theta1) + 4*dtheta1*lc1*lc2*theta2*sin(theta2) + 4*dtheta2*lc1*lc2*theta2*sin(theta1) + 4*dtheta2*lc1*lc2*theta2*sin(theta2) + 4*dtheta1*lc1*lc2*theta1*theta2*cos(theta2) + 4*dtheta2*lc1*lc2*theta1*theta2*cos(theta2) + 4*dtheta1*lc1*lc2*theta1*theta2^2*sin(theta2) + 3*dtheta2*lc1*lc2*theta1*theta2^2*sin(theta2)))/(8*theta1^2*theta2^3), (m0*(4*dtheta1*lc2*sin(theta1) + 4*dtheta1*lc2*sin(theta2) + 4*dtheta2*lc2*sin(theta1) + 4*dtheta2*lc2*sin(theta2) - 2*dlc2*theta2*sin(theta1) - 2*dlc2*theta2*sin(theta2) - dtheta2*lc2*theta2 - 4*dtheta1*lc2*sin(theta1 + theta2) - 4*dtheta2*lc2*sin(theta1 + theta2) + 2*dlc2*theta2*sin(theta1 + theta2) + 4*dtheta1*lc2*theta2*cos(theta1 + theta2) + 3*dtheta2*lc2*theta2*cos(theta1 + theta2) - 4*dtheta1*lc2*theta2*cos(theta2) + dtheta2*lc2*theta2*cos(theta1) - 3*dtheta2*lc2*theta2*cos(theta2)))/(8*theta1*theta2^2), (lc2*m0*(2*dlc2*theta2 - 2*dtheta2*lc2 + 2*dtheta2*lc2*cos(theta2) - 2*dlc2*theta2*cos(theta2) + dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^3),(lc2*m0*(cos(theta2) - 1)*(2*dtheta1 + dtheta2))/(4*theta2^2);
 (m0*(2*dtheta1 + dtheta2)*(2*lc2*theta1^2 + lc1*theta2*sin(theta1) + lc1*theta2*sin(theta2) - 2*lc2*theta1^2*cos(theta2) - lc1*theta1*theta2 - lc1*theta2*sin(theta1 + theta2) + lc1*theta1*theta2*cos(theta2)))/(4*theta1^2*theta2^2),-(m0*(2*dtheta1 + dtheta2)*(sin(theta1) - sin(theta1 + theta2) + sin(theta2)))/(4*theta1*theta2), -(lc2*m0*(cos(theta2) - 1)*(2*dtheta1 + dtheta2))/(4*theta2^2),0;
];
    simpG4x1 = [-(g*m0*(3*lc1*theta2*sin(theta1) - lc2*theta1^2*cos(theta1 + theta2) + lc2*theta1^2*cos(theta1) - 3*lc1*theta1*theta2*cos(theta1)))/(2*theta1^2*theta2);
(3*g*m0*sin(theta1))/(2*theta1);
 (g*lc2*m0*(sin(theta1) - sin(theta1 + theta2) + theta2*cos(theta1 + theta2)))/(2*theta2^2);
 (g*m0*(sin(theta1 + theta2)/2 - sin(theta1)/2))/theta2;];
    end
    mcg_array(1:4,i) = simpM4x4 * [ddtheta1;ddlc1;ddtheta2;ddlc2] + simpC4x4 * [dtheta1;dlc1;dtheta2;dlc2] + simpG4x1;
    Mi(1:4,i)  = simpM4x4 * [ddtheta1;ddlc1;ddtheta2;ddlc2];
    Ci(1:4,i) = simpC4x4 * [dtheta1;dlc1;dtheta2;dlc2];
    Gi(1:4,i) = simpG4x1;
    invM = inv(simpM4x4);
    end

    
output.mcg_array = mcg_array;
output.state_array = stateVarArr;
output.Mddq = Mi;
output.Gq = Gi;
output.Cqdq = Ci;
output.mean_u_pm_psi = mocapResult.mean_u_pm_psi;
output.u_pm_tf = mocapResult.u_pm_tf;
output.u_pm_psi = mocapResult.u_pm_psi;
output.acc_array = mocapResult.acc_array;
close all
figure(1)
titlelist = {'theta1','lc1','theta2','lc2'};
legnedlist = {'M','C','G'};
for i  =1 :4
subplot(4,1,i)
plot(output.Mddq(i,:))
hold on 
plot(output.Cqdq(i,:))
hold on 
plot(output.Gq(i,:))
hold on 
% plot(output.u_pm_tf(:,i))
title(titlelist{i})
end 
legend('M','C','G');
unitlistleft = {'Nm','N','Nm','N'};
unitlistright = {'rad','m','rad','m'};
ylimleft = [-100 100; 0 1000; -100 100; 0 1000];
ylimright = [deg2rad(-45) deg2rad(45); 0.13 0.16; deg2rad(-45) deg2rad(45); 0.09 0.12];
figure(2)
for i  =1 :4
subplot(4,1,i)
yyaxis left
plot(output.u_pm_tf(:,i)' - output.Mddq(i,:) - output.Cqdq(i,:) - output.Gq(i,:))
ylabel(unitlistleft{i})
ylim(ylimleft(i,:))
hold on
yyaxis right
ylabel(unitlistright{i})
ylim(ylimright(i,:))
plot(output.state_array(:,2*i-1))
title(titlelist{i})
end 
end
