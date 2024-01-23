function [output] = funcCompuRigidMCG_v1(stateVarArr)
    m2 = (100 + 34*2 + 25*5)/1000; %kg
    m1 = m2;
    h0 = 0.01; %m
    g = 9.8
    Iyy2 = 0;
    Iyy5 = 0;
a1 = 0.02;
for i  =  1:length(stateVarArr)
theta1 = stateVarArr(i,1);
dtheta1 = stateVarArr(i,2);
lc1 = stateVarArr(i,3);
dlc1 = stateVarArr(i,4);
theta2 = stateVarArr(i,5);
dtheta2 = stateVarArr(i,6);
lc2 = stateVarArr(i,7);
dlc2 = stateVarArr(i,8);
b_theta1 = lc1/(theta1)*tan(theta1/2);
b_theta2 = lc2/(theta2)*tan(theta2/2);
q1 = b_theta1; q2 = theta1; q3 = b_theta1;
q4 = b_theta2; q5 = theta2; q6 = b_theta2;
qd1 = (tan(theta1/2)*dlc1/theta1) - (tan(theta1/2)*lc1*dlc1)/theta1^2 + (lc1*(tan(theta1/2)^2 + 1)*dlc1)/(2*theta1);
qd2 = dtheta1;
qd3 =qd1;
qd4 = (tan(theta2/2)*dlc2/theta2) - (tan(theta2/2)*lc2*dlc2)/theta2^2 + (lc2*(tan(theta2/2)^2 + 1)*dlc2)/(2*theta2);
qd5 = dtheta2;
qd6 = qd4;
M6x6{i} = [m1 + m2, -m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)),m2*cos(q2),m2*cos(q2), 0, 0;
-m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4))^2 + m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))^2 + Iyy2*cos(q2)^2 + Iyy5*cos(q2)^2 + Iyy2*sin(q2)^2 + Iyy5*sin(q2)^2, m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), Iyy5*cos(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5)) + Iyy5*sin(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)), 0;
 m2*cos(q2), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*cos(q2)^2 + m2*sin(q2)^2, m2*cos(q2)^2 + m2*sin(q2)^2, 0, 0;
 m2*cos(q2), m2*sin(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) - m2*cos(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4)), m2*cos(q2)^2 + m2*sin(q2)^2, m2*cos(q2)^2 + m2*sin(q2)^2, 0, 0;
0, Iyy5*cos(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5)) + Iyy5*sin(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)), 0, 0, Iyy5*(cos(q2)*sin(q5) + cos(q5)*sin(q2))^2 + Iyy5*(cos(q2)*cos(q5) - sin(q2)*sin(q5))^2, 0;
0, 0, 0, 0, 0, 0;];

C6x6{i} = [ 0, 0,0,0, 0, 0;
- m2*qd3*sin(q2) - m2*qd4*sin(q2) - m2*qd2*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)), qd3*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))) + qd4*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), -qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), -qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))), 0, 0;
 -m2*qd2*sin(q2), qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))),0,0, 0, 0;
 -m2*qd2*sin(q2), qd2*(m2*cos(q2)*(cos(q2)*(a1 + q3) + cos(q2)*(a1 + q4)) + m2*sin(q2)*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4))),0,0, 0, 0;
 0,-qd5*(Iyy5*cos(q2)*(cos(q2)*sin(q5) + cos(q5)*sin(q2)) - Iyy5*sin(q2)*(cos(q2)*cos(q5) - sin(q2)*sin(q5))),0,0, 0, 0;
 0, 0,0,0, 0, 0;
];

G6x1{i}= [                                  g*m1 + g*m2;
-g*m2*(sin(q2)*(a1 + q3) + sin(q2)*(a1 + q4));
                                 g*m2*cos(q2);
                                 g*m2*cos(q2);
                                            0;
                                            0;];
output.detM(i) = det(M6x6{i});
output.detC(i) = det(C6x6{i});
output.rankM(i) = rank(M6x6{i});
% output.detG(i) = det(G4x1{i});
end

end

