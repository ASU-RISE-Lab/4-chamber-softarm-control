function [output] = funcCompuMCG_v1(stateVarArr)
    m0 = (100 + 34*2 + 25*5)/1000; %kg
%     m1 = m2;
    h0 = 0.01; %m
    g = 9.8;
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


M2x2{i} = [((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2) + (lc1^2*m0*sin(theta1/2)^2)/(4*theta1^2) + (lc1^2*m0*sin(theta1/2)^4)/(4*theta1^2) + (lc1^2*m0*cos(theta1/2)^2*sin(theta1/2)^2)/(4*theta1^2), (sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1;
(sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1, (sin(theta1/2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1^2];

C2x2{i} = [((dtheta1*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) + (dlc1*sin(theta1/2))/theta1)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/4, -(dtheta1*sin(theta1/2)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/(4*theta1);
 (dtheta1*sin(theta1/2)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/(4*theta1), 0;];

G2x1{i} = [g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (g*lc1*m0*sin(theta1/2)^2)/(2*theta1)
                                                                                 (g*m0*cos(theta1/2)*sin(theta1/2))/theta1];
output.detM(i) = det(M2x2{i});
output.detC(i) = det(C2x2{i});
output.rankG(i) = rank(G2x1{i});

end

