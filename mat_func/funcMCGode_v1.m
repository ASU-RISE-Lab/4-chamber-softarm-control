function dxdt = funcMCGode_v1(t,x)
m0 = (100 + 34*2 + 25*5)/1000; %kg
% m1 = m2;
g = 9.8;
a1 = 0.02;
theta1 = x(1);
dtheta1 = x(2);
lc1 = x(3);
dlc1 = x(4);

M2x2 = [((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2) + (lc1^2*m0*sin(theta1/2)^2)/(4*theta1^2) + (lc1^2*m0*sin(theta1/2)^4)/(4*theta1^2) + (lc1^2*m0*cos(theta1/2)^2*sin(theta1/2)^2)/(4*theta1^2), (sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1;
(sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1, (sin(theta1/2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1^2];

C2x2 = [((dtheta1*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) + (dlc1*sin(theta1/2))/theta1)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/4, -(dtheta1*sin(theta1/2)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/(4*theta1);
 (dtheta1*sin(theta1/2)*((lc1*m0*sin(theta1/2))/theta1 + (lc1*m0*sin(theta1/2)^3)/theta1 + (lc1*m0*cos(theta1/2)^2*sin(theta1/2))/theta1))/(4*theta1), 0;];

G2x1= [g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (g*lc1*m0*sin(theta1/2)^2)/(2*theta1)
                                                                                 (g*m0*cos(theta1/2)*sin(theta1/2))/theta1];
    temp_dxdt = -M2x2\(C2x2*[x(2);x(4);] + G2x1);
    dxdt = [x(2);temp_dxdt(1);
        x(4);temp_dxdt(2);];
end


