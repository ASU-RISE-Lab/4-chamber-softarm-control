function [dx, y] = func2segODE_m(t, x, u,alpha1,alpha2,alpha3,alpha4,...
                                k1,k2,k3,k4,d1,d2,d3,d4, varargin)
% Output equations.
%   y = [x(1);x(3);x(5);x(7)];% a1 l1 a2 l2
y = x;% full state feedback
% State equations.
m0 = (100 + 34*2 + 25*5)/1000; %kg
h0 = 0.01; %m
g = 9.8; %N/kg
tauy1 = u(1);
fz1 = u(2);
tauy2 = u(3);
fz2 = u(4);
theta1 = x(1); dtheta1 = x(2); lc1 = x(3); dlc1 = x(4);
theta2 = x(5); dtheta2 = x(6); lc2 = x(7); dlc2 = x(8);
condition =0;
angle_th = deg2rad(5);
if abs(theta1)<= angle_th && abs(theta2) <= angle_th
    M = [(lc2^2*m0)/4 + (m0*(3*lc1^2 + 4*lc1*lc2 + 2*lc2^2))/8,0, (lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0;
        0, (5*m0)/4, 0, m0/2;
        (lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0,(lc2^2*m0)/8,0;
        0, m0/2, 0, m0/4;
        ];

    C = [(m0*(3*dlc1*lc1 + 2*dlc1*lc2 + 2*dlc2*lc1 + 4*dlc2*lc2))/8, -(m0*(3*dtheta1*lc1 + 4*dtheta1*lc2 + 2*dtheta2*lc2))/8, (lc2*m0*(dlc1 + 2*dlc2))/8, -(m0*((dtheta1*lc1)/2 + 2*dtheta1*lc2 + dtheta2*lc2))/4;
        (dtheta1*m0*(3*lc1 + 2*lc2))/8, 0, (dtheta1*lc2*m0)/8, 0;
        (m0*(2*dlc2*lc1 + 4*dlc2*lc2))/16, -(m0*(2*dtheta1*lc2 + dtheta2*lc2))/8,(dlc2*lc2*m0)/8, -(lc2*m0*(2*dtheta1 + dtheta2))/8;
        (m0*(2*dtheta1 + dtheta2)*(lc1/2 + lc2))/4, 0, (lc2*m0*(2*dtheta1 + dtheta2))/8, 0;];

    G = [         0;
        (3*g*m0)/2;
        0;
        (g*m0)/2;];
    condition =0;
elseif abs(theta1)<= angle_th && abs(theta2) > angle_th
    M = [(m0*(4*lc2^2 - 4*lc2^2*cos(theta2) + 3*lc1^2*theta2^2 + 4*lc1*lc2*theta2*sin(theta2)))/(8*theta2^2) + (lc2^2*m0*sin(theta2/2)^2)/theta2^2, (lc2*m0*(cos(theta2)/2 - 1/2))/theta2, (lc2^2*m0*sin(theta2/2)^2)/(2*theta2^2) + (lc2*m0*(lc2 - lc1 + lc1*cos(theta2) - lc2*cos(theta2) + lc1*theta2*sin(theta2)))/(4*theta2^2), (lc1*m0*sin(theta2/2)^2)/(2*theta2);
        (lc2*m0*(cos(theta2)/2 - 1/2))/theta2,(5*m0)/4,-(lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2), (m0*cos(theta2/2)*sin(theta2/2))/theta2;
        (lc2^2*m0*sin(theta2/2)^2)/(2*theta2^2) + (lc2*m0*(lc2 - lc1 + lc1*cos(theta2) - lc2*cos(theta2) + lc1*theta2*sin(theta2)))/(4*theta2^2), -(lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2),(lc2^2*m0*sin(theta2/2)^2)/(4*theta2^2) - (lc2^2*m0*(2*cos(theta2) + 2*theta2*sin(theta2) - theta2^2 - 2))/(4*theta2^4), (m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/theta2;
        -(lc1*m0*(cos(theta2) - 1))/(4*theta2), (m0*cos(theta2/2)*sin(theta2/2))/theta2,(m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/theta2, (m0*sin(theta2/2)^2)/theta2^2;
        ];

    C = [ (m0*(3*dlc1*lc1*theta2^3 - 8*dtheta2*lc2^2 + 8*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 - dtheta1*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc1*lc2*theta2^2*sin(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 4*dtheta2*lc2^2*theta2*sin(theta2) + dtheta1*lc1*lc2*theta2^2*cos(theta2) + 2*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 2*dtheta2*lc1*lc2*theta2*sin(theta2)))/(8*theta2^3),-(m0*(4*dlc2*theta2 - 4*dtheta2*lc2 + 3*dtheta1*lc1*theta2^2 + 4*dtheta2*lc2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 4*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*(8*dtheta1*lc2 - 2*dlc1*theta2 + 4*dlc2*theta2 - 8*dtheta1*lc2*cos(theta2) + 2*dlc1*theta2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 2*dlc1*theta2^2*sin(theta2) + dtheta1*lc1*theta2*sin(theta2) - 4*dtheta1*lc2*theta2*sin(theta2) - dtheta1*lc1*theta2^2*cos(theta2)))/(8*theta2^3), -(m0*sin(theta2/2)*(8*dtheta1*lc2*sin(theta2/2) + 4*dtheta2*lc2*sin(theta2/2) - 2*dlc1*theta2*sin(theta2/2) + dtheta1*lc1*theta2*cos(theta2/2)))/(4*theta2^2);
        (dtheta1*m0*(3*lc1*theta2 + 2*lc2*sin(theta2)))/(8*theta2),0,-(dtheta1*lc2*m0*sin(theta2/2)*(sin(theta2/2) - theta2*cos(theta2/2)))/(2*theta2^2), (dtheta1*m0*sin(theta2/2)^2)/(2*theta2);
        (m0*(16*dtheta1*lc2^2*cos(theta2) - 16*dtheta2*lc2^2 - 16*dtheta1*lc2^2 + 16*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 + dtheta2*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 8*dtheta1*lc2^2*theta2*sin(theta2) + 8*dtheta2*lc2^2*theta2*sin(theta2) + 4*dtheta1*lc1*lc2*theta2^2*cos(theta2) + 3*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 4*dtheta1*lc1*lc2*theta2*sin(theta2) - 4*dtheta2*lc1*lc2*theta2*sin(theta2)))/(16*theta2^3), -(m0*(2*dlc2*theta2 - 4*dtheta2*lc2 - 4*dtheta1*lc2 + 4*dtheta1*lc2*cos(theta2) + 4*dtheta2*lc2*cos(theta2) - 2*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 3*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*sin(theta2/2)*(2*dlc2*theta2*sin(theta2/2) - 2*dtheta2*lc2*sin(theta2/2) + dtheta2*lc2*theta2*cos(theta2/2)))/(4*theta2^3),-(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2);
        (m0*sin(theta2/2)*(2*dtheta1 + dtheta2)*(4*lc2*sin(theta2/2) + lc1*theta2*cos(theta2/2)))/(4*theta2^2), -(m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2),(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2), 0;];

    G = [                    (g*lc2*m0*(cos(theta2)/2 - 1/2))/theta2;
        (3*g*m0)/2;
        -(g*lc2*m0*(sin(theta2) - theta2*cos(theta2)))/(2*theta2^2);
        (g*m0*cos(theta2/2)*sin(theta2/2))/theta2];
    condition  = 1;
elseif abs(theta1)> angle_th && abs(theta2) <= angle_th
    M = [(m0*(20*lc1^2 - 20*lc1^2*cos(theta1) + 9*lc1^2*theta1^2 + 4*lc2^2*theta1^4 + 8*lc1*lc2*theta1^2 - 20*lc1^2*theta1*sin(theta1) + lc1^2*theta1^2*cos(theta1) - 8*lc1*lc2*theta1^2*cos(theta1)))/(8*theta1^4) + (lc1^2*m0*sin(theta1/2)^2)/(4*theta1^2) + (lc1^2*m0*sin(theta1/2)^4)/(4*theta1^2) + (lc1^2*m0*cos(theta1/2)^2*sin(theta1/2)^2)/(4*theta1^2),(m0*(10*lc1*cos(theta1) - 2*lc2*theta1^2 - 10*lc1 + 5*lc1*theta1*sin(theta1) + 2*lc2*theta1^2*cos(theta1)))/(4*theta1^3), (lc2*m0*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2), (lc1*m0*(theta1 - sin(theta1)))/(2*theta1^2);
        ((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*((sin(theta1/2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1 + (sin(theta1/2)*(2*m0*cos(theta1/2)^2 + 2*m0*sin(theta1/2)^2))/theta1) + (lc2*m0*(cos(theta1) - 1))/(2*theta1) + (2*sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1, (2*sin(theta1/2)^2*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1^2 + (sin(theta1/2)*((sin(theta1/2)*(m0*cos(theta1/2)^2 + m0*sin(theta1/2)^2))/theta1 + (sin(theta1/2)*(2*m0*cos(theta1/2)^2 + 2*m0*sin(theta1/2)^2))/theta1))/theta1,(lc2*m0*(cos(theta1) - 1))/(4*theta1),(m0*cos(theta1/2)*sin(theta1/2))/theta1;
        (lc2*m0*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2),-(lc2*m0*sin(theta1/2)^2)/(2*theta1), (lc2^2*m0)/8,0;
        (lc1*m0*(theta1 - sin(theta1)))/(2*theta1^2), (m0*cos(theta1/2)*sin(theta1/2))/theta1,0, m0/4;
        ];

    C = [(m0*(4*dlc2*lc2*theta1^3 - 6*dtheta1*lc1^2 + 6*dtheta1*lc1^2*cos(theta1) + 6*dlc1*lc1*theta1 + 4*dlc2*lc1*theta1 - dtheta1*lc1*lc2*theta1^2 - 2*dtheta2*lc1*lc2*theta1^2 - 6*dlc1*lc1*theta1*cos(theta1) - 4*dlc2*lc1*theta1*cos(theta1) + 2*dlc1*lc2*theta1^2*sin(theta1) + 3*dtheta1*lc1^2*theta1*sin(theta1) + dtheta1*lc1*lc2*theta1^2*cos(theta1) + 2*dtheta2*lc1*lc2*theta1*sin(theta1)))/(8*theta1^3), -(m0*(3*dtheta1*lc1 + 2*dlc2*theta1 - 3*dtheta1*lc1*cos(theta1) - 2*dlc2*theta1*cos(theta1) + 2*dtheta1*lc2*theta1*sin(theta1) + dtheta2*lc2*theta1*sin(theta1)))/(4*theta1^2), (lc2*m0*(4*dlc2*theta1^2 - 4*dtheta1*lc1*sin(theta1) + 2*dlc1*theta1*sin(theta1) + 3*dtheta1*lc1*theta1 + dtheta1*lc1*theta1*cos(theta1)))/(16*theta1^2), -(m0*(4*dtheta1*lc1 - 2*dlc1*theta1 + 4*dtheta1*lc2*theta1^2 + 2*dtheta2*lc2*theta1^2 - 4*dtheta1*lc1*cos(theta1) + 2*dlc1*theta1*cos(theta1) - dtheta1*lc1*theta1*sin(theta1)))/(8*theta1^2);
        (dtheta1*m0*(3*lc1 - 3*lc1*cos(theta1) + lc2*theta1*sin(theta1)))/(4*theta1^2),0,(dtheta1*lc2*m0*sin(theta1))/(8*theta1), (dtheta1*m0*sin(theta1/2)^2)/(2*theta1);
        (m0*(2*dlc2*lc1 + 2*dlc2*lc2*theta1^2 - 2*dlc2*lc1*cos(theta1) - 2*dtheta1*lc1*lc2*theta1 - dtheta2*lc1*lc2*theta1 + 2*dtheta1*lc1*lc2*sin(theta1) + dtheta2*lc1*lc2*sin(theta1)))/(8*theta1^2),-(m0*sin(theta1/2)*(2*dlc2*sin(theta1/2) + 2*dtheta1*lc2*cos(theta1/2) + dtheta2*lc2*cos(theta1/2)))/(4*theta1),(dlc2*lc2*m0)/8, -(lc2*m0*(2*dtheta1 + dtheta2))/8;
        (m0*(2*dtheta1 + dtheta2)*(lc1 + lc2*theta1^2 - lc1*cos(theta1)))/(4*theta1^2), -(m0*sin(theta1/2)^2*(2*dtheta1 + dtheta2))/(2*theta1), (lc2*m0*(2*dtheta1 + dtheta2))/8, 0;];

    G = [3*g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (g*m0*(lc1 - lc1*cos(theta1) + lc2*theta1*sin(theta1)))/(2*theta1) - (g*lc1*m0*sin(theta1/2)^2)/(2*theta1);
        (3*g*m0*cos(theta1/2)*sin(theta1/2))/theta1;
        -(g*lc2*m0*sin(theta1))/4;
        (g*m0*cos(theta1))/2;];
    condition = 2;
else
    M = [(m0*(20*lc1^2*theta2^2 + 8*lc2^2*theta1^4 + 11*lc1^2*theta1^2*theta2^2 - 20*lc1^2*theta2^2*cos(theta1) - 8*lc2^2*theta1^4*cos(theta2) - 20*lc1^2*theta1*theta2^2*sin(theta1) - 8*lc1*lc2*theta1^3*theta2 - lc1^2*theta1^2*theta2^2*cos(theta1) - 8*lc1*lc2*theta1^2*theta2*sin(theta1 + theta2) + 8*lc1*lc2*theta1^3*theta2*cos(theta2) + 8*lc1*lc2*theta1^2*theta2*sin(theta1) + 8*lc1*lc2*theta1^2*theta2*sin(theta2)))/(8*theta1^4*theta2^2), -(m0*(10*lc1*theta2 - 10*lc1*theta2*cos(theta1) - 2*lc2*theta1^2*sin(theta1 + theta2) + 2*lc2*theta1^2*sin(theta1) + 2*lc2*theta1^2*sin(theta2) - 5*lc1*theta1*theta2*sin(theta1)))/(4*theta1^3*theta2), -(lc2*m0*(lc1 + lc1*cos(theta1 + theta2) - lc2*theta1^2 - lc1*cos(theta1) - lc1*cos(theta2) + lc1*theta1*sin(theta2) - lc1*theta2*sin(theta2) + lc2*theta1^2*cos(theta2) + lc1*theta2*sin(theta1 + theta2) - lc1*theta1*theta2*cos(theta2)))/(2*theta1^2*theta2^2), (lc1*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta1*sin(theta2) + 1))/(2*theta1^2*theta2);
        -(m0*(10*lc1*theta2 - 10*lc1*theta2*cos(theta1) - 2*lc2*theta1^2*sin(theta1 + theta2) + 2*lc2*theta1^2*sin(theta1) + 2*lc2*theta1^2*sin(theta2) - 5*lc1*theta1*theta2*sin(theta1)))/(4*theta1^3*theta2),-(5*m0*(cos(theta1) - 1))/(2*theta1^2), (lc2*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta2*sin(theta1 + theta2) - theta2*sin(theta2) + 1))/(2*theta1*theta2^2), -(m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + 1))/(2*theta1*theta2);
        -(lc2*m0*(lc1 + lc1*cos(theta1 + theta2) - lc2*theta1^2 - lc1*cos(theta1) - lc1*cos(theta2) + lc1*theta1*sin(theta2) - lc1*theta2*sin(theta2) + lc2*theta1^2*cos(theta2) + lc1*theta2*sin(theta1 + theta2) - lc1*theta1*theta2*cos(theta2)))/(2*theta1^2*theta2^2),(lc2*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta2*sin(theta1 + theta2) - theta2*sin(theta2) + 1))/(2*theta1*theta2^2),-(lc2^2*m0*(4*cos(theta2) + theta2^2*cos(theta2) + 4*theta2*sin(theta2) - 3*theta2^2 - 4))/(8*theta2^4), (lc2*m0*(2*cos(theta2) + theta2*sin(theta2) - 2))/(4*theta2^3);
        (lc1*m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + theta1*sin(theta2) + 1))/(2*theta1^2*theta2),-(m0*(cos(theta1 + theta2) - cos(theta1) - cos(theta2) + 1))/(2*theta1*theta2), (lc2*m0*(2*cos(theta2) + theta2*sin(theta2) - 2))/(4*theta2^3), -(m0*(cos(theta2) - 1))/(2*theta2^2);];

    C = [-(m0*(6*dtheta1*lc1^2*theta2^3 + 8*dtheta2*lc2^2*theta1^3 - 6*dtheta1*lc1^2*theta2^3*cos(theta1) - 8*dtheta2*lc2^2*theta1^3*cos(theta2) - 6*dlc1*lc1*theta1*theta2^3 - 8*dlc2*lc2*theta1^3*theta2 + 2*dlc1*lc2*theta1^2*theta2^2 + 4*dlc2*lc1*theta1^2*theta2^2 + 4*dlc2*lc1*theta1*theta2^2*sin(theta1 + theta2) + 6*dlc1*lc1*theta1*theta2^3*cos(theta1) + 8*dlc2*lc2*theta1^3*theta2*cos(theta2) - 4*dlc2*lc1*theta1*theta2^2*sin(theta1) - 4*dlc2*lc1*theta1*theta2^2*sin(theta2) + 2*dlc1*lc2*theta1^2*theta2^2*cos(theta1 + theta2) - 2*dlc1*lc2*theta1^2*theta2^2*cos(theta1) - 2*dlc1*lc2*theta1^2*theta2^2*cos(theta2) - 4*dlc2*lc1*theta1^2*theta2^2*cos(theta2) - 3*dtheta1*lc1^2*theta1*theta2^3*sin(theta1) - 4*dtheta2*lc2^2*theta1^3*theta2*sin(theta2) - 4*dtheta2*lc1*lc2*theta1^2*theta2 - dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta1 + theta2) + dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta1) + dtheta1*lc1*lc2*theta1^2*theta2^2*sin(theta2) + 4*dtheta2*lc1*lc2*theta1^2*theta2^2*sin(theta2) - 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta1 + theta2) + 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta1) + 4*dtheta2*lc1*lc2*theta1*theta2*sin(theta2) + 4*dtheta2*lc1*lc2*theta1*theta2^2*cos(theta1 + theta2) - 4*dtheta2*lc1*lc2*theta1*theta2^2*cos(theta2) + 4*dtheta2*lc1*lc2*theta1^2*theta2*cos(theta2)))/(8*theta1^3*theta2^3), -(m0*(3*dtheta1*lc1*theta2^2 - 2*dtheta1*lc2*theta1*theta2 + 2*dtheta2*lc2*theta1*sin(theta1 + theta2) - 2*dlc2*theta1*theta2*sin(theta1 + theta2) - 2*dtheta2*lc2*theta1*sin(theta1) - 2*dtheta2*lc2*theta1*sin(theta2) + 2*dlc2*theta1*theta2*sin(theta1) + 2*dlc2*theta1*theta2*sin(theta2) - 3*dtheta1*lc1*theta2^2*cos(theta1) - 2*dtheta1*lc2*theta1*theta2*cos(theta1 + theta2) - 2*dtheta2*lc2*theta1*theta2*cos(theta1 + theta2) + 2*dtheta1*lc2*theta1*theta2*cos(theta1) + 2*dtheta1*lc2*theta1*theta2*cos(theta2) + 2*dtheta2*lc2*theta1*theta2*cos(theta2)))/(4*theta1^2*theta2^2), (lc2*m0*(8*dtheta1*lc2*theta1^2 + 4*dlc2*theta1^2*theta2 - 3*dtheta1*lc1*theta1*theta2 - 4*dtheta1*lc1*theta2*sin(theta1 + theta2) + 2*dlc1*theta1*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*theta2*sin(theta1) + 4*dtheta1*lc1*theta2*sin(theta2) - 2*dlc1*theta1*theta2*sin(theta1) - 2*dlc1*theta1*theta2*sin(theta2) + 4*dtheta1*lc1*theta2^2*cos(theta1 + theta2) - 2*dlc1*theta1*theta2^2*cos(theta1 + theta2) - 4*dtheta1*lc1*theta2^2*cos(theta2) - 8*dtheta1*lc2*theta1^2*cos(theta2) + 2*dlc1*theta1*theta2^2*cos(theta2) - 4*dlc2*theta1^2*theta2*cos(theta2) + dtheta1*lc1*theta1*theta2^2*sin(theta1 + theta2) + 3*dtheta1*lc1*theta1*theta2^2*sin(theta2) - 4*dtheta1*lc2*theta1^2*theta2*sin(theta2) + dtheta1*lc1*theta1*theta2*cos(theta1 + theta2) - dtheta1*lc1*theta1*theta2*cos(theta1) + 3*dtheta1*lc1*theta1*theta2*cos(theta2)))/(8*theta1^2*theta2^3), -(m0*(8*dtheta1*lc2*theta1^2 + 4*dtheta2*lc2*theta1^2 - 3*dtheta1*lc1*theta1*theta2 - 4*dtheta1*lc1*theta2*sin(theta1 + theta2) + 2*dlc1*theta1*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*theta2*sin(theta1) + 4*dtheta1*lc1*theta2*sin(theta2) - 2*dlc1*theta1*theta2*sin(theta1) - 2*dlc1*theta1*theta2*sin(theta2) - 8*dtheta1*lc2*theta1^2*cos(theta2) - 4*dtheta2*lc2*theta1^2*cos(theta2) + dtheta1*lc1*theta1*theta2*cos(theta1 + theta2) - dtheta1*lc1*theta1*theta2*cos(theta1) + 3*dtheta1*lc1*theta1*theta2*cos(theta2)))/(8*theta1^2*theta2^2);
        (dtheta1*m0*(3*lc1*theta2 - lc2*theta1 - 3*lc1*theta2*cos(theta1) + lc2*theta1*cos(theta1) + lc2*theta1*cos(theta2) - lc2*theta1*cos(theta1 + theta2)))/(4*theta1^2*theta2), 0,-(dtheta1*lc2*m0*(sin(theta1) - sin(theta1 + theta2) + sin(theta2) + theta2*cos(theta1 + theta2) - theta2*cos(theta2)))/(4*theta1*theta2^2),(dtheta1*m0*(sin(theta1) - sin(theta1 + theta2) + sin(theta2)))/(4*theta1*theta2);
        -(m0*(8*dtheta1*lc2^2*theta1^2 + 8*dtheta2*lc2^2*theta1^2 - 8*dtheta1*lc2^2*theta1^2*cos(theta2) - 8*dtheta2*lc2^2*theta1^2*cos(theta2) - dtheta2*lc1*lc2*theta2^2 + 2*dlc2*lc1*theta1*theta2^2 - 4*dlc2*lc2*theta1^2*theta2 + 2*dlc2*lc1*theta2^2*sin(theta1 + theta2) - 2*dlc2*lc1*theta2^2*sin(theta1) - 2*dlc2*lc1*theta2^2*sin(theta2) + 4*dtheta1*lc1*lc2*theta2^2*cos(theta1 + theta2) + 3*dtheta2*lc1*lc2*theta2^2*cos(theta1 + theta2) - 4*dtheta1*lc1*lc2*theta2^2*cos(theta2) + dtheta2*lc1*lc2*theta2^2*cos(theta1) - 3*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 2*dlc2*lc1*theta1*theta2^2*cos(theta2) + 4*dlc2*lc2*theta1^2*theta2*cos(theta2) - 4*dtheta1*lc1*lc2*theta1*theta2 - 4*dtheta2*lc1*lc2*theta1*theta2 - 4*dtheta1*lc2^2*theta1^2*theta2*sin(theta2) - 4*dtheta2*lc2^2*theta1^2*theta2*sin(theta2) - 4*dtheta1*lc1*lc2*theta2*sin(theta1 + theta2) - 4*dtheta2*lc1*lc2*theta2*sin(theta1 + theta2) + 4*dtheta1*lc1*lc2*theta2*sin(theta1) + 4*dtheta1*lc1*lc2*theta2*sin(theta2) + 4*dtheta2*lc1*lc2*theta2*sin(theta1) + 4*dtheta2*lc1*lc2*theta2*sin(theta2) + 4*dtheta1*lc1*lc2*theta1*theta2*cos(theta2) + 4*dtheta2*lc1*lc2*theta1*theta2*cos(theta2) + 4*dtheta1*lc1*lc2*theta1*theta2^2*sin(theta2) + 3*dtheta2*lc1*lc2*theta1*theta2^2*sin(theta2)))/(8*theta1^2*theta2^3), (m0*(4*dtheta1*lc2*sin(theta1) + 4*dtheta1*lc2*sin(theta2) + 4*dtheta2*lc2*sin(theta1) + 4*dtheta2*lc2*sin(theta2) - 2*dlc2*theta2*sin(theta1) - 2*dlc2*theta2*sin(theta2) - dtheta2*lc2*theta2 - 4*dtheta1*lc2*sin(theta1 + theta2) - 4*dtheta2*lc2*sin(theta1 + theta2) + 2*dlc2*theta2*sin(theta1 + theta2) + 4*dtheta1*lc2*theta2*cos(theta1 + theta2) + 3*dtheta2*lc2*theta2*cos(theta1 + theta2) - 4*dtheta1*lc2*theta2*cos(theta2) + dtheta2*lc2*theta2*cos(theta1) - 3*dtheta2*lc2*theta2*cos(theta2)))/(8*theta1*theta2^2), (lc2*m0*(2*dlc2*theta2 - 2*dtheta2*lc2 + 2*dtheta2*lc2*cos(theta2) - 2*dlc2*theta2*cos(theta2) + dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^3),(lc2*m0*(cos(theta2) - 1)*(2*dtheta1 + dtheta2))/(4*theta2^2);
        (m0*(2*dtheta1 + dtheta2)*(2*lc2*theta1^2 + lc1*theta2*sin(theta1) + lc1*theta2*sin(theta2) - 2*lc2*theta1^2*cos(theta2) - lc1*theta1*theta2 - lc1*theta2*sin(theta1 + theta2) + lc1*theta1*theta2*cos(theta2)))/(4*theta1^2*theta2^2),-(m0*(2*dtheta1 + dtheta2)*(sin(theta1) - sin(theta1 + theta2) + sin(theta2)))/(4*theta1*theta2), -(lc2*m0*(cos(theta2) - 1)*(2*dtheta1 + dtheta2))/(4*theta2^2),0;
        ];
    G = [-(g*m0*(3*lc1*theta2*sin(theta1) - lc2*theta1^2*cos(theta1 + theta2) + lc2*theta1^2*cos(theta1) - 3*lc1*theta1*theta2*cos(theta1)))/(2*theta1^2*theta2);
        (3*g*m0*sin(theta1))/(2*theta1);
        (g*lc2*m0*(sin(theta1) - sin(theta1 + theta2) + theta2*cos(theta1 + theta2)))/(2*theta2^2);
        (g*m0*(sin(theta1 + theta2)/2 - sin(theta1)/2))/theta2;];
    condition =3;
end
% invM = inv(M);
temp_dx = M\([tauy1;fz1;tauy2;fz2] - [alpha1,alpha2,alpha3,alpha4]'...
    - diag([k1,k2,k3,k4])*[x(1);x(3);x(5);x(7)] ...
    - (diag([d1,d2,d3,d4]) + C)*[x(2);x(4);x(6);x(8)] ...
    - G ...
    );
dx = [x(2);temp_dx(1);x(4);temp_dx(2);x(6);temp_dx(3);x(8);temp_dx(4);];
end