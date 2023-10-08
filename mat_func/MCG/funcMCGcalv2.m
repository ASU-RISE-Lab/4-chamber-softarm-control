function [M,G,detM] = funcMCGcalv2(x)
theta1 = x(1); dtheta1 = x(5); lc1 = x(2); dlc1 = x(6);
theta2 = x(3); dtheta2 = x(7); lc2 = x(4); dlc2 = x(8);
angle_th = deg2rad(2);
a1=0.0430;a2 =0.0576;
m0 = (100 + 34*2 + 25*5)/1000; %kg
g = -9.8; %N/kg
if abs(theta1)<= angle_th && abs(theta2) <= angle_th
    M = [(lc2^2*m0)/4 + (m0*(3*lc1^2 + 4*lc1*lc2 + 2*lc2^2))/8,0, (lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0;
        0, (5*m0)/4, 0, m0/2;
        (lc2^2*m0)/8 + (lc2*m0*(lc1/2 + lc2/2))/4,0,(lc2^2*m0)/8,0;
        0, m0/2, 0, m0/4;
        ];

    C = [];

    G = [         0;
        (3*g*m0)/2;
        0;
        (g*m0)/2;];
    condition =0;
elseif abs(theta1)<= angle_th && abs(theta2) > angle_th
    M = [(m0*(2*a2 + (2*lc2*sin(theta2/2))/theta2)^2)/12 + (3*m0*(a2 + (lc2*sin(theta2/2))/theta2)^2)/4 + (m0*(15*lc2^2 - 15*lc2^2*cos(theta2) + 40*a1^2*theta2^2 + 30*a2^2*theta2^2 + 10*lc1^2*theta2^2 + 40*a1*lc1*theta2^2 + 48*a1*a2*theta2^2*cos(theta2/2) + 24*a2*lc1*theta2^2*cos(theta2/2) + 24*a1*lc2*theta2*sin(theta2) + 12*lc1*lc2*theta2*sin(theta2) + 60*a2*lc2*theta2*sin(theta2/2)))/(24*theta2^2),-(m0*sin(theta2/2)*(lc2*sin(theta2/2) + a2*theta2))/theta2, (m0*(2*a2 + (2*lc2*sin(theta2/2))/theta2)^2)/12 + (5*m0*(a2 + (lc2*sin(theta2/2))/theta2)^2)/12 + (m0*(5*a2^2*theta2^2 - 5*lc2^2*(cos(theta2)/2 - 1/2) + 6*a1*lc2*(cos(theta2) - 1) + 3*lc1*lc2*(cos(theta2) - 1) + 6*a1*a2*theta2^2*cos(theta2/2) + 3*a2*lc1*theta2^2*cos(theta2/2) + 6*a1*lc2*theta2*sin(theta2) + 3*lc1*lc2*theta2*sin(theta2) + 10*a2*lc2*theta2*sin(theta2/2)))/(12*theta2^2), -(m0*(2*a1 + lc1)*(cos(theta2)/2 - 1/2))/(2*theta2);
 -(m0*sin(theta2/2)*(lc2*sin(theta2/2) + a2*theta2))/theta2,(17*m0)/12,-(m0*(lc2*sin(theta2) - lc2*theta2*cos(theta2) + a2*theta2^2*sin(theta2/2)))/(2*theta2^2), (m0*cos(theta2/2)*sin(theta2/2))/theta2;
 (m0*(2*a2 + (2*lc2*sin(theta2/2))/theta2)^2)/12 + (5*m0*(a2 + (lc2*sin(theta2/2))/theta2)^2)/12 + (m0*(5*a2^2*theta2^2 - 5*lc2^2*(cos(theta2)/2 - 1/2) + 6*a1*lc2*(cos(theta2) - 1) + 3*lc1*lc2*(cos(theta2) - 1) + 6*a1*a2*theta2^2*cos(theta2/2) + 3*a2*lc1*theta2^2*cos(theta2/2) + 6*a1*lc2*theta2*sin(theta2) + 3*lc1*lc2*theta2*sin(theta2) + 10*a2*lc2*theta2*sin(theta2/2)))/(12*theta2^2), -(m0*(lc2*sin(theta2) - lc2*theta2*cos(theta2) + a2*theta2^2*sin(theta2/2)))/(2*theta2^2),(m0*(2*a2 + (2*lc2*sin(theta2/2))/theta2)^2)/12 + (5*m0*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2)^2)/3 + (m0*(a2 + (lc2*sin(theta2/2))/theta2)^2)/4, (5*m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/(3*theta2);
-(m0*(2*a1 + lc1)*(cos(theta2)/2 - 1/2))/(2*theta2), (m0*cos(theta2/2)*sin(theta2/2))/theta2,(5*m0*sin(theta2/2)*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2))/(3*theta2), (5*m0*sin(theta2/2)^2)/(3*theta2^2);];

    C = [ (m0*(3*dlc1*lc1*theta2^3 - 8*dtheta2*lc2^2 + 8*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 - dtheta1*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc1*lc2*theta2^2*sin(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 4*dtheta2*lc2^2*theta2*sin(theta2) + dtheta1*lc1*lc2*theta2^2*cos(theta2) + 2*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 2*dtheta2*lc1*lc2*theta2*sin(theta2)))/(8*theta2^3),-(m0*(4*dlc2*theta2 - 4*dtheta2*lc2 + 3*dtheta1*lc1*theta2^2 + 4*dtheta2*lc2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 4*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*(8*dtheta1*lc2 - 2*dlc1*theta2 + 4*dlc2*theta2 - 8*dtheta1*lc2*cos(theta2) + 2*dlc1*theta2*cos(theta2) - 4*dlc2*theta2*cos(theta2) + 2*dlc1*theta2^2*sin(theta2) + dtheta1*lc1*theta2*sin(theta2) - 4*dtheta1*lc2*theta2*sin(theta2) - dtheta1*lc1*theta2^2*cos(theta2)))/(8*theta2^3), -(m0*sin(theta2/2)*(8*dtheta1*lc2*sin(theta2/2) + 4*dtheta2*lc2*sin(theta2/2) - 2*dlc1*theta2*sin(theta2/2) + dtheta1*lc1*theta2*cos(theta2/2)))/(4*theta2^2);
        (dtheta1*m0*(3*lc1*theta2 + 2*lc2*sin(theta2)))/(8*theta2),0,-(dtheta1*lc2*m0*sin(theta2/2)*(sin(theta2/2) - theta2*cos(theta2/2)))/(2*theta2^2), (dtheta1*m0*sin(theta2/2)^2)/(2*theta2);
        (m0*(16*dtheta1*lc2^2*cos(theta2) - 16*dtheta2*lc2^2 - 16*dtheta1*lc2^2 + 16*dtheta2*lc2^2*cos(theta2) + 8*dlc2*lc2*theta2 + dtheta2*lc1*lc2*theta2^2 - 8*dlc2*lc2*theta2*cos(theta2) + 2*dlc2*lc1*theta2^2*sin(theta2) + 8*dtheta1*lc2^2*theta2*sin(theta2) + 8*dtheta2*lc2^2*theta2*sin(theta2) + 4*dtheta1*lc1*lc2*theta2^2*cos(theta2) + 3*dtheta2*lc1*lc2*theta2^2*cos(theta2) - 4*dtheta1*lc1*lc2*theta2*sin(theta2) - 4*dtheta2*lc1*lc2*theta2*sin(theta2)))/(16*theta2^3), -(m0*(2*dlc2*theta2 - 4*dtheta2*lc2 - 4*dtheta1*lc2 + 4*dtheta1*lc2*cos(theta2) + 4*dtheta2*lc2*cos(theta2) - 2*dlc2*theta2*cos(theta2) + 4*dtheta1*lc2*theta2*sin(theta2) + 3*dtheta2*lc2*theta2*sin(theta2)))/(8*theta2^2), (lc2*m0*sin(theta2/2)*(2*dlc2*theta2*sin(theta2/2) - 2*dtheta2*lc2*sin(theta2/2) + dtheta2*lc2*theta2*cos(theta2/2)))/(4*theta2^3),-(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2);
        (m0*sin(theta2/2)*(2*dtheta1 + dtheta2)*(4*lc2*sin(theta2/2) + lc1*theta2*cos(theta2/2)))/(4*theta2^2), -(m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2),(lc2*m0*sin(theta2/2)^2*(2*dtheta1 + dtheta2))/(2*theta2^2), 0;];

    G = [                               -(g*m0*sin(theta2/2)*(lc2*sin(theta2/2) + a2*theta2))/theta2
                                                                                 (3*g*m0)/2
-(g*m0*(lc2*sin(theta2) - lc2*theta2*cos(theta2) + a2*theta2^2*sin(theta2/2)))/(2*theta2^2)
                                                  (g*m0*cos(theta2/2)*sin(theta2/2))/theta2];
    condition  = 1;
elseif abs(theta1)> angle_th && abs(theta2) <= angle_th
    M = [(m0*(2*a1 + (2*lc1*sin(theta1/2))/theta1)^2)/6 + (m0*(a1 + (lc1*sin(theta1/2))/theta1)^2)/3 + (m0*(68*lc1^2 - 68*lc1^2*cos(theta1) + 16*a1^2*theta1^4 + 56*a2^2*theta1^4 + 25*lc1^2*theta1^2 + 14*lc2^2*theta1^4 + 48*a2*lc1*theta1^2 + 56*a2*lc2*theta1^4 + 24*lc1*lc2*theta1^2 - 68*lc1^2*theta1*sin(theta1) + 9*lc1^2*theta1^2*cos(theta1) + 48*a1*a2*theta1^4*cos(theta1/2) + 24*a1*lc2*theta1^4*cos(theta1/2) + 32*a1*lc1*theta1^3*sin(theta1/2) - 48*a2*lc1*theta1^2*cos(theta1) - 24*lc1*lc2*theta1^2*cos(theta1)))/(24*theta1^4), -(m0*sin(theta1/2)*(34*lc1*sin(theta1/2) - 17*lc1*theta1*cos(theta1/2) + 12*a2*theta1^2*sin(theta1/2) + 6*lc2*theta1^2*sin(theta1/2)))/(6*theta1^3), (m0*(2*a2 + lc2)*(12*lc1 - 12*lc1*cos(theta1/2)^2 + 14*a2*theta1^2 + 7*lc2*theta1^2 + 6*a1*theta1^2*cos(theta1/2)))/(24*theta1^2), (m0*(lc1*theta1 - lc1*sin(theta1) + a1*theta1^2*sin(theta1/2)))/(2*theta1^2);
 (17*m0*sin(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2))/(3*theta1) - (m0*sin(theta1/2)^2*(2*a2 + lc2))/theta1,(17*m0*sin(theta1/2)^2)/(3*theta1^2),(m0*(2*a2 + lc2)*(cos(theta1)/2 - 1/2))/(2*theta1),(m0*cos(theta1/2)*sin(theta1/2))/theta1;
 (m0*(2*a2 + lc2)*(12*lc1 - 12*lc1*cos(theta1/2)^2 + 14*a2*theta1^2 + 7*lc2*theta1^2 + 6*a1*theta1^2*cos(theta1/2)))/(24*theta1^2), -(m0*sin(theta1/2)^2*(2*a2 + lc2))/(2*theta1),(m0*(2*a2 + lc2)^2)/12 + (m0*(a2 + lc2/2)^2)/4,0;
(m0*(lc1*theta1 - lc1*sin(theta1) + a1*theta1^2*sin(theta1/2)))/(2*theta1^2), (m0*cos(theta1/2)*sin(theta1/2))/theta1, 0,(5*m0)/12;];

    C = [];
    G = [ 
3*g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (5*g*m0*sin(theta1/2)*(a1 + (lc1*sin(theta1/2))/theta1))/6 - (g*m0*(2*lc1 - 2*lc1*cos(theta1) + 6*a2*theta1*sin(theta1) + 3*lc2*theta1*sin(theta1) + 4*a1*theta1*sin(theta1/2)))/(6*theta1)
                                                                                                                                                                                                                                       (3*g*m0*cos(theta1/2)*sin(theta1/2))/theta1
                                                                                                                                                                                                                                                -(g*m0*sin(theta1)*(a2 + lc2/2))/2
                                                                                                                                                                                                                                                              (g*m0*cos(theta1))/2
];
    condition = 2;
else
    M = [(m0*(2*a2 + lc2)^2)/12 + (3*m0*(a2 + lc2/2)^2)/4 + (m0*(40*a1^2 + 48*a1*a2 + 40*a1*lc1 + 24*a1*lc2 + 30*a2^2 + 24*a2*lc1 + 30*a2*lc2 + 10*lc1^2 + 12*lc1*lc2 + (15*lc2^2)/2))/24,0, (m0*(6*a1*a2 + 3*a1*lc2 + 3*a2*lc1 + 5*a2*lc2 + (3*lc1*lc2)/2 + 5*a2^2 + (5*lc2^2)/4))/12 + (m0*(2*a2 + lc2)^2)/12 + (5*m0*(a2 + lc2/2)^2)/12, 0;
 0, (17*m0)/12, 0,m0/2;
 (m0*(6*a1*a2 + 3*a1*lc2 + 3*a2*lc1 + 5*a2*lc2 + (3*lc1*lc2)/2 + 5*a2^2 + (5*lc2^2)/4))/12 + (m0*(2*a2 + lc2)^2)/12 + (5*m0*(a2 + lc2/2)^2)/12,0,(m0*(2*a2 + lc2)^2)/12 + (m0*(a2 + lc2/2)^2)/4, 0;
 0, m0/2, 0, (5*m0)/12;];

    C = [];

    G = [3*g*m0*cos(theta1/2)*((lc1*cos(theta1/2))/(2*theta1) - (lc1*sin(theta1/2))/theta1^2) - (g*m0*(2*(sin(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) + 2*cos(theta1/2)*cos(theta2/2)*sin(theta1/2))*(a2 + (lc2*sin(theta2/2))/theta2) + 2*sin(theta1/2)*(a1 + (lc1*sin(theta1/2))/theta1)))/6 - (5*g*m0*sin(theta1/2)*(a1 + (lc1*sin(theta1/2))/theta1))/6 - (g*m0*((sin(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) + 2*cos(theta1/2)*cos(theta2/2)*sin(theta1/2))*(a2 + (lc2*sin(theta2/2))/theta2) + 2*sin(theta1/2)*(a1 + (lc1*sin(theta1/2))/theta1)))/6 - (g*m0*(sin(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) + 2*cos(theta1/2)*cos(theta2/2)*sin(theta1/2))*(a2 + (lc2*sin(theta2/2))/theta2))/2
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 (3*g*m0*cos(theta1/2)*sin(theta1/2))/theta1
                                                                                                                                                                                                                                                                                                                                                                                     g*m0*(cos(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) - 2*cos(theta1/2)*sin(theta1/2)*sin(theta2/2))*((lc2*cos(theta2/2))/(2*theta2) - (lc2*sin(theta2/2))/theta2^2) - (g*m0*(sin(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) + 2*cos(theta1/2)*cos(theta2/2)*sin(theta1/2))*(a2 + (lc2*sin(theta2/2))/theta2))/2
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (g*m0*sin(theta2/2)*(cos(theta2/2)*(cos(theta1/2)^2 - sin(theta1/2)^2) - 2*cos(theta1/2)*sin(theta1/2)*sin(theta2/2)))/theta2
];
    condition =3;
end
% condition    
% det(M)
detM = det(M);
invM = [];
end