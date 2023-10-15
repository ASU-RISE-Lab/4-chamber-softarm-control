function [xk_est,Pk] =funcEKF_improve(xk_old,uk,zk,Pk_old,Q,R,H,dT,e_max,e_min,x_min,x_max)
theta1 = xk_old(1);lc1 = xk_old(2);
theta2 = xk_old(3);lc2 = xk_old(4);

pm11 = xk_old(5); pm12 = xk_old(6);
pm21 = xk_old(7); pm22 = xk_old(8);

pd11 = uk(1);pd12 = uk(2);
pd21 = uk(3);pd22 = uk(4);

alphap = -0.9665; betap = 0.9698;
k1a2=-1.767; k1a1=17.55; k1a0 =33.471;
k2a2=10.25; k2a1=-325.1; k2a0 = 3299;
k3a2=-1.013; k3a1=11.55; k3a0 =4.419;
k4a2=15.34; k4a1=-474.1; k4a0 =4475;

d1a2=0.9725; d1a1=-11.2; d1a0 =38.471;
d2a2=-0.9725; d2a1=30.23; d2a0 =435.471;
d3a2=0.1125; d3a1=-1.2; d3a0=14.471;
d4a2=4.34; d4a1=-155.21; d4a0=2146;

F_old = [theta1 - dT*(pm11 - pm12 + (theta1*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)));
lc1 + dT*(pm11 + pm12 - (lc1*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0));
theta2 - dT*(pm21 - pm22 + (theta2*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)));
lc2 + dT*(pm21 + pm22 - (lc2*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0));
pm11 + dT*(alphap*pm11 + betap*pd11);
pm12 + dT*(alphap*pm12 + betap*pd12);
pm21 + dT*(alphap*pm21 + betap*pd21);
pm22 + dT*(alphap*pm22 + betap*pd22);];
if pm11-pm12 ==0 && pm21-pm22 ~=0 
    A_old = [ 1 - (dT*k1a0)/d1a0,0,0,0, 0, 0, 0, 0;
 0, 1 - (dT*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0),0,0, 0, 0, 0, 0;
 0,0, 1 - (dT*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)),0, 0, 0, 0, 0;
 0,0,0, 1 - (dT*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0), 0, 0, 0, 0;
 -dT, dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, alphap*dT + 1, 0, 0, 0;
dT, dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, 0, alphap*dT + 1, 0, 0;
 0,0, -dT*((theta2*(k3a2*(2*pm21 - 2*pm22) + (0.5000*k3a1*(2*pm21 - 2*pm22))/(pm21^2 - 2*pm21*pm22 + pm22^2)^0.5000))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)) - (theta2*(d3a2*(2*pm21 - 2*pm22) + (0.5000*d3a1*(2*pm21 - 2*pm22))/(pm21^2 - 2*pm21*pm22 + pm22^2)^0.5000)*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2))^2 + 1), dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, alphap*dT + 1, 0;
 0,0,dT*((theta2*(k3a2*(2*pm21 - 2*pm22) + (0.5000*k3a1*(2*pm21 - 2*pm22))/(pm21^2 - 2*pm21*pm22 + pm22^2)^0.5000))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)) - (theta2*(d3a2*(2*pm21 - 2*pm22) + (0.5000*d3a1*(2*pm21 - 2*pm22))/(pm21^2 - 2*pm21*pm22 + pm22^2)^0.5000)*(k3a0 + k3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + k3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2)))/(d3a0 + d3a2*(pm21^2 - 2*pm21*pm22 + pm22^2) + d3a1*(pm21^2 - 2*pm21*pm22 + pm22^2)^(1/2))^2 + 1), dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, 0, alphap*dT + 1;
 ];
    
elseif pm11-pm12 ~=0 && pm21-pm22 ==0 
     A_old = [1 - (dT*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)),0,0,0, 0, 0, 0, 0;
 0, 1 - (dT*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0),0,0, 0, 0, 0, 0;
 0,0, 1 - (dT*k3a0)/d3a0,0, 0, 0, 0, 0;
 0,0,0, 1 - (dT*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0), 0, 0, 0, 0;
-dT*((theta1*(k1a2*(2*pm11 - 2*pm12) + (0.5000*k1a1*(2*pm11 - 2*pm12))/(pm11^2 - 2*pm11*pm12 + pm12^2)^0.5000))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)) - (theta1*(d1a2*(2*pm11 - 2*pm12) + (0.5000*d1a1*(2*pm11 - 2*pm12))/(pm11^2 - 2*pm11*pm12 + pm12^2)^0.5000)*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2))^2 + 1), dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, alphap*dT + 1, 0, 0, 0;
 dT*((theta1*(k1a2*(2*pm11 - 2*pm12) + (0.5000*k1a1*(2*pm11 - 2*pm12))/(pm11^2 - 2*pm11*pm12 + pm12^2)^0.5000))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)) - (theta1*(d1a2*(2*pm11 - 2*pm12) + (0.5000*d1a1*(2*pm11 - 2*pm12))/(pm11^2 - 2*pm11*pm12 + pm12^2)^0.5000)*(k1a0 + k1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + k1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2)))/(d1a0 + d1a2*(pm11^2 - 2*pm11*pm12 + pm12^2) + d1a1*(pm11^2 - 2*pm11*pm12 + pm12^2)^(1/2))^2 + 1), dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, 0, alphap*dT + 1, 0, 0;
 0,0,-dT, dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, alphap*dT + 1, 0;
 0,0, dT, dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, 0, alphap*dT + 1;
        ];

elseif pm11-pm12 ==0 && pm21-pm22 ==0 
    A_old = [1 - (dT*k1a0)/d1a0,0,0,0, 0, 0, 0, 0;
 0, 1 - (dT*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0),0,0, 0, 0, 0, 0;
 0,0, 1 - (dT*k3a0)/d3a0,0, 0, 0, 0, 0;
 0,0,0, 1 - (dT*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0), 0, 0, 0, 0;
 -dT, dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, alphap*dT + 1, 0, 0, 0;
dT, dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, 0, alphap*dT + 1, 0, 0;
 0,0,-dT, dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, alphap*dT + 1, 0;
 0,0, dT, dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, 0, alphap*dT + 1;];

else
A_old = [1 - (dT*(k1a0 + k1a1*((pm11 - pm12)^2)^(1/2) + k1a2*pm11^2 + k1a2*pm12^2 - 2*k1a2*pm11*pm12))/(d1a0 + d1a1*((pm11 - pm12)^2)^(1/2) + d1a2*pm11^2 + d1a2*pm12^2 - 2*d1a2*pm11*pm12),0,0,0, 0, 0, 0, 0;
 0, 1 - (dT*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0),0,0, 0, 0, 0, 0;
 0,0, 1 - (dT*(k3a0 + k3a1*((pm21 - pm22)^2)^(1/2) + k3a2*pm21^2 + k3a2*pm22^2 - 2*k3a2*pm21*pm22))/(d3a0 + d3a1*((pm21 - pm22)^2)^(1/2) + d3a2*pm21^2 + d3a2*pm22^2 - 2*d3a2*pm21*pm22),0, 0, 0, 0, 0;
 0,0,0, 1 - (dT*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0), 0, 0, 0, 0;
-dT*((theta1*(pm11 - pm12)*(k1a1 + 2*k1a2*((pm11 - pm12)^2)^(1/2)))/(((pm11 - pm12)^2)^0.5000*(d1a0 + d1a1*((pm11 - pm12)^2)^(1/2) + d1a2*pm11^2 + d1a2*pm12^2 - 2*d1a2*pm11*pm12)) - (theta1*(pm11 - pm12)*(d1a1 + 2*d1a2*((pm11 - pm12)^2)^(1/2))*(k1a0 + k1a1*((pm11 - pm12)^2)^(1/2) + k1a2*pm11^2 + k1a2*pm12^2 - 2*k1a2*pm11*pm12))/(((pm11 - pm12)^2)^0.5000*(d1a0 + d1a1*((pm11 - pm12)^2)^(1/2) + d1a2*pm11^2 + d1a2*pm12^2 - 2*d1a2*pm11*pm12)^2) + 1), dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, alphap*dT + 1, 0, 0, 0;
 dT*((theta1*(pm11 - pm12)*(k1a1 + 2*k1a2*((pm11 - pm12)^2)^(1/2)))/(((pm11 - pm12)^2)^0.5000*(d1a0 + d1a1*((pm11 - pm12)^2)^(1/2) + d1a2*pm11^2 + d1a2*pm12^2 - 2*d1a2*pm11*pm12)) - (theta1*(pm11 - pm12)*(d1a1 + 2*d1a2*((pm11 - pm12)^2)^(1/2))*(k1a0 + k1a1*((pm11 - pm12)^2)^(1/2) + k1a2*pm11^2 + k1a2*pm12^2 - 2*k1a2*pm11*pm12))/(((pm11 - pm12)^2)^0.5000*(d1a0 + d1a1*((pm11 - pm12)^2)^(1/2) + d1a2*pm11^2 + d1a2*pm12^2 - 2*d1a2*pm11*pm12)^2) + 1), dT*((lc1*(d2a1 + 2*d2a2*pm11 + 2*d2a2*pm12)*(k2a2*pm11^2 + 2*k2a2*pm11*pm12 + k2a1*pm11 + k2a2*pm12^2 + k2a1*pm12 + k2a0))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0)^2 - (lc1*(k2a1 + 2*k2a2*pm11 + 2*k2a2*pm12))/(d2a2*pm11^2 + 2*d2a2*pm11*pm12 + d2a1*pm11 + d2a2*pm12^2 + d2a1*pm12 + d2a0) + 1),0,0, 0, alphap*dT + 1, 0, 0;
 0,0, -dT*((theta2*(pm21 - pm22)*(k3a1 + 2*k3a2*((pm21 - pm22)^2)^(1/2)))/(((pm21 - pm22)^2)^0.5000*(d3a0 + d3a1*((pm21 - pm22)^2)^(1/2) + d3a2*pm21^2 + d3a2*pm22^2 - 2*d3a2*pm21*pm22)) - (theta2*(pm21 - pm22)*(d3a1 + 2*d3a2*((pm21 - pm22)^2)^(1/2))*(k3a0 + k3a1*((pm21 - pm22)^2)^(1/2) + k3a2*pm21^2 + k3a2*pm22^2 - 2*k3a2*pm21*pm22))/(((pm21 - pm22)^2)^0.5000*(d3a0 + d3a1*((pm21 - pm22)^2)^(1/2) + d3a2*pm21^2 + d3a2*pm22^2 - 2*d3a2*pm21*pm22)^2) + 1), dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, alphap*dT + 1, 0;
 0,0,dT*((theta2*(pm21 - pm22)*(k3a1 + 2*k3a2*((pm21 - pm22)^2)^(1/2)))/(((pm21 - pm22)^2)^0.5000*(d3a0 + d3a1*((pm21 - pm22)^2)^(1/2) + d3a2*pm21^2 + d3a2*pm22^2 - 2*d3a2*pm21*pm22)) - (theta2*(pm21 - pm22)*(d3a1 + 2*d3a2*((pm21 - pm22)^2)^(1/2))*(k3a0 + k3a1*((pm21 - pm22)^2)^(1/2) + k3a2*pm21^2 + k3a2*pm22^2 - 2*k3a2*pm21*pm22))/(((pm21 - pm22)^2)^0.5000*(d3a0 + d3a1*((pm21 - pm22)^2)^(1/2) + d3a2*pm21^2 + d3a2*pm22^2 - 2*d3a2*pm21*pm22)^2) + 1), dT*((lc2*(d4a1 + 2*d4a2*pm21 + 2*d4a2*pm22)*(k4a2*pm21^2 + 2*k4a2*pm21*pm22 + k4a1*pm21 + k4a2*pm22^2 + k4a1*pm22 + k4a0))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0)^2 - (lc2*(k4a1 + 2*k4a2*pm21 + 2*k4a2*pm22))/(d4a2*pm21^2 + 2*d4a2*pm21*pm22 + d4a1*pm21 + d4a2*pm22^2 + d4a1*pm22 + d4a0) + 1), 0, 0, 0, alphap*dT + 1;];
end
xk_est_pre = F_old;
Pk_pre = A_old*Pk_old*A_old'+Q;
Lk = (Pk_pre*H')/(H*Pk_pre*H' + R);
%%%%%%%%%
 ek = zk-H*xk_est_pre;
 amp = ones(8,1);
for i =5:8
ek_out_i = abs(ek/zk(i));
if ek_out_i >= e_max
    amp(i) = 1.5;
elseif ek_out_i <= e_min
    amp(i) = 0.5;
else
    amp(i) = 1;
end
end
xk_est= xk_est_pre + diag(amp)*Lk*(zk -H*xk_est_pre);
for i =5:8
    if xk_est(i)< x_min(i) || xk_est(i)> x_max(i)
        amp(i) = 0.001;
    end
end
xk_est= xk_est_pre + diag(amp)*Lk*(zk -H*xk_est_pre);
%%%%%%%%%
% xk_est= xk_est_pre + Lk*(zk -H*xk_est_pre);
Pk = (eye(8) - Lk*H)*Pk_pre;
if isreal(xk_est) == 0
    xk_old
end
end