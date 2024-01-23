function [x_new] = funcRK42segODE_m(x,alpha4x1,k4x1,d4x1,u,h)
m0 = (100 + 34*2 + 25*5)/1000; %kg
h0 = 0.01; %m
g = 9.8; %N/kg
tauy1 = u(1);
fz1 = u(2);
tauy2 = u(3);
fz2 = u(4);
ksp = diag(k4x1);
damp = diag(d4x1);
uoffset = alpha4x1;
%% estimating k1
theta1 = x(1); dtheta1 = x(2); lc1 = x(3); dlc1 = x(4);
theta2 = x(5); dtheta2 = x(6); lc2 = x(7); dlc2 = x(8);
M=[];C=[];G=[];condition =0;
[M,C,G,condition] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);

temp_dx = M\([tauy1;fz1;tauy2;fz2]...
    - ( C)*[x(2);x(4);x(6);x(8)] ...
    - G ...
    -(ksp*[x(1);x(3);x(5);x(7)] + uoffset)...
    -damp*[x(2);x(4);x(6);x(8)]);
k1 = [x(2);temp_dx(1);x(4);temp_dx(2);x(6);temp_dx(3);x(8);temp_dx(4);];

%% estimating k2
theta1 = x(1) + 0.5*h*k1(1); dtheta1 = x(2)+ 0.5*h*k1(2); 
lc1 = x(3)+ 0.5*h*k1(3); dlc1 = x(4)+ 0.5*h*k1(4);
theta2 = x(5)+ 0.5*h*k1(5); dtheta2 = x(6)+ 0.5*h*k1(6); 
lc2 = x(7)+ 0.5*h*k1(7); dlc2 = x(8)+ 0.5*h*k1(8);
M=[];C=[];G=[];condition =0;
[M,C,G,condition] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);

temp_dx = M\([tauy1;fz1;tauy2;fz2]...
    - ( C)*[x(2);x(4);x(6);x(8)] ...
    - G ...
    -(ksp*[x(1);x(3);x(5);x(7)] + uoffset)...
    -damp*[x(2);x(4);x(6);x(8)]);
k2 = [x(2);temp_dx(1);x(4);temp_dx(2);x(6);temp_dx(3);x(8);temp_dx(4);];

%% estimating k3
theta1 = x(1) + 0.5*h*k2(1); dtheta1 = x(2)+ 0.5*h*k2(2); 
lc1 = x(3)+ 0.5*h*k2(3); dlc1 = x(4)+ 0.5*h*k2(4);
theta2 = x(5)+ 0.5*h*k2(5); dtheta2 = x(6)+ 0.5*h*k2(6); 
lc2 = x(7)+ 0.5*h*k2(7); dlc2 = x(8)+ 0.5*h*k2(8);
M=[];C=[];G=[];condition =0;
[M,C,G,condition] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);

temp_dx = M\([tauy1;fz1;tauy2;fz2]...
    - ( C)*[x(2);x(4);x(6);x(8)] ...
    - G ...
    -(ksp*[x(1);x(3);x(5);x(7)] + uoffset)...
    -damp*[x(2);x(4);x(6);x(8)]);
k3 = [x(2);temp_dx(1);x(4);temp_dx(2);x(6);temp_dx(3);x(8);temp_dx(4);];
%% estimating k4
theta1 = x(1) + h*k3(1); dtheta1 = x(2)+ h*k3(2); 
lc1 = x(3)+ h*k3(3); dlc1 = x(4)+ h*k3(4);
theta2 = x(5)+ h*k3(5); dtheta2 = x(6)+ h*k3(6); 
lc2 = x(7)+ h*k3(7); dlc2 = x(8)+ h*k3(8);
M=[];C=[];G=[];condition =0;
[M,C,G,condition] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);

temp_dx = M\([tauy1;fz1;tauy2;fz2]...
    - ( C)*[x(2);x(4);x(6);x(8)] ...
    - G ...
    -(ksp*[x(1);x(3);x(5);x(7)] + uoffset)...
    -damp*[x(2);x(4);x(6);x(8)]);
k4 = [x(2);temp_dx(1);x(4);temp_dx(2);x(6);temp_dx(3);x(8);temp_dx(4);];

x_new = x + h/6*(k1 + 2*k2 + 2*k3 +k4);
end