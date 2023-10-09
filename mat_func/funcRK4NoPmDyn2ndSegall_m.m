function [x_new] = funcRK4NoPmDyn2ndSegall_m(x8x1,u4x1,h,par_set)
%% 
%%%
% kk1 =31.05;kk2 =2667.2;kk3 =20.7519;kk4 =3373.2;
% d1= 18.1638;d2= 944.3;d3= 29.6254;d4= 1084.5;
%%% u_pm_tf in Psi
kk1 =17.69;kk2 =9724;kk3 =13.43;kk4 =11585.1;
d1= 9.422;d2= 24280.4;d3= 9.244;d4= 39243.1;

a1=1 ;
a2 =1;
Kmat = diag([a1*kk1,a2*kk2,a1*kk3,a2*kk4]);
Dmat = diag([a1*d1,a2*d2,a1*d3,a2*d4]);

u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);

%% estimating k1
theta1 = x8x1(1);lc1 = x8x1(2);theta2 = x8x1(3);lc2 = x8x1(4);
dtheta1 = x8x1(5);dlc1 = x8x1(6);dtheta2 = x8x1(7);dlc2 = x8x1(8);

dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
[M4x4i,Gi,~] = funcMCGcalv2([theta1,lc1,theta2,lc2,dtheta1,dlc1,dtheta2,dlc2]);
Gi =zeros(4,1);
ddq4x1 = M4x4i\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2] - Dmat*[dtheta1;dlc1;dtheta2;dlc2]-Gi);


k1 = [dq4x1;ddq4x1];


%% estimating k2

 theta1 = x8x1(2)+ 0.5*h*k1(2); 
lc1 = x8x1(2)+ 0.5*h*k1(2); 
theta2 = x8x1(3)+ 0.5*h*k1(3); 
lc2 = x8x1(4)+ 0.5*h*k1(4); 

dtheta1 = x8x1(5)+ 0.5*h*k1(5); 
dlc1 = x8x1(6)+ 0.5*h*k1(6); 
dtheta2 = x8x1(7)+ 0.5*h*k1(7); 
dlc2 = x8x1(8)+ 0.5*h*k1(8); 
Gi =zeros(4,1);
[M4x4i,Gi,~] = funcMCGcalv2([theta1,lc1,theta2,lc2,dtheta1,dlc1,dtheta2,dlc2]);
Gi =zeros(4,1);
ddq4x1 = M4x4i\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2] - Dmat*[dtheta1;dlc1;dtheta2;dlc2]-Gi);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];

k2 = [dq4x1;ddq4x1];

%% estimating k3
theta1 = x8x1(1)+ 0.5*h*k2(1); 
lc1 = x8x1(2)+ 0.5*h*k2(2); 
theta2 = x8x1(3)+ 0.5*h*k2(3); 
lc2 = x8x1(4)+ 0.5*h*k2(4); 

dtheta1 = x8x1(5)+ 0.5*h*k2(5); 
dlc1 = x8x1(6)+ 0.5*h*k2(6); 
dtheta2 = x8x1(7)+ 0.5*h*k2(7); 
dlc2 = x8x1(8)+ 0.5*h*k2(8); 
Gi =zeros(4,1);
[M4x4i,Gi,~] = funcMCGcalv2([theta1,lc1,theta2,lc2,dtheta1,dlc1,dtheta2,dlc2]);
ddq4x1 = M4x4i\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2] - Dmat*[dtheta1;dlc1;dtheta2;dlc2]-Gi);
Gi =zeros(4,1);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];

k3 = [dq4x1;ddq4x1];
%% estimating k4
theta1 = x8x1(1)+ 0.5*h*k3(1); 
lc1 = x8x1(2)+ 0.5*h*k3(2); 
theta2 = x8x1(3)+ 0.5*h*k3(3); 
lc2 = x8x1(4)+ 0.5*h*k3(4); 

dtheta1 = x8x1(5)+ 0.5*h*k3(5); 
dlc1 = x8x1(6)+ 0.5*h*k3(6); 
dtheta2 = x8x1(7)+ 0.5*h*k3(7); 
dlc2 = x8x1(8)+ 0.5*h*k3(8); 


[M4x4i,Gi,~] = funcMCGcalv2([theta1,lc1,theta2,lc2,dtheta1,dlc1,dtheta2,dlc2]);
Gi =zeros(4,1);
ddq4x1 = M4x4i\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2] - Dmat*[dtheta1;dlc1;dtheta2;dlc2]-Gi);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];

k4 = [dq4x1;ddq4x1];

x_new = x8x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end