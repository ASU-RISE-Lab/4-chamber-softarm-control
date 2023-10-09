function [x_new] = funcRK4NoPmDynSegall_m(x4x1,u4x1,h,par_set)
%% 
%%%
% kk1 =31.05;kk2 =2667.2;kk3 =20.7519;kk4 =3373.2;
% d1= 18.1638;d2= 944.3;d3= 29.6254;d4= 1084.5;
%%%
kk1 =31.66;kk2 =952;kk3 =30.3006;kk4 =1096.67;
d1= 18.3847;d2= 2709.77;d3= 20.9183;d4= 3430.97;

Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);

u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);

%% estimating k1
theta1 = x4x1(1);lc1 = x4x1(2);theta2 = x4x1(3);lc2 = x4x1(4);

dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2]);


k1 = [dq4x1];


%% estimating k2

 theta1 = x4x1(2)+ 0.5*h*k1(2); 
lc1 = x4x1(2)+ 0.5*h*k1(2); 
theta2 = x4x1(3)+ 0.5*h*k1(3); 
lc2 = x4x1(4)+ 0.5*h*k1(4); 



dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2]);


k2 = [dq4x1];

%% estimating k3
theta1 = x4x1(1)+ 0.5*h*k2(1); 
lc1 = x4x1(2)+ 0.5*h*k2(2); 
theta2 = x4x1(3)+ 0.5*h*k2(3); 
lc2 = x4x1(4)+ 0.5*h*k2(4); 



dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2]);


k3 = [dq4x1];
%% estimating k4
theta1 = x4x1(1)+ 0.5*h*k3(1); 
lc1 = x4x1(2)+ 0.5*h*k3(2); 
theta2 = x4x1(3)+ 0.5*h*k3(3); 
lc2 = x4x1(4)+ 0.5*h*k3(4); 



dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2]);


k4 = [dq4x1];

x_new = x4x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end