function [x_new] = funcRK4fullODEv3_m(x10x1,u6x1,h,par_set)
%% 
kk1=28.3353; kk2 =3.3031e04; kk3 = 23.2665; kk4 =1.5814e04;...
d1 =28.2389;d2= 3.9189e04;d3= 28.041;d4= 1.8099e04;...
koff1=-3140.72;koff2=-2901.08;
alpha = -0.9665; beta = 0.9698;

pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
%% estimating k1
pm11 = x10x1(1); pm12 = x10x1(2); pm13 = x10x1(3);
pm21 = x10x1(4); pm22 = x10x1(5); pm23 = x10x1(6);
theta1 = x10x1(7); lc1 = x10x1(8);theta2 = x10x1(9); lc2 = x10x1(10); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);

dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-[0;koff1;0;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k1 = [dpm6x1;dq4x1];


%% estimating k2
pm11 = x10x1(1) + 0.5*h*k1(1); pm12 = x10x1(2)+ 0.5*h*k1(2); 
pm13 = x10x1(3)+ 0.5*h*k1(3); pm21 = x10x1(4)+ 0.5*h*k1(4);
pm22 = x10x1(5)+ 0.5*h*k1(5); pm23 = x10x1(6)+ 0.5*h*k1(6); 

theta1 = x10x1(7)+ 0.5*h*k1(7);
lc1 = x10x1(8)+ 0.5*h*k1(8); 
theta2 = x10x1(9)+ 0.5*h*k1(9); 
lc2 = x10x1(10)+ 0.5*h*k1(10); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);

dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-[0;koff1;0;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k2 = [dpm6x1;dq4x1];

%% estimating k3
pm11 = x10x1(1) + 0.5*h*k2(1); pm12 = x10x1(2)+ 0.5*h*k2(2); 
pm13 = x10x1(3)+ 0.5*h*k2(3); pm21 = x10x1(4)+ 0.5*h*k2(4);
pm22 = x10x1(5)+ 0.5*h*k2(5); pm23 = x10x1(6)+ 0.5*h*k2(6); 

theta1 = x10x1(7)+ 0.5*h*k2(7);
lc1 = x10x1(8)+ 0.5*h*k2(8); 
theta2 = x10x1(9)+ 0.5*h*k2(9); 
lc2 = x10x1(10)+ 0.5*h*k2(10); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);

dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-[0;koff1;0;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k3 = [dpm6x1;dq4x1];
%% estimating k4
pm11 = x10x1(1) + h*k3(1); pm12 = x10x1(2)+ h*k3(2); 
pm13 = x10x1(3)+ h*k3(3); pm21 = x10x1(4)+ h*k3(4);
pm22 = x10x1(5)+ h*k3(5); pm23 = x10x1(6)+ h*k3(6); 
theta1 = x10x1(7)+ 0.5*h*k3(7);
lc1 = x10x1(8)+ 0.5*h*k3(8); 
theta2 = x10x1(9)+ 0.5*h*k3(9); 
lc2 = x10x1(10)+ 0.5*h*k3(10); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);

dq4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-[0;koff1;0;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k4 = [dpm6x1;dq4x1];

x_new = x10x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end