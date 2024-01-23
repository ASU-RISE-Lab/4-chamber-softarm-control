function [x_new] = funcRK4elongv1_m(x8x1,u6x1,h,par_set)
%% 
kk1=28.3353; kk2 = 29788.3 ; kk3 = 23.2665; kk4 =16277.1;...
d1 =28.2389;d2= 16862.7 ;d3= 28.041;d4= 10502.5;...
koff1=-2822.11 ;koff2=-3071.37;
amp = 0.908219;
% kk1=28.3353; kk2 =2786.11; kk3 = 23.2665; kk4 =3342.61;...
% d1 =28.2389;d2= 94005;d3= 28.041;d4= 49174.2;...
% koff1=-9152.71;koff2=-9876.58; 
% amp = 2.90936;
alpha = -0.9665; beta = 0.9698;

pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
%% estimating k1
pm11 = x8x1(1); pm12 = x8x1(2); pm13 = x8x1(3);
pm21 = x8x1(4); pm22 = x8x1(5); pm23 = x8x1(6);
lc1 = x8x1(7); lc2 = x8x1(8); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk2,kk4]);
Dmat = diag([d2,d4]);

dq2x1 =  Dmat\(amp*[u_pm_tf(2);u_pm_tf(4);]-Kmat*[lc1;lc2;]-[koff1;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k1 = [dpm6x1;dq2x1];


%% estimating k2
pm11 = x8x1(1) + 0.5*h*k1(1); pm12 = x8x1(2)+ 0.5*h*k1(2); 
pm13 = x8x1(3)+ 0.5*h*k1(3); pm21 = x8x1(4)+ 0.5*h*k1(4);
pm22 = x8x1(5)+ 0.5*h*k1(5); pm23 = x8x1(6)+ 0.5*h*k1(6); 


lc1 = x8x1(7)+ 0.5*h*k1(7); 
lc2 = x8x1(8)+ 0.5*h*k1(8); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk2,kk4]);
Dmat = diag([d2,d4]);

dq2x1 =  Dmat\(amp*[u_pm_tf(2);u_pm_tf(4);]-Kmat*[lc1;lc2;]-[koff1;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k2 = [dpm6x1;dq2x1];

%% estimating k3
pm11 = x8x1(1) + 0.5*h*k2(1); pm12 = x8x1(2)+ 0.5*h*k2(2); 
pm13 = x8x1(3)+ 0.5*h*k2(3); pm21 = x8x1(4)+ 0.5*h*k2(4);
pm22 = x8x1(5)+ 0.5*h*k2(5); pm23 = x8x1(6)+ 0.5*h*k2(6); 


lc1 = x8x1(7)+ 0.5*h*k2(7); 
lc2 = x8x1(8)+ 0.5*h*k2(8); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk2,kk4]);
Dmat = diag([d2,d4]);

dq2x1 =  Dmat\(amp*[u_pm_tf(2);u_pm_tf(4);]-Kmat*[lc1;lc2;]-[koff1;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k3 = [dpm6x1;dq2x1];
%% estimating k4
pm11 = x8x1(1) + h*k3(1); pm12 = x8x1(2)+ h*k3(2); 
pm13 = x8x1(3)+ h*k3(3); pm21 = x8x1(4)+ h*k3(4);
pm22 = x8x1(5)+ h*k3(5); pm23 = x8x1(6)+ h*k3(6); 

lc1 = x8x1(7)+ 0.5*h*k3(7); 
lc2 = x8x1(8)+ 0.5*h*k3(8); 

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([kk2,kk4]);
Dmat = diag([d2,d4]);

dq2x1 =  Dmat\(amp*[u_pm_tf(2);u_pm_tf(4);]-Kmat*[lc1;lc2;]-[koff1;koff2]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k4 = [dpm6x1;dq2x1];

x_new = x8x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end