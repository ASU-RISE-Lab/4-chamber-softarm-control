function [dq10x1,y] = func1stWithPmDynAmp(t,q10x1,u6x1,k1,k2,k3,k4,d1,d2,d3,d4,koff1,koff2,a1,a2,a3,a4,varargin)
alpha = -0.9665; beta = 0.9698;
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
pm11 = q10x1(1); pm12 = q10x1(2); pm13 = q10x1(3);
pm21 = q10x1(4); pm22 = q10x1(5); pm23 = q10x1(6);
theta1 =q10x1(7);lc1 = q10x1(8);
theta2 =q10x1(9);lc2 = q10x1(10);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;

Kmat = diag([k1,k2,k3,k4]);
Dmat = diag([d1,d2,d3,d4]);
Amat = diag([a1,a2,a3,a4]);
dthetalc4x1 = Dmat\(Amat*u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-[0;koff1;0;koff2]);
dq10x1=[dpm6x1;dthetalc4x1];
   y = q10x1;

end