function [dq5x1,y] = func1stWithPmDynSeg1(t,q5x1,u3x1,k11,k12,k21,k22,d11,d12,d21,d22,koff11,koff21,varargin)
alpha = -0.9665; beta = 0.9698;
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pd11 = u3x1(1);
pd12 = u3x1(2);
pd13 = u3x1(3);

pm11 = q5x1(1); pm12 = q5x1(2); pm13 = q5x1(3);
theta1 =q5x1(4);lc1 = q5x1(5);
dpm3x1 = alpha*eye(3)*[pm11,pm12,pm13]'+ beta*[pd11,pd12,pd13]';

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);

u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;


Kmat = [k11,k12;k21,k22];
Dmat = [d11,d12;d21,d22];

dthetalc2x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1]-[koff11;koff21]);
dq5x1=[dpm3x1;dthetalc2x1];
   y = q5x1;

end