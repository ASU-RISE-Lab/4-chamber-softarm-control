function [dq2x1,y] = func1stNoPmDynElong(t,q2x1,u6x1,k1,k2,d1,d2,koff1,koff2,amp,varargin)
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pm11 = u6x1(1);
pm12 = u6x1(2);
pm13 = u6x1(3);
pm21 = u6x1(4);
pm22 = u6x1(5);
pm23 = u6x1(6);

lc1 = q2x1(1);
lc2 = q2x1(2);

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = (pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;

Kmat = diag([k1,k2]);
Dmat = diag([d1,d2]);

dlc2x1 = Dmat\(amp*[u_pm_tf(2);u_pm_tf(4);]-Kmat*[lc1;lc2;]-[koff1;koff2]);
dq2x1=dlc2x1;
   y = q2x1;

end