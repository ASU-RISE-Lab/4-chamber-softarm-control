function [dq4x1,y] = func1stNoPmDynLtermOnly(t,q4x1,u6x1,k1,k2,k3,k4,ko1,ko2,ko3,ko4,d1,d2,d3,d4,a1,a2,a3,a4,varargin)
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pm11 = u6x1(1);
pm12 = u6x1(2);
pm13 = u6x1(3);
pm21 = u6x1(4);
pm22 = u6x1(5);
pm23 = u6x1(6);
l11 =q4x1(1);l12 = q4x1(2);
l21 =q4x1(3);l22 = q4x1(4);

u_pm_psi(1,1) = pm12 + pm13;
u_pm_psi(2,1) = pm11 + pm13;
u_pm_psi(3,1) = pm22 + pm23;
u_pm_psi(4,1) = pm21 + pm23;
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 ;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 ;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;

Kmat =diag([k1,k2,k3,k4]);
Dmat = diag([d1,d2,d3,d4]);
Amat = diag([a1,a2,a3,a4]);
dl4x1 = Dmat\(Amat*u_pm_tf-Kmat*[l11;l12;l21;l22;]+[ko1;ko2;ko3;ko4;]);
dq4x1=[dl4x1];
   y = q4x1;

end