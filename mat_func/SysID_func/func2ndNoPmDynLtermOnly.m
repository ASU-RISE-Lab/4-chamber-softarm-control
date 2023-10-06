function [dq2x1,y] = func2ndNoPmDynLtermOnly(t,q2x1,u2x1,k1,ko1,d1,a1,varargin)
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pm12 = u2x1(1);
pm13 = u2x1(2);

l11 =q2x1(1);

dl11 =q2x1(2);


u_pm_psi(1,1) = pm12 + pm13;

u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 ;

Kmat =k1;
Dmat = d1;
Amat = a1;
ddl4x1 = (Amat*u_pm_tf-Kmat*[l11;]-Dmat*[dl11;]+[ko1;]);
dq2x1=[dl11;ddl4x1];
   y = q2x1;

end