function [dq4x1,y] = func2ndNoPmDynNoM(t,q8x1,u4x1,k1,k2,d1,d2,varargin)
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pm11 = u4x1(1);
pm12 = u4x1(2);
% pm21 = u4x1(3);
% pm22 = u4x1(4);

theta1 =q8x1(1);lc1 = q8x1(2);
% theta2 =q8x1(3);lc2 = q8x1(4);
dtheta1 =q8x1(3);dlc1 = q8x1(4);
% dtheta2 =q8x1(7);dlc2 = q8x1(8);

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = (pm11 + pm12);
% u_pm_psi(3,1) = -(pm21 - pm22);
% u_pm_psi(4,1) = (pm21 + pm22);

u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
% u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
% u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


Kmat = diag([k1,k2]);
Dmat = diag([d1,d2]);
[~,Gi,~] = funcMCGcalv2([theta1;lc1;zeros(2,1);dtheta1;dlc1;zeros(2,1)]');
% M2x2i = M4x4i(1:2,1:2);
% invM2x2i = inv(M2x2i);
ddthetalc2x1 = (u_pm_tf-Kmat*[theta1;lc1;]-Dmat*[dtheta1;dlc1;]-Gi(1:2));
dq4x1=[dtheta1;dlc1;ddthetalc2x1;];
   y = q8x1;

end