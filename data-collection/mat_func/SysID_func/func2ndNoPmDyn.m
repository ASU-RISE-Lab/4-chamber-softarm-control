function [dq4x1,y] = func2ndNoPmDyn(t,q4x1,u2x1,k1,k2,d1,d2,varargin)
par_set.fz_a0 = 0.0015;par_set.tau_l0 = 0.0480;
pm11 = u2x1(1);
pm12 = u2x1(2);

theta1 =q4x1(1);lc1 = q4x1(2);
dtheta1 =q4x1(3);dlc1 = q4x1(4);

u_pm_tf = u2x1';



Kmat = diag([k1,k2]);
Dmat = diag([d1,d2]);
[M4x4i,Gi,~] = funcMCGcalv2([theta1;lc1;zeros(2,1);dtheta1;dlc1;zeros(2,1)]');
M2x2i = M4x4i(1:2,1:2);
ddthetalc2x1 = M2x2i\(u_pm_tf-Kmat*[theta1;lc1;]-Dmat*[dtheta1;dlc1;]-Gi(1:2));
dq4x1=[dtheta1;dlc1;ddthetalc2x1;];
   y = q4x1;

end