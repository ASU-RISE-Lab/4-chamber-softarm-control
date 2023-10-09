function [dq8x1,y] = func2ndNoPmUnitnm(t,q4x1,u4x1,k1,k2,k3,k4,d1,d2,d3,d4,varargin)


theta1 =q4x1(1);lc1 = q4x1(2);theta2 =q4x1(3);lc2 = q4x1(4);
dtheta1 =q4x1(5);dlc1 = q4x1(6);dtheta2 =q4x1(7);dlc2 = q4x1(8);

u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);

Kmat = diag([k1,k2,k3,k4]);
Dmat = diag([d1,d2,d3,d4]);

[M4x4i,Gi,~] = funcMCGcalv2([theta1,lc1,theta2,lc2,dtheta1,dlc1,dtheta2,dlc2]);
dthetalc4x1 = M4x4i\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]-Dmat*[dtheta1;dlc1;dtheta2;dlc2]-Gi);
dq8x1=[dtheta1;dlc1;dtheta2;dlc2;dthetalc4x1];
   y = dq8x1;

end