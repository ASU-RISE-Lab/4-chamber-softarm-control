function [dq4x1,y] = func1stNoPmUnitpsi(t,q4x1,u4x1,k1,k2,k3,k4,d1,d2,d3,d4,varargin)


theta1 =q4x1(1);lc1 = q4x1(2);
theta2 =q4x1(3);lc2 = q4x1(4);


u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);

Kmat = diag([k1,k2,k3,k4]);
Dmat = diag([d1,d2,d3,d4]);

dthetalc4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]);
dq4x1=dthetalc4x1;
   y = q4x1;

end