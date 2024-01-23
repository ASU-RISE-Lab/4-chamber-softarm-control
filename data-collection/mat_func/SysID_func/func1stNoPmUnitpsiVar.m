function [dq4x1,y] = func1stNoPmUnitpsiVar(t,q4x1,u4x1,ka1,ka2,ka3,ka4,kb1,kb2,kb3,kb4,da1,da2,da3,da4,db1,db2,db3,db4,varargin)


theta1 =q4x1(1);lc1 = q4x1(2);
theta2 =q4x1(3);lc2 = q4x1(4);


u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);

k1 = ka1*abs(u4x1(1))+kb1;
k2 = ka2*abs(u4x1(2))+kb2;
k3 = ka3*abs(u4x1(3))+kb3;
k4 = ka4*abs(u4x1(4))+kb4;

d1 = da1*abs(u4x1(1))+db1;
d2 = da2*abs(u4x1(2))+db2;
d3 = da3*abs(u4x1(3))+db3;
d4 = da4*abs(u4x1(4))+db4;

Kmat = diag([k1,k2,k3,k4]);
Dmat = diag([d1,d2,d3,d4]);

dthetalc4x1 = Dmat\(u_pm_tf-Kmat*[theta1;lc1;theta2;lc2;]);
dq4x1=dthetalc4x1;
   y = q4x1;

end