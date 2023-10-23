function [dq4x1,y] = func_otc22_1st_u_minus_M(t,q4x1,u4x1,k1,k2,k3,k4,d1,d2,d3,d4,varargin)
Kmat = diag([k1,k2,k3,k4]);
dq4x1 = diag([1/d1,1/d2,1/d3,1/d4])*(u4x1'-Kmat*q4x1);
y = q4x1;
end