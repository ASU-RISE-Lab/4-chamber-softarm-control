function [x_new] = funcRK4xdyn_m(x4x1,v4x1,h,Fmat,Bmat,D)
%% 
%%%

%% estimating k1
xk1 = x4x1;

dx = Fmat*xk1+Bmat*v4x1 + D;
k1 = [dx];


%% estimating k2
xk2 =x4x1+0.5*h*xk1;
dx = Fmat*xk2+Bmat*v4x1 + D;
k2 = [dx];

%% estimating k3
xk3 =x4x1+0.5*h*xk2;
dx = Fmat*xk3+Bmat*v4x1 + D;
k3 = [dx];
%% estimating k4
xk4 =x4x1+0.5*h*xk2;
dx = Fmat*xk4+Bmat*v4x1 + D;
k4 = [dx];

x_new = x4x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end