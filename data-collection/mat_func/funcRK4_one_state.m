function [x_new] = funcRK4_one_state(xold,u,ki,di,h)



%% estimating k1
xk1 = xold;
dx = (u-ki*xk1)/di;

k1 = [dx];


%% estimating k2
xk2 = xk1 + 0.5*h*k1;
dx = (u-ki*xk2)/di;

k2 = [dx];

%% estimating k3
xk3 = xk2 + 0.5*h*k2;
dx = (u-ki*xk3)/di;

k3 = [dx];
%% estimating k4
xk4 = xk3 + 0.5*h*k3;
dx = (u-ki*xk4)/di;

k4 = [dx];

x_new = xold + h/6*(k1 + 2*k2 + 2*k3 +k4);
end