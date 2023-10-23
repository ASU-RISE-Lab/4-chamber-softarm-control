function [x_new] = funcRK4pm4ch_m(x4x1,alpha,beta,u4x1,h)


%% estimating k1
xk1 =x4x1;
dx = alpha*eye(length(x4x1))*[xk1]'+ beta*[u4x1]';
k1 = dx;

%% estimating k2
xk2 =x4x1+0.5*h*k1;


dx = alpha*eye(length(x4x1))*[xk2]'+ beta*[u4x1]';
k2 = dx;

%% estimating k3
xk3 =x4x1+0.5*h*k2;


dx = alpha*eye(length(x4x1))*[xk3]'+ beta*[u4x1]';
k3 = dx;
%% estimating k4
xk4 =x4x1+0.5*h*k3;

dx = alpha*eye(length(x4x1))*[xk4]'+ beta*[u4x1]';
k4 = dx;

x_new = x4x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end