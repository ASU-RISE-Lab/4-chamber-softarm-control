function [x_new] = funcRK4fintvar_onestate(h,li,e0i,etai)
%% 

%% estimating k1
xold_k1 = e0i;
dtdintvar = -li*etai*sign(xold_k1);
k1 = [dtdintvar];


%% estimating k2
xold_k2 = e0i + 0.5*h*k1;
dtdintvar = -li*etai*sign(xold_k2);
k2 = [dtdintvar];

%% estimating k3
xold_k3 = e0i + 0.5*h*k2;
dtdintvar = -li*etai*sign(xold_k3);
k3 = [dtdintvar];
%% estimating k4
xold_k4 = e0i + 0.5*h*k3;
dtdintvar = -li*etai*sign(xold_k4);
k4 = [dtdintvar];

x_new = e0i + h/6*(k1 + 2*k2 + 2*k3 +k4);
end