function [x_new] = funcRK4fintvar_onestate_v2(h,li,e0i,etai,eta0i)
%% 
xold = e0i;
%% estimating k1
xold_k1 = xold ;
dtdintvar = li*(eta0i*xold_k1+etai*sign(xold_k1));
k1 = [dtdintvar];


%% estimating k2
xold_k2 = xold  + 0.5*h*k1;
dtdintvar = li*(eta0i*xold_k2+etai*sign(xold_k2));
k2 = [dtdintvar];

%% estimating k3
xold_k3 = xold  + 0.5*h*k2;
dtdintvar = li*(eta0i*xold_k3+etai*sign(xold_k3));
k3 = [dtdintvar];
%% estimating k4
xold_k4 = xold  + 0.5*h*k3;
dtdintvar = li*(eta0i*xold_k4+etai*sign(xold_k4));
k4 = [dtdintvar];

x_new = xold  + h/6*(k1 + 2*k2 + 2*k3 +k4);
end