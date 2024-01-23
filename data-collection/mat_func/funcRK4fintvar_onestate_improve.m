function [x_new] = funcRK4fintvar_onestate_improve(h,li,pi,kki,di,xoldi,zoldi,intvaroldi)
%% 

%% estimating k1
xold_k1 = intvaroldi;
dtdintvar = li*(1/di*(-kki*xoldi-zoldi)-(xold_k1+pi));
k1 = [dtdintvar];


%% estimating k2
xold_k2 = intvaroldi + 0.5*h*k1;
dtdintvar = li*(1/di*(-kki*xoldi-zoldi)-(xold_k2+pi));
k2 = [dtdintvar];

%% estimating k3
xold_k3 = intvaroldi + 0.5*h*k2;
dtdintvar = li*(1/di*(-kki*xoldi-zoldi)-(xold_k3+pi));
k3 = [dtdintvar];
%% estimating k4
xold_k4 = intvaroldi+ 0.5*h*k3;
dtdintvar = li*(1/di*(-kki*xoldi-zoldi)-(xold_k4+pi));
k4 = [dtdintvar];

x_new = intvaroldi + h/6*(k1 + 2*k2 + 2*k3 +k4);
end