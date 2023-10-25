function [x_new] = funcRK4fintvar_onestate_improve_v3( h,li,dtdxdi,eta0i,etai,e0i)
%% 
xold = e0i;
%% estimating k1
xold_k1 = xold ;
dtdintvar = li*(eta0i*xold_k1+etai*sign(xold_k1)-dtdxdi);
k1 = [dtdintvar];


%% estimating k2
xold_k2 = xold  + 0.5*h*k1;
dtdintvar = li*(eta0i*xold_k2+etai*sign(xold_k2)-dtdxdi);
k2 = [dtdintvar];

%% estimating k3
xold_k3 = xold  + 0.5*h*k2;
dtdintvar = li*(eta0i*xold_k3+etai*sign(xold_k3)-dtdxdi);
k3 = [dtdintvar];
%% estimating k4
xold_k4 = xold  + 0.5*h*k3;
dtdintvar = li*(eta0i*xold_k4+etai*sign(xold_k4)-dtdxdi);
k4 = [dtdintvar];

x_new = xold  + h/6*(k1 + 2*k2 + 2*k3 +k4);
end