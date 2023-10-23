function [x_new] = funcRK4fintvar(xold,v4x1,destold,h,Lx,Fmat,Bmat)
%% 

%% estimating k1
xold_k1 = xold;
dtdintvar = Lx*(-Fmat*xold_k1 -Bmat*v4x1 - destold);
k1 = [dtdintvar];


%% estimating k2
xold_k2 = xold + 0.5*h*k1;

dtdintvar = Lx*(-Fmat*xold_k2 -Bmat*v4x1 - destold);
k2 = [dtdintvar];

%% estimating k3
xold_k3 = xold + 0.5*h*k2;

dtdintvar = Lx*(-Fmat*xold_k3 -Bmat*v4x1 - destold);
k3 = [dtdintvar];
%% estimating k4
xold_k4 = xold + 0.5*h*k3;

dtdintvar = Lx*(-Fmat*xold_k2 -Bmat*v4x1 - destold);
k4 = [dtdintvar];

x_new = xold + h/6*(k1 + 2*k2 + 2*k3 +k4);
end