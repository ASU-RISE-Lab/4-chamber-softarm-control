function [x_new] = funcRK4_pm_minus_M(x8x1,u4x1,h,par_set)
ParVal    = {11;7985;13;9135;...
0.257;2629;0.0697;1228.9;};

kk1 =41.66;kk2 =852;kk3 =30.3006;kk4 =896.67;
d1= 18.3847;d2= 1709.77;d3= 20.9183;d4= 1730.97;





u_pm_tf(1,1) = u4x1(1);
u_pm_tf(2,1) = u4x1(2);
u_pm_tf(3,1) = u4x1(3);
u_pm_tf(4,1) = u4x1(4);
Kmat = diag([ParVal{1},ParVal{2},ParVal{3},ParVal{4}]);
Dmat = diag([ParVal{5},ParVal{6},ParVal{7},ParVal{8}]);
% Kmat = diag([kk1,kk2,kk3,kk4]);
% Dmat = diag([d1,d2,d3,d4]);

%% estimating k1
x8x1_k1 = x8x1;
[Mmati] = funcMCGcalv2(x8x1_k1);
ddq4x1 = inv(Mmati)*(u_pm_tf-Kmat*[x8x1_k1(1:4)]-Dmat*[x8x1_k1(5:8)]);

dx8x1 = [x8x1_k1(5:8);ddq4x1];

k1 = [dx8x1];


%% estimating k2
x8x1_k2 = x8x1_k1 + 0.5*h*k1;
[Mmati] = funcMCGcalv2(x8x1_k2);
ddq4x1 = inv(Mmati)*(u_pm_tf-Kmat*[x8x1_k2(1:4)]-Dmat*[x8x1_k2(5:8)]);

dx8x1 = [x8x1_k2(5:8);ddq4x1];

k2 = [dx8x1];

%% estimating k3
x8x1_k3 = x8x1_k2 + 0.5*h*k2;
[Mmati] = funcMCGcalv2(x8x1_k3);
ddq4x1 = inv(Mmati)*(u_pm_tf-Kmat*[x8x1_k3(1:4)]-Dmat*[x8x1_k3(5:8)]);

dx8x1 = [x8x1_k3(5:8);ddq4x1];

k3 = [dx8x1];
%% estimating k4
x8x1_k4 = x8x1_k3 + 0.5*h*k3;
[Mmati] = funcMCGcalv2(x8x1_k4);
ddq4x1 = inv(Mmati)*(u_pm_tf-Kmat*[x8x1_k4(1:4)]-Dmat*[x8x1_k4(5:8)]);

dx8x1 = [x8x1_k4(5:8);ddq4x1];

k4 = [dx8x1];

x_new = x8x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end