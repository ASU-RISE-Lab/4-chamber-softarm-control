function [x_new] = funcRK4NoPmDynSeg1_m(x2x1,u2x1,h,other_cont,par_set)
%% 
kk2 =3000;
d2= 1490;

alpha = -0.9665; beta = 0.9698;
Kmat = diag([kk2]);
Dmat = diag([d2]);

u_pm_tf(1,1) = u2x1(1);
u_pm_tf(2,1) = u2x1(2);
theta1 = other_cont(1);
theta2 = other_cont(2);
lc2 = other_cont(3);
dtheta1 = other_cont(4);
dtheta2 = other_cont(5);
dlc2 = other_cont(6);
%% estimating k1
lc1 = x2x1(1);
dlc1 = x2x1(2); 
state_vec = [theta1,lc1,theta2,lc2,dtheta1,dtheta2,dlc1,dlc2];
[Mmat,Gmat,~] = funcMCGcalv2(state_vec);
dq1x1 = Mmat(2,2)\(u_pm_tf(2)-Kmat*[lc1;]-Dmat*[dlc1;]-Gmat(2));


k1 = [x2x1(2);dq1x1];


%% estimating k2

lc1 = x2x1(1)+ 0.5*h*k1(1); 

dlc1 = x2x1(2)+ 0.5*h*k1(2); 




state_vec = [theta1,lc1,theta2,lc2,dtheta1,dtheta2,dlc1,dlc2];
[Mmat,Gmat,~] = funcMCGcalv2(state_vec);
dq1x1 = Mmat(2,2)\(u_pm_tf(2)-Kmat*[lc1;]-Dmat*[dlc1;]-Gmat(2));


k2 = [x2x1(2);dq1x1];

%% estimating k3
lc1 = x2x1(1)+ 0.5*h*k2(1); 

dlc1 = x2x1(2)+ 0.5*h*k2(2); 




state_vec = [theta1,lc1,theta2,lc2,dtheta1,dtheta2,dlc1,dlc2];
[Mmat,Gmat,~] = funcMCGcalv2(state_vec);
dq1x1 = Mmat(2,2)\(u_pm_tf(2)-Kmat*[lc1;]-Dmat*[dlc1;]-Gmat(2));


k3 = [x2x1(2);dq1x1];
%% estimating k4
lc1 = x2x1(1)+ 0.5*h*k3(1); 

dlc1 = x2x1(2)+ 0.5*h*k3(2); 




state_vec = [theta1,lc1,theta2,lc2,dtheta1,dtheta2,dlc1,dlc2];
[Mmat,Gmat,~] = funcMCGcalv2(state_vec);
dq1x1 = Mmat(2,2)\(u_pm_tf(2)-Kmat*[lc1;]-Dmat*[dlc1;]-Gmat(2));


k4 = [x2x1(2);dq1x1];

x_new = x2x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end