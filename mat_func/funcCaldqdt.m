function [dqdt] = funcCaldqdt(x14x1,u6x1,alpha,beta,gprMdl1,gprMdl2,gprMdl3,gprMdl4,par_set)
%% using pm/20 psi for GP INPUT
pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
%% estimating k1
pm11 = x14x1(1); pm12 = x14x1(2); pm13 = x14x1(3);
pm21 = x14x1(4); pm22 = x14x1(5); pm23 = x14x1(6);
theta1 = x14x1(7); dtheta1 =x14x1(8);
lc1 = x14x1(9); dlc1 =x14x1(10);
theta2 = x14x1(11); dtheta2 =x14x1(12);
lc2 = x14x1(13); dlc2 =x14x1(14);
statevar =[theta1,lc1,theta2,lc2];

xob1 = [theta1,dtheta1,pm11/20,pm12/20,pm13/20];
[ypred1,~,~] = predict(gprMdl1,xob1);

xob2 = [lc1,dlc1,pm11/20,pm12/20,pm13/20];
[ypred2,~,~] = predict(gprMdl2,xob2);

xob3 = [theta2,dtheta2,pm21/20,pm22/20,pm23/20];
[ypred3,~,~] = predict(gprMdl3,xob3);

xob4 = [lc2,dlc2,pm21/20,pm22/20,pm23/20];
[ypred4,~,~] = predict(gprMdl4,xob4);
ypred4x1 = [ypred1;ypred2;ypred3;ypred4];

u_pm_psi(1,1) = -(pm11 - pm12);
u_pm_psi(2,1) = -(pm11 + pm12 + 2*pm13);
u_pm_psi(3,1) = -(pm21 - pm22);
u_pm_psi(4,1) = -(pm21 + pm22 + 2*pm23);
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0 * par_set.tau_l0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;

[M4x4,C4x4,G4x1,c] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);
c1 =c;
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
ddq4x1 = M4x4\(u_pm_tf - ypred4x1-C4x4*x14x1(8:2:14)'-G4x1);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

dqdt = [dpm6x1;dq4x1(1);ddq4x1(1);dq4x1(2);ddq4x1(2);dq4x1(3);ddq4x1(3);dq4x1(4);ddq4x1(4);];

end