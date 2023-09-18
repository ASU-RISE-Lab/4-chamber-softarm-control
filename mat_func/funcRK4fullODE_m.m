function [x_new] = funcRK4fullODE_m(x14x1,alpha,beta,u6x1,h,gprMdl1,gprMdl2,gprMdl3,gprMdl4)
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
xob1 = [theta1,dtheta1,pm11,pm12,pm13];
[ypred1,~,~] = predict(gprMdl1,xob1);

xob2 = [lc1,dlc1,pm11,pm12,pm13];
[ypred2,~,~] = predict(gprMdl2,xob2);

xob3 = [theta2,dtheta2,pm21,pm22,pm23];
[ypred3,~,~] = predict(gprMdl3,xob3);

xob4 = [lc2,dlc2,pm21,pm22,pm23];
[ypred4,~,~] = predict(gprMdl4,xob4);
ypred4x1 = [ypred1;ypred2;ypred3;ypred4];

[M4x4,C4x1,G4x1,~] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);
inv(M4x4)
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
ddq4x1 = M4x4\(ypred4x1-C4x1*x14x1(8:2:14)-G4x1);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k1 = [dpm6x1;dq4x1(1);ddq4x1(1);dq4x1(2);ddq4x1(2);dq4x1(3);ddq4x1(3);dq4x1(4);ddq4x1(4);];

%% estimating k2
pm11 = x14x1(1) + 0.5*h*k1(1); pm12 = x14x1(2)+ 0.5*h*k1(2); 
pm13 = x14x1(3)+ 0.5*h*k1(3); pm21 = x14x1(4)+ 0.5*h*k1(4);
pm22 = x14x1(5)+ 0.5*h*k1(5); pm23 = x14x1(6)+ 0.5*h*k1(6); 
theta1 = x14x1(7)+ 0.5*h*k1(7);dtheta1 =x14x1(8)+ 0.5*h*k1(8);
lc1 = x14x1(9)+ 0.5*h*k1(9); dlc1 =x14x1(10)+ 0.5*h*k1(10);
theta2 = x14x1(11)+ 0.5*h*k1(11); dtheta2 =x14x1(12)+ 0.5*h*k1(12);
lc2 = x14x1(13)+ 0.5*h*k1(13); dlc2 =x14x1(14)+ 0.5*h*k1(14);
xob1 = [theta1,dtheta1,pm11,pm12,pm13];
[ypred1,~,~] = predict(gprMdl1,xob1);

xob2 = [lc1,dlc1,pm11,pm12,pm13];
[ypred2,~,~] = predict(gprMdl2,xob2);

xob3 = [theta2,dtheta2,pm21,pm22,pm23];
[ypred3,~,~] = predict(gprMdl3,xob3);

xob4 = [lc2,dlc2,pm21,pm22,pm23];
[ypred4,~,~] = predict(gprMdl4,xob4);
ypred4x1 = [ypred1;ypred2;ypred3;ypred4];

[M4x4,C4x1,G4x1,~] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
ddq4x1 = M4x4\(ypred4x1-C4x1*x14x1(8:2:14)-G4x1);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k2 = [dpm6x1;dq4x1(1);ddq4x1(1);dq4x1(2);ddq4x1(2);dq4x1(3);ddq4x1(3);dq4x1(4);ddq4x1(4);];

%% estimating k3
pm11 = x14x1(1) + 0.5*h*k2(1); pm12 = x14x1(2)+ 0.5*h*k2(2); 
pm13 = x14x1(3)+ 0.5*h*k2(3); pm21 = x14x1(4)+ 0.5*h*k2(4);
pm22 = x14x1(5)+ 0.5*h*k2(5); pm23 = x14x1(6)+ 0.5*h*k2(6); 

theta1 = x14x1(7)+ 0.5*h*k2(7);dtheta1 =x14x1(8)+ 0.5*h*k2(8);
lc1 = x14x1(9)+ 0.5*h*k2(9); dlc1 =x14x1(10)+ 0.5*h*k2(10);
theta2 = x14x1(11)+ 0.5*h*k2(11); dtheta2 =x14x1(12)+ 0.5*h*k2(12);
lc2 = x14x1(13)+ 0.5*h*k2(13); dlc2 =x14x1(14)+ 0.5*h*k2(14);
xob1 = [theta1,dtheta1,pm11,pm12,pm13];
[ypred1,~,~] = predict(gprMdl1,xob1);

xob2 = [lc1,dlc1,pm11,pm12,pm13];
[ypred2,~,~] = predict(gprMdl2,xob2);

xob3 = [theta2,dtheta2,pm21,pm22,pm23];
[ypred3,~,~] = predict(gprMdl3,xob3);

xob4 = [lc2,dlc2,pm21,pm22,pm23];
[ypred4,~,~] = predict(gprMdl4,xob4);
ypred4x1 = [ypred1;ypred2;ypred3;ypred4];

[M4x4,C4x1,G4x1,~] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
ddq4x1 = M4x4\(ypred4x1-C4x1*x14x1(8:2:14)-G4x1);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k3 = [dpm6x1;dq4x1(1);ddq4x1(1);dq4x1(2);ddq4x1(2);dq4x1(3);ddq4x1(3);dq4x1(4);ddq4x1(4);];
%% estimating k4
pm11 = x14x1(1) + h*k3(1); pm12 = x14x1(2)+ h*k3(2); 
pm13 = x14x1(3)+ h*k3(3); pm21 = x14x1(4)+ h*k3(4);
pm22 = x14x1(5)+ h*k3(5); pm23 = x14x1(6)+ h*k3(6); 
theta1 = x14x1(7)+ 0.5*h*k3(7);dtheta1 =x14x1(8)+ 0.5*h*k3(8);
lc1 = x14x1(9)+ 0.5*h*k3(9); dlc1 =x14x1(10)+ 0.5*h*k3(10);
theta2 = x14x1(11)+ 0.5*h*k3(11); dtheta2 =x14x1(12)+ 0.5*h*k3(12);
lc2 = x14x1(13)+ 0.5*h*k3(13); dlc2 =x14x1(14)+ 0.5*h*k3(14);
xob1 = [theta1,dtheta1,pm11,pm12,pm13];
[ypred1,~,~] = predict(gprMdl1,xob1);

xob2 = [lc1,dlc1,pm11,pm12,pm13];
[ypred2,~,~] = predict(gprMdl2,xob2);

xob3 = [theta2,dtheta2,pm21,pm22,pm23];
[ypred3,~,~] = predict(gprMdl3,xob3);

xob4 = [lc2,dlc2,pm21,pm22,pm23];
[ypred4,~,~] = predict(gprMdl4,xob4);
ypred4x1 = [ypred1;ypred2;ypred3;ypred4];

[M4x4,C4x1,G4x1,~] = funcMCGcal([theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);
dq4x1 = [dtheta1;dlc1;dtheta2;dlc2];
ddq4x1 = M4x4\(ypred4x1-C4x1*x14x1(8:2:14)-G4x1);
dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k4 = [dpm6x1;dq4x1(1);ddq4x1(1);dq4x1(2);ddq4x1(2);dq4x1(3);ddq4x1(3);dq4x1(4);ddq4x1(4);];

x_new = x14x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end