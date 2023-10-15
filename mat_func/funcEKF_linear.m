function [Ak,F,Ak10,Ak20,Ak120] =funcEKF_linear()
fprintf( 'EKF... \n' )
sympref('FloatingPointOutput',true)
syms kk1 kk2 kk3 kk4 d1 d2 d3 d4
syms pd11 pd12 pd21 pd22
syms theta1 lc1 theta2 lc2 pm11 pm21 pm12 pm22
syms dT
syms alphap betap 
syms k1a2 k1a1 k1a0 k2a2 k2a1 k2a0 k3a2 k3a1 k3a0 k4a2 k4a1 k4a0
syms d1a2 d1a1 d1a0 d2a2 d2a1 d2a0 d3a2 d3a1 d3a0 d4a2 d4a1 d4a0
q8x1 = [theta1 lc1 theta2 lc2 pm11 pm12 pm21 pm22].';
pd4x1 = [pd11 pd12 pd21 pd22].';
u4x1(1,1) = -(pm11 - pm12);
u4x1(2,1) = (pm11 + pm12);
u4x1(3,1) = -(pm21 - pm22);
u4x1(4,1) = (pm21 + pm22);

kk1 = k1a2*u4x1(1)^2 + k1a1*sqrt(u4x1(1)^2)+k1a0;
kk2 = k2a2*u4x1(2)^2 + k2a1*u4x1(2)+k2a0;
kk3 = k3a2*u4x1(3)^2 + k3a1*sqrt(u4x1(3)^2)+k3a0;
kk4 = k4a2*u4x1(4)^2 + k4a1*u4x1(4)+k4a0;

d1 = d1a2*u4x1(1)^2 + d1a1*sqrt(u4x1(1)^2)+d1a0;
d2 = d2a2*u4x1(2)^2 + d2a1*u4x1(2)+d2a0;
d3 = d3a2*u4x1(3)^2 + d3a1*sqrt(u4x1(3)^2)+d3a0;
d4 = d4a2*u4x1(4)^2 + d4a1*u4x1(4)+d4a0;

% d1= 0.9725*u4x1(1)^2- 11.2*sqrt(u4x1(1)^2)+38.471;
% d2= -0.9725*u4x1(2)^2+ 30.23*u4x1(2)+435.471;
% d3=  0.1125*u4x1(3)^2- 1.2*sqrt(u4x1(3)^2)+14.471;
% d4= 4.34*u4x1(4)^2 - 155.21*u4x1(4)+2146;
Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);
% dqidt = Aq*q + B*upd
Pmat = [-1 1 0 0;1 1 0 0;0 0 -1 1;0 0 1 1];
Aq8x8 = [-Dmat\Kmat, Pmat;
        zeros(4,4), alphap*eye(4)];
B8x4 = [zeros(4,4);betap*eye(4)];
F = q8x1 + (Aq8x8*q8x1 + B8x4*pd4x1)*dT;
for i =1:8
Ak(i,:) = diff(F,q8x1(i));
end

kk1 = k1a0;
kk2 = k2a2*u4x1(2)^2 + k2a1*u4x1(2)+k2a0;
kk3 = k3a2*u4x1(3)^2 + k3a1*sqrt(u4x1(3)^2)+k3a0;
kk4 = k4a2*u4x1(4)^2 + k4a1*u4x1(4)+k4a0;

d1 = d1a0;
d2 = d2a2*u4x1(2)^2 + d2a1*u4x1(2)+d2a0;
d3 = d3a2*u4x1(3)^2 + d3a1*sqrt(u4x1(3)^2)+d3a0;
d4 = d4a2*u4x1(4)^2 + d4a1*u4x1(4)+d4a0;

Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);
% dqidt = Aq*q + B*upd
Pmat = [-1 1 0 0;1 1 0 0;0 0 -1 1;0 0 1 1];
Aq8x8 = [-Dmat\Kmat, Pmat;
        zeros(4,4), alphap*eye(4)];
B8x4 = [zeros(4,4);betap*eye(4)];
F = q8x1 + (Aq8x8*q8x1 + B8x4*pd4x1)*dT;
for i =1:8
Ak10(i,:) = diff(F,q8x1(i));
end

kk1 = k1a2*u4x1(1)^2 + k1a1*sqrt(u4x1(1)^2)+k1a0;
kk2 = k2a2*u4x1(2)^2 + k2a1*u4x1(2)+k2a0;
kk3 = k3a0;
kk4 = k4a2*u4x1(4)^2 + k4a1*u4x1(4)+k4a0;

d1 = d1a2*u4x1(1)^2 + d1a1*sqrt(u4x1(1)^2)+d1a0;
d2 = d2a2*u4x1(2)^2 + d2a1*u4x1(2)+d2a0;
d3 = d3a0;
d4 = d4a2*u4x1(4)^2 + d4a1*u4x1(4)+d4a0;

% d1= 0.9725*u4x1(1)^2- 11.2*sqrt(u4x1(1)^2)+38.471;
% d2= -0.9725*u4x1(2)^2+ 30.23*u4x1(2)+435.471;
% d3=  0.1125*u4x1(3)^2- 1.2*sqrt(u4x1(3)^2)+14.471;
% d4= 4.34*u4x1(4)^2 - 155.21*u4x1(4)+2146;
Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);
% dqidt = Aq*q + B*upd
Pmat = [-1 1 0 0;1 1 0 0;0 0 -1 1;0 0 1 1];
Aq8x8 = [-Dmat\Kmat, Pmat;
        zeros(4,4), alphap*eye(4)];
B8x4 = [zeros(4,4);betap*eye(4)];
F = q8x1 + (Aq8x8*q8x1 + B8x4*pd4x1)*dT;
for i =1:8
Ak20(i,:) = diff(F,q8x1(i));
end

kk1 = k1a0;
kk2 = k2a2*u4x1(2)^2 + k2a1*u4x1(2)+k2a0;
kk3 = k3a0;
kk4 = k4a2*u4x1(4)^2 + k4a1*u4x1(4)+k4a0;

d1 = d1a0;
d2 = d2a2*u4x1(2)^2 + d2a1*u4x1(2)+d2a0;
d3 = d3a0;
d4 = d4a2*u4x1(4)^2 + d4a1*u4x1(4)+d4a0;

% d1= 0.9725*u4x1(1)^2- 11.2*sqrt(u4x1(1)^2)+38.471;
% d2= -0.9725*u4x1(2)^2+ 30.23*u4x1(2)+435.471;
% d3=  0.1125*u4x1(3)^2- 1.2*sqrt(u4x1(3)^2)+14.471;
% d4= 4.34*u4x1(4)^2 - 155.21*u4x1(4)+2146;
Kmat = diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);
% dqidt = Aq*q + B*upd
Pmat = [-1 1 0 0;1 1 0 0;0 0 -1 1;0 0 1 1];
Aq8x8 = [-Dmat\Kmat, Pmat;
        zeros(4,4), alphap*eye(4)];
B8x4 = [zeros(4,4);betap*eye(4)];
F = q8x1 + (Aq8x8*q8x1 + B8x4*pd4x1)*dT;
for i =1:8
Ak120(i,:) = diff(F,q8x1(i));
end
end
