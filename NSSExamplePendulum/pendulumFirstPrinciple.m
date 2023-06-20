function dxdt = pendulumFirstPrinciple(x,tau)
%% simple pendulum with mass-less rod and air drag
% x(1) is theta [-pi/2 pi/2], x(2) is omega, tau is torque.
g = 9.8; 
m = 0.25;
l = 2;
r = 1; % damping constant 
dxdt = [x(2); tau/m/l/l-r*x(2)-g/l*sin(x(1))];
