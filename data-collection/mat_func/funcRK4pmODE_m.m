function [x_new] = funcRK4pmODE_m(x6x1,alpha,beta,u6x1,h)
pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
%% estimating k1
pm11 = x6x1(1); pm12 = x6x1(2); pm13 = x6x1(3);
pm21 = x6x1(4); pm22 = x6x1(5); pm23 = x6x1(6);
condition =0;
dx = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
k1 = dx;

%% estimating k2
pm11 = x6x1(1) + 0.5*h*k1(1); pm12 = x6x1(2)+ 0.5*h*k1(2); 
pm13 = x6x1(3)+ 0.5*h*k1(3); pm21 = x6x1(4)+ 0.5*h*k1(4);
pm22 = x6x1(5)+ 0.5*h*k1(5); pm23 = x6x1(6)+ 0.5*h*k1(6); 

dx = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
k2 = dx;

%% estimating k3
pm11 = x6x1(1) + 0.5*h*k2(1); pm12 = x6x1(2)+ 0.5*h*k2(2); 
pm13 = x6x1(3)+ 0.5*h*k2(3); pm21 = x6x1(4)+ 0.5*h*k2(4);
pm22 = x6x1(5)+ 0.5*h*k2(5); pm23 = x6x1(6)+ 0.5*h*k2(6); 

dx = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
k3 = dx;
%% estimating k4
pm11 = x6x1(1) + h*k3(1); pm12 = x6x1(2)+ h*k3(2); 
pm13 = x6x1(3)+ h*k3(3); pm21 = x6x1(4)+ h*k3(4);
pm22 = x6x1(5)+ h*k3(5); pm23 = x6x1(6)+ h*k3(6); 
dx = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';
k4 = dx;

x_new = x6x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end