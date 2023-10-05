function [x_new] = funcRK4fullODELtermOnly1_m(x10x1,u6x1,h,par_set)
%% 
ParVal    ={20993; 19867.7;23824.3; 23670.9;... % Par initial values
    2093.65; 1951.15; 2309.74; 2227.23;...
    99719.5; 94343.2 ; 93506.2; 73528.2;...
    1;1;1;1};
kk1 = ParVal{1};kk2 = ParVal{2};kk3 = ParVal{3};kk4 = ParVal{4};
ko1 = ParVal{5};ko2 = ParVal{6};ko3 = ParVal{7};ko4 = ParVal{8};
d1 =ParVal{9};d2= ParVal{10};d3= ParVal{11};d4= ParVal{12};

a1 = 1;a2 = 1 ;a3 = 1; a4 =1;
alpha = -0.9665; beta = 0.9698;

Kmat =diag([kk1,kk2,kk3,kk4]);
Dmat = diag([d1,d2,d3,d4]);
Amat = diag([a1,a2,a3,a4]);

pd11 = u6x1(1);
pd12 = u6x1(2);
pd13 = u6x1(3);
pd21 = u6x1(4);
pd22 = u6x1(5);
pd23 = u6x1(6);
%% estimating k1
pm11 = x10x1(1); pm12 = x10x1(2); pm13 = x10x1(3);
pm21 = x10x1(4); pm22 = x10x1(5); pm23 = x10x1(6);
l11 = x10x1(7); l12 = x10x1(8);l21 = x10x1(9); l22 = x10x1(10); 



u_pm_psi(1,1) = pm12 + pm13;
u_pm_psi(2,1) = pm11 + pm13;
u_pm_psi(3,1) = pm22 + pm23;
u_pm_psi(4,1) = pm21 + pm23;
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;



dq4x1 =Dmat\(Amat*u_pm_tf-Kmat*[l11;l12;l21;l22;]+[ko1;ko2;ko3;ko4;]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k1 = [dpm6x1;dq4x1];


%% estimating k2
pm11 = x10x1(1) + 0.5*h*k1(1); pm12 = x10x1(2)+ 0.5*h*k1(2); 
pm13 = x10x1(3)+ 0.5*h*k1(3); pm21 = x10x1(4)+ 0.5*h*k1(4);
pm22 = x10x1(5)+ 0.5*h*k1(5); pm23 = x10x1(6)+ 0.5*h*k1(6); 

l11 = x10x1(7)+ 0.5*h*k1(7);
l12 = x10x1(8)+ 0.5*h*k1(8); 
l21 = x10x1(9)+ 0.5*h*k1(9); 
l22 = x10x1(10)+ 0.5*h*k1(10); 



u_pm_psi(1,1) = pm12 + pm13;
u_pm_psi(2,1) = pm11 + pm13;
u_pm_psi(3,1) = pm22 + pm23;
u_pm_psi(4,1) = pm21 + pm23;
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;



dq4x1 =Dmat\(Amat*u_pm_tf-Kmat*[l11;l12;l21;l22;]+[ko1;ko2;ko3;ko4;]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k2 = [dpm6x1;dq4x1];

%% estimating k3
pm11 = x10x1(1) + 0.5*h*k2(1); pm12 = x10x1(2)+ 0.5*h*k2(2); 
pm13 = x10x1(3)+ 0.5*h*k2(3); pm21 = x10x1(4)+ 0.5*h*k2(4);
pm22 = x10x1(5)+ 0.5*h*k2(5); pm23 = x10x1(6)+ 0.5*h*k2(6); 

l11 = x10x1(7)+ 0.5*h*k2(7);
l12 = x10x1(8)+ 0.5*h*k2(8); 
l21 = x10x1(9)+ 0.5*h*k2(9); 
l22 = x10x1(10)+ 0.5*h*k2(10); 



u_pm_psi(1,1) = pm12 + pm13;
u_pm_psi(2,1) = pm11 + pm13;
u_pm_psi(3,1) = pm22 + pm23;
u_pm_psi(4,1) = pm21 + pm23;
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;




dq4x1 =Dmat\(Amat*u_pm_tf-Kmat*[l11;l12;l21;l22;]+[ko1;ko2;ko3;ko4;]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k3 = [dpm6x1;dq4x1];
%% estimating k4
pm11 = x10x1(1) + h*k3(1); pm12 = x10x1(2)+ h*k3(2); 
pm13 = x10x1(3)+ h*k3(3); pm21 = x10x1(4)+ h*k3(4);
pm22 = x10x1(5)+ h*k3(5); pm23 = x10x1(6)+ h*k3(6); 
l11 = x10x1(7)+ 0.5*h*k3(7);
l12 = x10x1(8)+ 0.5*h*k3(8); 
l21 = x10x1(9)+ 0.5*h*k3(9); 
l22 = x10x1(10)+ 0.5*h*k3(10); 



u_pm_psi(1,1) = pm12 + pm13;
u_pm_psi(2,1) = pm11 + pm13;
u_pm_psi(3,1) = pm22 + pm23;
u_pm_psi(4,1) = pm21 + pm23;
u_pm_pa = u_pm_psi * 6894.76;
u_pm_tf(1,1) = u_pm_pa(1) * par_set.fz_a0;
u_pm_tf(2,1) = u_pm_pa(2) * par_set.fz_a0;
u_pm_tf(3,1) = u_pm_pa(3) * par_set.fz_a0;
u_pm_tf(4,1) = u_pm_pa(4) * par_set.fz_a0;


dq4x1 =Dmat\(Amat*u_pm_tf-Kmat*[l11;l12;l21;l22;]+[ko1;ko2;ko3;ko4;]);

dpm6x1 = alpha*eye(6)*[pm11,pm12,pm13,pm21,pm22,pm23]'+ beta*[pd11,pd12,pd13,pd21,pd22,pd23]';

k4 = [dpm6x1;dq4x1];

x_new = x10x1 + h/6*(k1 + 2*k2 + 2*k3 +k4);
end