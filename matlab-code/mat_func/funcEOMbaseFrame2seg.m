function par =funcEOMbaseFrame2seg(par)
fprintf( 'EOM... \n' )
% par=[];
%% Transformations
par.n= 8;%DOF
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
pi =sym('pi');
syms m0 g h
xi = sym('xi', [par.n 1]);
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
dxi = sym('dxi', [par.n 1]);

rigid_a=sym(zeros(1,par.n));
rigid_alpha=sym(zeros(1,par.n));%alpha
rigid_d=sym(zeros(1,par.n));    
rigid_theta=sym(zeros(1,par.n));
rigid_m = sym(zeros(1,par.n));
%%% DH talbe %%%
%%% Link    theta   d     alpha   a   
%%% 1       0       xi(1)  -pi/2   0   
%%% 2       xi(2)   xi(3)  pi/2   
%%% 3       0       0.02199 0   0
%%% 4       0       xi(4)  -pi/2   0   
%%% 5       xi(5)   xi(6)  pi/2
rigid_alpha(1)= -pi/2;
rigid_alpha(3)= pi/2;
rigid_alpha(4)= -pi/2;
rigid_alpha(5)= -pi/2;
rigid_alpha(7)= pi/2;
rigid_alpha(8)= -pi/2;

rigid_a(5)= h;

rigid_d(2)= xi(2);
rigid_d(3)= xi(3);
rigid_d(6)= xi(6);
rigid_d(7)= xi(7);


rigid_theta(1)= xi(1);
rigid_theta(4)= xi(4);
rigid_theta(5)= xi(5);
rigid_theta(8)= xi(8);

rigid_m(1)= m0/2;
rigid_m(4)= m0/2;
rigid_m(5)= m0/2;
rigid_m(8)= m0/2;
% n=length(q);% DOF
% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic transform matrix
T_old_to_i = cell(par.n,1);% z0 z_end_effector
Ti = T_old_to_i;
% Ti(1) = {[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1]};
for i =1:length(rigid_m)
    T_old_to_i{i} = [cos(rigid_theta(i)), -sin(rigid_theta(i)), 0, rigid_alpha(i);...
                     sin(rigid_theta(i)) * cos(rigid_alpha(i)), cos(rigid_theta(i)) * cos(rigid_alpha(i)), -sin(rigid_alpha(i)), -sin(rigid_alpha(i))*rigid_d(i);...
                     sin(rigid_theta(i)) * sin(rigid_alpha(i)), cos(rigid_theta(i)) * sin(rigid_alpha(i)), cos(rigid_alpha(i)),cos(rigid_alpha(i))*rigid_d(i);...
                     0, 0, 0, 1   ];
end
Ti{1} = T_old_to_i{1};
p_i{1}=Ti{1}(1:3,4);
z_i{1}=Ti{1}(1:3,3);
for i = 2:length(rigid_m)
    Ti{i} =  Ti{i-1} * T_old_to_i{i};
    p_i{i}=Ti{i}(1:3,4);
    z_i{i}=Ti{i}(1:3,3);
end
par.Ti=Ti;
fprintf( 'P.. \n' )
%% Protential energy
E_p=0;
for link_i=1:length(rigid_a)
    E_p=E_p+rigid_m(link_i)*g*p_i{link_i}(3);
end
fprintf( 'J_v... \n' )
%% Linear Velocity
J_v=cell(par.n,1);
for link_i = 1:par.n
    j_v=sym(zeros(3,par.n));
    if rigid_theta(1) == 0
        j_v(:,1)=[0;0;0];
    else
        j_v(:,1)=cross([0;0;1],(p_i{link_i}- 0));
    end
    for j_counter =2:link_i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_v(:,j_counter)=z_i{j_counter-1};
        else %% Rotational Joint
            j_v(:,j_counter)=cross(z_i{j_counter-1},(p_i{link_i}-p_i{j_counter-1}));
        end
    end
    J_v{link_i}=j_v;
end
% par.Jv=J_v;
fprintf( 'J_w... \n' )
%% Angular Velocity
J_w=cell(par.n,1);
for link_i= 1:par.n
    j_w=sym(zeros(3,par.n));
    if rigid_theta(1) == 0
        j_w(:,1)=zeros(3,1);
    else
        j_w(:,1)=[0;0;1];
    end
    for j_counter =2:link_i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_w(:,j_counter)=zeros(3,1);
        else %% Rotational Joint
            j_w(:,j_counter)=z_i{j_counter-1};
        end
    end
    J_w{link_i}=j_w;
end
% par.Jw=J_w;
%% Unite Jacobian
par.J_xyz=cell(par.n,1);
for i =1:length(J_w)
    par.J_xyz{i}=[J_v{i};J_w{i}];
    
end
par.sym_J_xyz2xi=par.J_xyz{end};    
fprintf( 'D... \n' )
%% Inerial and Kinetic energy
syms Ixx Iyy Izz Ixy Ixz Iyz
I=cell(par.n,1);
% I = [0 0 0 0 I_u 0 0 0 0 0];
for link_i =1:par.n
    if rigid_m(link_i) == 0 %%%%%%par.n/2
        I{link_i}=zeros(3,3);
    else
                I{link_i}=[Ixx Ixy Ixz;
                   Ixy Iyy Iyz;
                   Ixz Iyz Izz];
    end
end

D = (rigid_m(1)*J_v{1}.'*J_v{1} + J_w{1}.'*I{1}*J_w{1});

for d_counter = 2:par.n
    D = D + (rigid_m(d_counter)*J_v{d_counter}.'*J_v{d_counter} + J_w{d_counter}.'*I{d_counter}*J_w{d_counter});
    d_counter
end
% D=simplify(D);
% v_dq=[vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1) vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1)];
% E_k=simplify(1/2*dxi.'*D*dxi);
fprintf( 'C... \n' )
%% Coriolis
Cs = sym(zeros(par.n,par.n,par.n));
for i1 = 1:par.n
    for j1 = 1:par.n
        for k1 = 1:par.n
              diff1 = 1/2*(diff(D(k1,j1),xi(i1)));
            diff2 = 1/2*(diff(D(k1,i1),xi(j1)));
            diff3 = 1/2*(diff(D(i1,j1),xi(k1)));
            Cs(i1,j1,k1) = (diff1+diff2-diff3)*dxi(i1);
        end
    end
end
cor = sym(zeros(par.n,par.n));
fprintf( 'cor... \n' )
for k1 = 1:par.n
    for j1 = 1:par.n 
        for i1 = 1:par.n
            cor(k1,j1)=cor(k1,j1)+Cs(i1, k1 , j1);
        end
    end
end
Phi = sym(zeros(par.n,1));

for i1 = 1:par.n
    Phi(i1) = diff(E_p,xi(i1));
%     Phi = Phi;2345
end
%% EOM rigid
% syms f_p1 f_p2 f_p3
% ddxi = sym('ddxi', [par.n 1], 'rational'); % "q double dot" - the second derivative of the q's in time (joint accelerations)
% eom_lhs = D*ddxi+cor*dxi+Phi;

par.B_rigid=D;
par.C_rigid=cor;
par.G_rigid=Phi;
% for i =1:3
%     T_p{i}=Ti{end}*[eye(3),par.r_p{i};0 0 0 1];
%     r_p_base{i}=T_p{i}(1:3,4);
% end
% % par.sym_wrench=[f_p1*T_p{1}(1:3,3);cross(r_p_base{1},f_p1*T_p{1}(1:3,3))]+...
% %     [f_p2*T_p{2}(1:3,3);cross(r_p_base{2},f_p2*T_p{2}(1:3,3))]+...
% %     [f_p3*T_p{3}(1:3,3);cross(r_p_base{3},f_p3*T_p{3}(1:3,3))];
% % 
% % eom_rhs=par.J_xyz{end}.'*par.sym_wrench;
% % par.f_xi=eom_rhs;
% % % par.sym_wrench=[f_x f_y f_z tau_x tau_y tau_z].';
%% mapping

syms theta1 dtheta1 ddtheta1 theta1_t(t) lc1 dlc1 ddlc1 lc1_t(t)
syms theta2 dtheta2 ddtheta2 theta2_t(t) lc2 dlc2 ddlc2 lc2_t(t)
b_theta1 = lc1/(theta1)*sin(theta1/2);
b_theta2 = lc2/(theta2)*sin(theta2/2);

m_q=[theta1/2 b_theta1 b_theta1 theta1/2 theta2/2 b_theta2 b_theta2 theta2/2].';% 8x1
J_f=[diff(m_q,theta1),diff(m_q,lc1),diff(m_q,theta2),diff(m_q,lc2)];%8x4

temp.dJ_f=diff(subs(J_f,[theta1,lc1,theta2,lc2],[theta1_t(t),lc1_t(t),theta2_t(t),lc2_t(t)]),t);%8x4
dJ_fdt=subs(temp.dJ_f,[theta1_t(t),diff(theta1_t(t),t),lc1_t(t),diff(lc1_t(t),t),theta2_t(t),diff(theta2_t(t),t),lc2_t(t),diff(lc2_t(t), t)]...
    ,[theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);% 8x4
par.J_xi2q=J_f;%8x4
temp.xi=m_q;%8x1
temp.dxi=J_f*[dtheta1 dlc1 dtheta2 dlc2].'; %8x4 * 2x1
temp.ddxi=dJ_fdt*[dtheta1 dlc1 dtheta2 dlc2].'+J_f*[ddtheta1 ddlc1 ddtheta2 ddlc2].';% 8x4 * 4x1 + 8x4 * 4x1
% %%
B_xi_q=subs(D,xi,m_q);
par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;
%%%
% M_xi_q=subs(M,xi,m_q);
% par.B_q_simplify=J_f.'*M_xi_q*J_f;
% %%
% temp_1=subs(cor,xi,f);
% temp_2=subs(temp_1,dxi,df);
% %%
% C_q=J_f.'*subs(D,xi,f)*J_ff+J_f.'*temp_2*J_f;
fprintf( 'cq... \n' )
C_q=J_f.'*subs(D,xi,m_q)*dJ_fdt+J_f.'*subs(cor,[xi;dxi],[temp.xi;temp.dxi])*J_f;
% %%
fprintf( 'g_q... \n' )
G_q=J_f.'*subs(Phi,xi,m_q);
% %%
% par.J_xyz2q=subs(par.J_xyz2xi*par.J_xi2q,[xi],[m_q]);

% f_q=J_f.'*subs(par.f_xi,xi,f);
% 
% %%
% % par={};
par.B_q=B_q;
par.C_q=C_q;
par.G_q=G_q;
% 
% % par.C_q_simplify=simplify(C_q);
% % par.G_q_simplify=simplify(G_q);
%% Actuation mapping
fprintf('EOM Done\n')
end