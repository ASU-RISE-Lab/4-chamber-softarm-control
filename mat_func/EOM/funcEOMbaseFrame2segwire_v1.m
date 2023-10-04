function par =funcEOMbaseFrame2segwire_v1(par)
fprintf( 'EOM... \n' )
% par=[];
%% Transformations
par.n= 8;%DOF
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
% pi =sym('pi');
syms m0 g a1 lc1
xi = sym('xi', [par.n 1]);
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
dxi = sym('dxi', [par.n 1]);
Izzi = sym('Izz',[par.n 1]);



rigid_r=sym(zeros(1,par.n));
rigid_alpha=sym(zeros(1,par.n));%alpha
rigid_d=sym(zeros(1,par.n));    
rigid_theta=sym(zeros(1,par.n));
rigid_m = sym(zeros(1,par.n));
%%% DH talbe %%% z0+ points inward, x0+ points left
%%% Link    theta   d     alpha   a   
%%% 1       xi(1)   0     -pi/2   0   
%%% 2       0      xi(2)   0   
%%% 3       0      xi(3)   pi/2   0
%%% 4       xi(4)   0     0      0
%%% 5       xi(5)   0     -pi/2   0   
%%% 6       0      xi(6)   0   
%%% 7       0      xi(7)   pi/2   0
%%% 8       xi(8)        0      0

rigid_theta(1)= xi(1);
rigid_theta(4)= xi(4);
rigid_theta(5)= xi(5);
rigid_theta(8)= xi(8);

rigid_alpha(1)= pi/2;
rigid_alpha(3)= -pi/2;
rigid_alpha(5)= pi/2;
rigid_alpha(7)= -pi/2;

rigid_d(2)= xi(2);
rigid_d(3)= xi(3);
rigid_d(6)= xi(6);
rigid_d(7)= xi(7)+a1;









rigid_m(2)= m0;
rigid_m(6)= m0;
Izzi(2) = rigid_m(2)*xi(2)*xi(2);
Izzi(6) = rigid_m(6)*xi(6)*xi(6);
% n=length(q);% DOF
% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic transform matrix
T_old_to_i = cell(par.n,1);% z0 z_end_effector
Ti = cell(par.n+1,1);
p_i = cell(par.n+1,1);
z_i = cell(par.n+1,1);
% Ti(1) = {[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1]};
for i =1:length(rigid_m)
    T_old_to_i{i} = [cos(rigid_theta(i)), -sin(rigid_theta(i)) * cos(rigid_alpha(i)), sin(rigid_theta(i)) * sin(rigid_alpha(i)), rigid_r(i) * cos(rigid_theta(i));...
                     sin(rigid_theta(i)),  cos(rigid_theta(i)) * cos(rigid_alpha(i)), -cos(rigid_theta(i)) * sin(rigid_alpha(i)), rigid_r(i) * sin(rigid_theta(i));...
                     0, sin(rigid_alpha(i)), cos(rigid_alpha(i)),rigid_d(i);...
                     0, 0, 0, 1   ];
end
Ti{1} = eye(4);
p_i{1}=Ti{1}(1:3,4);
z_i{1}=Ti{1}(1:3,3);
for i = 2:length(rigid_m)+1
    Ti{i} =  Ti{i-1} * T_old_to_i{i-1};
    p_i{i}=Ti{i}(1:3,4);
    z_i{i}=Ti{i}(1:3,3);
end
par.Ti=Ti;
fprintf( 'Ep.. \n' )
par.rigid_2_htm = Ti{par.n/2};
par.rigid_3_htm = Ti{end};
% return
%% Protential energy
E_p=0;
for link_i=2:length(rigid_r)
%     Epi = rigid_m(link_i)*[0;0;g].'*p_i{link_i+1}
% p_i{link_i+1}
    E_p=E_p+rigid_m(link_i)*[0;-g;0].'*p_i{link_i+1};
end
fprintf( 'J_v... \n' )
par.Ep = E_p;
% simplify(E_p)

% return
%% Linear Velocity
% J_v=cell(par.n,1);
% for link_i = 1:par.n
%     j_v=sym(zeros(3,par.n));
%     if rigid_theta(1) == 0
%         j_v(:,1)=[0;0;0];
%     else
%         j_v(:,1)=cross([0;0;1],(p_i{link_i}- 0));
%     end
%     for j_counter =2:link_i
%         if rigid_theta(j_counter) == 0 %% Prismatic Joint
%             j_v(:,j_counter)=z_i{j_counter-1};
%         else %% Rotational Joint
%             j_v(:,j_counter)=cross(z_i{j_counter-1},(p_i{link_i}-p_i{j_counter-1}));
%         end
%     end
%     J_v{link_i}=j_v;
% end
J_v=cell(par.n,1);
for link_i = 1:par.n
    j_v=sym(zeros(3,par.n));
    for j_counter =1:link_i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_v(:,j_counter)=z_i{j_counter};
%             z_ii = z_i{j_counter};
        else %% Rotational Joint
            j_v(:,j_counter)=cross(z_i{j_counter},(p_i{link_i+1}-p_i{j_counter}));
%             z_ii = cross(z_i{j_counter},(p_i{link_i+1}-p_i{j_counter}))
        end
    end
    J_v{link_i}=j_v;
end
fprintf( 'J_w... \n' )
% return
%% Angular Velocity
% J_w=cell(par.n,1);
% for link_i= 1:par.n
%     j_w=sym(zeros(3,par.n));
%     if rigid_theta(1) == 0
%         j_w(:,1)=zeros(3,1);
%     else
%         j_w(:,1)=[0;0;1];
%     end
%     for j_counter =2:link_i
%         if rigid_theta(j_counter) == 0 %% Prismatic Joint
%             j_w(:,j_counter)=zeros(3,1);
%         else %% Rotational Joint
%             j_w(:,j_counter)=z_i{j_counter-1};
%         end
%     end
%     J_w{link_i}=j_w;
% end

J_w=cell(par.n,1);
for link_i= 1:par.n
    j_w=sym(zeros(3,par.n));
    for j_counter =1:link_i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_w(:,j_counter)=zeros(3,1);
        else %% Rotational Joint
            j_w(:,j_counter)=z_i{j_counter};
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
I=cell(par.n,1);
% I = [0 0 0 0 I_u 0 0 0 0 0];
for link_i =1:par.n
    if rigid_m(link_i) == 0 % no mass
        I{link_i}=zeros(3,3);
    else
%         I{link_i}=[Ixx Ixy Ixz;
%            Ixy Iyy Iyz;
%            Ixz Iyz Izz];
        I{link_i}= Izzi(link_i);

    end
end
D = (rigid_m(1)*J_v{1}.'*J_v{1} + J_w{1}.'*I{1}*J_w{1});

for d_counter = 2:par.n
%     Di_v = rigid_m(d_counter)*J_v{d_counter}.'*J_v{d_counter}
%     Di_w = J_w{d_counter}.'*I{d_counter}*J_w{d_counter}
    D = D + (rigid_m(d_counter)*J_v{d_counter}.'*J_v{d_counter} + J_w{d_counter}.'*I{d_counter}*J_w{d_counter});
%     d_counter
end
% D=simplify(D);
% v_dq=[vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1) vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1)];
% E_k=simplify(1/2*dxi.'*D*dxi);
% subs(D,xi,[lc1,0,lc1,lc2,0,lc2])
% fprintf('Done')
% return
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
% return
%% mapping

syms l11 dl11 ddl11 l11_t(t) l12 dl12 ddl12 l12_t(t) rlc1
syms l21 dl21 ddl21 l21_t(t) l22 dl22 ddl22 l22_t(t) rlc2
theta1 = (l11 - l12)/rlc1;
theta2 = (l21 -l22)/rlc2;
lc1 = (l11 + l12)/2;
lc2 = (l11 + l12)/2;
b_theta1 = lc1/(theta1)*sin(theta1/2);
b_theta2 = lc2/(theta2)*sin(theta2/2);
xyz3x1 = Ti{end}(1:3,4);
for i =1:par.n
par.J_xyz2xi(:,i) = [diff(xyz3x1,xi(i))];
end
m_q=[0.5*theta1 b_theta1 b_theta1 0.5*theta1 0.5*theta2 b_theta2 b_theta2 0.5*theta2].';% 8x1

J_f=[diff(m_q,l11),diff(m_q,l12),diff(m_q,l21),diff(m_q,l22)];%8x4

temp.dJ_f=diff(subs(J_f,[l11,l12,l21,l22],[l11_t(t),l12_t(t),l21_t(t),l22_t(t),]),t);%8x4
dJ_fdt=subs(temp.dJ_f,[l11_t(t),diff(l11_t(t),t),l12_t(t),diff(l12_t(t),t),...
    l21_t(t),diff(l21_t(t),t),l22_t(t),diff(l22_t(t),t)]...
    ,[l11,dl11,l12,dl12,l21,dl21,l22,dl22]);% 8x4
par.J_xi2q=J_f;%8x4
temp.xi=m_q;%8x1
temp.dxi=J_f*[dl11 dl12 dl21 dl22].'; %8x4 * 4x1
temp.ddxi=dJ_fdt*[dl11 dl12 dl21 dl22].'+J_f*[ddl11 ddl12 ddl21 ddl22].';% 8x4 * 4x1 + 8x4 * 4x1
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
par.J_xyz2q=subs(par.J_xyz2xi*par.J_xi2q,[xi],[m_q]);

% f_q=J_f.'*subs(par.f_xi,xi,f);
% 
% %%
% % par={};
par.B_q=B_q;
par.C_q=C_q;
par.G_q=G_q;

%% Actuation mapping
fprintf('EOM Done\n')
end