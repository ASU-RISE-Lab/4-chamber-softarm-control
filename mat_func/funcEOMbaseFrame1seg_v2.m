function par =funcEOMbaseFrame1seg_v2(par)
fprintf( 'EOM... \n' )
n = 6; % DOF

% DH parameters symbols
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
qd = sym('qd', [n 1], 'real');
Iyyi = sym('Iyy',[n 1],'real');
syms a1 g m1 m2 real
m = sym(zeros(n,1));
m(3) = 0.5*m1;
m(4) = 0.5*m2;
m(6) = 0.5*m2;
T0i = cell(n,1);
% mocap xm pointing left hand, zm pointing up, z_offset = 0.55m
% x0 pointing right hand, z0 pointing down
% x1 same, z1 pointing out
%T x0 to xm = [-1 0 0 0 
%               0 1 0 0
%               0 0 -1 0.55 
%               0 0 0 1]
DH = [0 -pi/2 q(1)  0;%0
      0  pi/2   0   q(2); 
      0     0 q(3)+a1  0;
      0 -pi/2 q(4)+a1  0;
      0  pi/2   0   q(5); 
      0     0 q(6)+a1  0;];
Iyyi(1) = 0; Iyyi(2) = 0; Iyyi(3) = m(3)*(q(3)+a1).^2;
Iyyi(4) = m(4)*(q(3)+q(4)+2*a1).^2; Iyyi(5) = 0; Iyyi(6) = m(6)*(q(6)+a1).^2;
T = eye(4);
for i = 1:n
    temp = compute_dh_matrix(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    T = T*temp;
    T0i{i} = T;
    zi_from_0{i} = T(1:3,3);
    pi_from_0{i} = T(1:3,4);
end
%% Jacobian
Jv = cell(n,1);
P = [];
for i=1:n    
P = T0i{i};
    x = P(1,4);
    y = P(2,4);
    z = P(3,4);
    for j=1:n
        Jv{i} = [Jv{i} [diff(x,q(j));diff(y,q(j));diff(z,q(j))]];
    end        
end
Jw=cell(n,1);
for i= 1:n
    jw=sym(zeros(3,n));
    for j_counter =1:i
        if DH(j_counter,4) == 0 %% Prismatic Joint
            jw(:,j_counter)=zeros(3,1);
        else %% Rotational Joint
            jw(:,j_counter)=zi_from_0{j_counter};
        end
    end
    Jw{i}=jw;
end
%% Protential energy and Kinetic 

PE = 0;
D = 0;


for i=1:n
    P = T0i{i};
    PE=PE+m(i)*g*P(3,4);
    D = D + (m(i)*Jv{i}'*Jv{i} + Jw{i}'*Iyyi(i)*Jw{i});
end
fprintf( 'J_v... \n' )
par.Ep = PE;
%% 
c = zeros(n,n,n,'sym');
for k = 1:n
    for i = 1:n
        for j =1:n
            c(i,j,k) = 0.5 * (diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
        end
    end
end

% The coriolis matrix
C = zeros(n,n,'sym');
for k = 1:n
    for j = 1:n
        temp = 0;
        for i = 1:n
            temp = temp + c(i,j,k)*qd(i);
        end
        C(j,k) = temp;
    end
end
G = zeros(n,1,'sym');
for k = 1:n
    G(k) = diff(PE,q(k));
end

par.B_rigid=D;
par.C_rigid=C;
par.G_rigid=G;

%% mapping
syms theta1 dtheta1 ddtheta1 theta1_t(t) lc1 dlc1 ddlc1 lc1_t(t)
syms theta2 dtheta2 ddtheta2 theta2_t(t) lc2 dlc2 ddlc2 lc2_t(t)
b_theta1 = lc1/(theta1)*tan(theta1/2);
b_theta2 = lc2/(theta2)*tan(theta2/2);

m_q=[b_theta1 theta1 b_theta1 b_theta2 theta2 b_theta2 ].';% 6x1
J_f=[diff(m_q,theta1),diff(m_q,lc1),diff(m_q,theta2),diff(m_q,lc2)];%6x4


temp_dJ_f=diff(subs(J_f,[theta1,lc1,theta2,lc2],...
    [theta1_t(t),lc1_t(t),theta2_t(t),lc2_t(t)]),t);%6x4
dJ_fdt=subs(temp_dJ_f,[theta1_t(t),diff(theta1_t(t),t),lc1_t(t),diff(lc1_t(t),t),theta2_t(t),diff(theta2_t(t),t),lc2_t(t),diff(lc2_t(t), t)]...
    ,[theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);% 6x4
par.J_xi2q=J_f;%6x4
temp_xi=m_q;%6x1
temp_dxi=J_f*[dtheta1 dlc1 dtheta2 dlc2].'; %6x4 * 4x1
temp_ddxi=dJ_fdt*[dtheta1 dlc1 dtheta2 dlc2].'+J_f*[ddtheta1 ddlc1 ddtheta2 ddlc2].';% 6x4 * 4x1 + 6x4 * 4x1
% %%
B_xi_q=subs(D,q,m_q);
% par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;

fprintf( 'cq... \n' )
C_q=J_f.'*subs(D,q,m_q)*dJ_fdt+J_f.'*subs(C,[q;qd],[temp_xi;temp_dxi])*J_f;
% %%
fprintf( 'g_q... \n' )
G_q=J_f.'*subs(G,q,m_q);

par.B_q=B_q;
par.C_q=C_q;
par.G_q=G_q;
par.Ti = T0i;
%% mapping for straight cases
b_theta1 = 0.5*lc1;
b_theta2 = lc2/(theta2)*tan(theta2/2);

m_q=[b_theta1 theta1 b_theta1 b_theta2 theta2 b_theta2 ].';% 6x1
J_f=[diff(m_q,theta1),diff(m_q,lc1),diff(m_q,theta2),diff(m_q,lc2)];%6x4


temp_dJ_f=diff(subs(J_f,[theta1,lc1,theta2,lc2],...
    [theta1_t(t),lc1_t(t),theta2_t(t),lc2_t(t)]),t);%6x4
dJ_fdt=subs(temp_dJ_f,[theta1_t(t),diff(theta1_t(t),t),lc1_t(t),diff(lc1_t(t),t),theta2_t(t),diff(theta2_t(t),t),lc2_t(t),diff(lc2_t(t), t)]...
    ,[theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);% 6x4
par.J_xi2q=J_f;%6x4
temp_xi=m_q;%6x1
temp_dxi=J_f*[dtheta1 dlc1 dtheta2 dlc2].'; %6x4 * 4x1
temp_ddxi=dJ_fdt*[dtheta1 dlc1 dtheta2 dlc2].'+J_f*[ddtheta1 ddlc1 ddtheta2 ddlc2].';% 6x4 * 4x1 + 6x4 * 4x1
% %%
B_xi_q=subs(D,q,m_q);
% par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;

fprintf( 'cq... \n' )
C_q=J_f.'*subs(D,q,m_q)*dJ_fdt+J_f.'*subs(C,[q;qd],[temp_xi;temp_dxi])*J_f;
% %%
fprintf( 'g_q... \n' )
G_q=J_f.'*subs(G,q,m_q);

par.B_q_0x=subs(B_q,[theta1],[0]);
par.C_q_0x=subs(C_q,[theta1],[0]);
par.G_q_0x=subs(G_q,[theta1],[0]);
%%%%%%%
b_theta1 = lc1/(theta1)*tan(theta1/2);
b_theta2 = 0.5*lc2;

m_q=[b_theta1 theta1 b_theta1 b_theta2 theta2 b_theta2 ].';% 6x1
J_f=[diff(m_q,theta1),diff(m_q,lc1),diff(m_q,theta2),diff(m_q,lc2)];%6x4


temp_dJ_f=diff(subs(J_f,[theta1,lc1,theta2,lc2],...
    [theta1_t(t),lc1_t(t),theta2_t(t),lc2_t(t)]),t);%6x4
dJ_fdt=subs(temp_dJ_f,[theta1_t(t),diff(theta1_t(t),t),lc1_t(t),diff(lc1_t(t),t),theta2_t(t),diff(theta2_t(t),t),lc2_t(t),diff(lc2_t(t), t)]...
    ,[theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);% 6x4
par.J_xi2q=J_f;%6x4
temp_xi=m_q;%6x1
temp_dxi=J_f*[dtheta1 dlc1 dtheta2 dlc2].'; %6x4 * 4x1
temp_ddxi=dJ_fdt*[dtheta1 dlc1 dtheta2 dlc2].'+J_f*[ddtheta1 ddlc1 ddtheta2 ddlc2].';% 6x4 * 4x1 + 6x4 * 4x1
% %%
B_xi_q=subs(D,q,m_q);
% par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;

fprintf( 'cq... \n' )
C_q=J_f.'*subs(D,q,m_q)*dJ_fdt+J_f.'*subs(C,[q;qd],[temp_xi;temp_dxi])*J_f;
% %%
fprintf( 'g_q... \n' )
G_q=J_f.'*subs(G,q,m_q);

par.B_q_x0=subs(B_q,theta2,0);
par.C_q_x0=subs(C_q,theta2,0);
par.G_q_x0=subs(G_q,theta2,0);
%%%%%%%%%%%
b_theta1 = 0.5*lc1;
b_theta2 = 0.5*lc2;

m_q=[b_theta1 theta1 b_theta1 b_theta2 theta2 b_theta2 ].';% 6x1
J_f=[diff(m_q,theta1),diff(m_q,lc1),diff(m_q,theta2),diff(m_q,lc2)];%6x4


temp_dJ_f=diff(subs(J_f,[theta1,lc1,theta2,lc2],...
    [theta1_t(t),lc1_t(t),theta2_t(t),lc2_t(t)]),t);%6x4
dJ_fdt=subs(temp_dJ_f,[theta1_t(t),diff(theta1_t(t),t),lc1_t(t),diff(lc1_t(t),t),theta2_t(t),diff(theta2_t(t),t),lc2_t(t),diff(lc2_t(t), t)]...
    ,[theta1,dtheta1,lc1,dlc1,theta2,dtheta2,lc2,dlc2]);% 6x4
par.J_xi2q=J_f;%6x4
temp_xi=m_q;%6x1
temp_dxi=J_f*[dtheta1 dlc1 dtheta2 dlc2].'; %6x4 * 4x1
temp_ddxi=dJ_fdt*[dtheta1 dlc1 dtheta2 dlc2].'+J_f*[ddtheta1 ddlc1 ddtheta2 ddlc2].';% 6x4 * 4x1 + 6x4 * 4x1
% %%
B_xi_q=subs(D,q,m_q);
% par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;

fprintf( 'cq... \n' )
C_q=J_f.'*subs(D,q,m_q)*dJ_fdt+J_f.'*subs(C,[q;qd],[temp_xi;temp_dxi])*J_f;
% %%
fprintf( 'g_q... \n' )
G_q=J_f.'*subs(G,q,m_q);

par.B_q_00=subs(B_q,[theta1,theta2],[0,0]);
par.C_q_00=subs(C_q,[theta1,theta2],[0,0]);
par.G_q_00=subs(G_q,[theta1,theta2],[0,0]);
% par.Ti = T0i;
%% Actuation mapping
fprintf('EOM Done\n')
end