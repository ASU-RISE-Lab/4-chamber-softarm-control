function [output] = funcCompuFK_v1(stateVarArr,TiSymb)
n = 6; % DOF

% DH parameters symbols
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
% mocap xm pointing left hand, zm pointing up, z_offset = 0.55m
% x0 pointing left, z0 pointing down
% x1 same, z1 pointing out
%T x0 to xm = [-1 0 0 0.0068 
%               0 1 0 -0.2111
%               0 0 -1 0.5036 
%               0 0 0 1]
Tbase2mocap = [-1 0 0 0.0068; 
              0 1 0 -0.2111;
              0 0 -1 0.5086; 
              0 0 0 1];
Tbase2mocap = [1 0 0 0.0068; 
              0 1 0 -0.2111;
              0 0 -1 0.5086; 
              0 0 0 1];
a1 = 0.0;
for i  =  1:length(stateVarArr)
theta1 = stateVarArr(i,1);
dtheta1 = stateVarArr(i,2);
lc1 = stateVarArr(i,3);
dlc1 = stateVarArr(i,4);
theta2 = stateVarArr(i,5);
dtheta2 = stateVarArr(i,6);
lc2 = stateVarArr(i,7);
dlc2 = stateVarArr(i,8);

b_theta1 = lc1/(theta1)*tan(theta1/2);
b_theta2 = lc2/(theta2)*tan(theta2/2);

m_q=[b_theta1 theta1 b_theta1 b_theta2 theta2 b_theta2 ].';% 6x1
% endEffector1x(i) = double(subs(TiSymb{3}(1,4),q,m_q));
% endEffector1y(i) = double(subs(TiSymb{3}(2,4),q,m_q));
% endEffector1z(i) = double(subs(TiSymb{3}(3,4),q,m_q));
%
endEffector1x(i) = b_theta1*sin(theta1);
endEffector1y(i) = 0;
endEffector1z(i) = b_theta1 + b_theta1*cos(theta1);

% endEffector2x(i) = double(subs(TiSymb{3}(1,4),q,m_q));
% endEffector2y(i) = double(subs(TiSymb{3}(2,4),q,m_q));
% endEffector2z(i) = double(subs(TiSymb{3}(3,4),q,m_q));
endEffector2x(i) = sin(theta1)*(a1 + b_theta2)...
                    + b_theta2*cos(theta1 + theta2) + b_theta1*sin(theta1);
endEffector2y(i) = 0;
endEffector2z(i) = b_theta1 + cos(theta1)*(a1 + b_theta2) ...
                   + b_theta2*cos(theta1 + theta2) + b_theta1*cos(theta1);

xyz1 = [endEffector1x(i);endEffector1y(i);endEffector1z(i);1];
xyz2 = [endEffector2x(i);endEffector2y(i);endEffector2z(i);1];
fkxyz1(:,i) = Tbase2mocap*xyz1;
fkxyz2(:,i) = Tbase2mocap*xyz2;
i
end
output.baseFrameE1 = [endEffector1x,endEffector1y,endEffector1z];
output.baseFrameE2 = [endEffector2x,endEffector2y,endEffector2z];
output.camFrameE1 = fkxyz1(1:3,:)';
output.camFrameE2 = fkxyz2(1:3,:)';
end

