function [output] = funcCompuFK2seg_v1(stateVarArr)
% use funcEOMBaseFrame1seg_v4
a1 = 0.05;
Tbase2mocap = [-1 0 0 0.0068; 
                0 0 -1 -0.2111;
                0 1 0  0.5086; 
                0 0 0 1];

for i  =  1:length(stateVarArr)
theta1 = stateVarArr(i,1);
lc1 = stateVarArr(i,3);
theta2 = stateVarArr(i,5);
lc2 = stateVarArr(i,7);

b_theta1 = lc1/(theta1)*sin(theta1/2);
b_theta2 = lc2/(theta2)*sin(theta2/2);

m_q=[0.5*theta1 b_theta1 b_theta1 0.5*theta1 0.5*theta2 b_theta2 b_theta2 0.5*theta2].';

endEffector1x(i) = (2*b_theta1)*sin(theta1/2);
endEffector1y(i) = -(2*b_theta1)*cos(theta1/2); 
endEffector1z(i) = 0;

endEffector2x(i) = (2*b_theta1)*sin(theta1/2) + (2*b_theta2 + a1)*sin(theta1 + theta2/2);
endEffector2y(i) = -(2*b_theta1)*cos(theta1/2) - (2*b_theta2 + a1)*cos(theta1 + theta2/2);
endEffector2z(i) = 0;


xyz1 = [endEffector1x(i);endEffector1y(i);endEffector1z(i);1];
xyz2 = [endEffector2x(i);endEffector2y(i);endEffector2z(i);1];
fkxyz1(:,i) = Tbase2mocap*xyz1;
fkxyz2(:,i) = Tbase2mocap*xyz2;
end
output.baseFrameE1 = [endEffector1x,endEffector1y,endEffector1z];
output.baseFrameE2 = [endEffector2x,endEffector2y,endEffector2z];
output.camFrameE1 = fkxyz1(1:3,:)';
output.camFrameE2 = fkxyz2(1:3,:)';
end

