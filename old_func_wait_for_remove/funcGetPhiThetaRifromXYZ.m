function trainSet = funcGetPhiThetaRifromXYZ(trainSet,par_set)
%% Calculate Phi in camera frame then maps to robot base frame 0,2pi
%%%% Y-up to Z-up rotate -90deg w/ x axis
temp.Yup=trainSet.tip_exp;
temp.Zup=trainSet.tip_exp;
Rx=[1 0 0;
    0 cosd(-90) -sind(-90)
    0 sind(-90) cosd(-90)];

Rz=[cosd(-90) -sind(-90)  0;
    sind(-90) cosd(-90)   0;
    0                   0                   1];

for i =1:length(trainSet.pd_psi)
    temp.Zup(i,2:4)=((Rx)'*(temp.Yup(i,2:4)'))';
end

trainSet.tip_exp=temp.Zup;

% for i =1:length(trainSet.pd_psi)
%     temp.ZupR90(i,2:4)=((Rz)'*(temp.Zup(i,2:4)'))';
% end
% trainSet.tip_exp=temp.ZupR90;
temp.phi_vector=trainSet.tip_exp(:,2:4); %xyz in Camera frame
temp.angle_phi=[];
%%% Calculate phi anlge ranging [0,2pi] atan(y_top/x_top)
temp.tip_exp_baseFrame(:,1)=trainSet.tip_exp(:,1);
for i =1:length(trainSet.pd_psi)
    temp.angle_phi(i,1)=rad2deg(func_myatan(temp.phi_vector(i,2),temp.phi_vector(i,1)));
    temp.Rz=[cosd(temp.angle_phi(i,1)) -sind(temp.angle_phi(i,1))  0;
        sind(temp.angle_phi(i,1)) cosd(temp.angle_phi(i,1))   0;
        0                   0                   1];
    temp.Rz2=[1 0 0
              0 0 -1
              0 1 0];
    temp.R_cam2Base=[cosd(-temp.angle_phi(i,1)) 0 sind(-temp.angle_phi(i,1));
        sind(-temp.angle_phi(i,1)) 0 -cosd(-temp.angle_phi(i,1));
        0                   1                   0];
    %         temp.Rz2=eye(3);
    temp.tip_exp_baseFrame(i,2:4)=((temp.Rz*temp.Rz2)'*(trainSet.tip_exp(i,2:4)'))';
end
trainSet.phi=temp.angle_phi;
trainSet.phi_rad=deg2rad(temp.angle_phi);
trainSet.tip_exp_baseFrame=temp.tip_exp_baseFrame;
%%%%%%%
if par_set.flag_plot_rawData==1
    func_compareCamWithBase(trainSet)
end
%% Calculate Ri in Arc Frame using phi only
trainSet.Ri=[];
Ri_array=zeros(length(trainSet.phi),3);
Ri_array=par_set.trianlge_length/sqrt(3)*cos(pi/3)./cos(mod((trainSet.phi_rad)-pi/6,2*pi/3)-pi/3);
for i=1:length(Ri_array)
    Ri_k=Ri_array(i,:);
    phi_rad_k=trainSet.phi_rad(i);
    trainSet.Ri(i,1)=Ri_k;
    trainSet.x_y_edge(i,:)=[Ri_k*cos(phi_rad_k-pi),Ri_k*sin(phi_rad_k-pi),0];
end
if par_set.flag_plot_movingCC==1
    funcCamFramePlotMovingCC(trainSet,par_set);
end
%% Calculate theta in base frame ranging -pi/2,pi/2
trainSet.theta_rad=2*-sign(trainSet.tip_exp_baseFrame(:,2)).*asin(sqrt(trainSet.tip_exp_baseFrame(:,2).^2)./sqrt(trainSet.tip_exp_baseFrame(:,2).^2+trainSet.tip_exp_baseFrame(:,3).^2));
trainSet.theta_deg=rad2deg(trainSet.theta_rad);

% func_fwdKinematic(trainSet,par_set);
end