function [output] = funcKnownTerm2seg_v3(testData,par_set)
output =[];
    % Use mocap for angle but encoder for arclength
% State equations.
    m0 = (100 + 34*2 + 25*5)/1000; %kg
    a1 = 0.025; %m
    g = -9.8; %N/kg
    angle_th = deg2rad(5);
    timestamp=testData.time_stamp;%sec
    Ts=timestamp(2)-timestamp(1);
    mocapResult = funcComputeStateVar_v2(testData,par_set);
    stateVarArr = mocapResult.state_array_wire;
    for i = 1: length(stateVarArr)
    theta1 = stateVarArr(i,1);
    lc1 = stateVarArr(i,3);
    theta2 = stateVarArr(i,5);
    lc2 = stateVarArr(i,7);

    dtheta1 = stateVarArr(i,2);
    dlc1 = stateVarArr(i,4);
    dtheta2 = stateVarArr(i,6);
    dlc2 = stateVarArr(i,8);

    ddtheta1 = mocapResult.acc_array(i,1);
    ddlc1 = mocapResult.acc_array(i,2);
    ddtheta2 = mocapResult.acc_array(i,3);
    ddlc2 = mocapResult.acc_array(i,4);

    x = stateVarArr(i,:);
    [simpM4x4,simpC4x4,simpG4x1,condition] = funcMCGcal(x);
    mcg_array(1:4,i) = simpM4x4 * [ddtheta1;ddlc1;ddtheta2;ddlc2] + simpC4x4 * [dtheta1;dlc1;dtheta2;dlc2] + simpG4x1;
    Mi(1:4,i)  = simpM4x4 * [ddtheta1;ddlc1;ddtheta2;ddlc2];
    Ci(1:4,i) = simpC4x4 * [dtheta1;dlc1;dtheta2;dlc2];
    Gi(1:4,i) = simpG4x1;
    invM{i} = inv(simpM4x4);
    end

    
output.mcg_array = mcg_array;
output.state_array_wire = stateVarArr;
output.Mddq = Mi;
output.Gq = Gi;
output.Cqdq = Ci;
output.mean_u_pm_psi = mocapResult.mean_u_pm_psi;
output.u_pm_tf = mocapResult.u_pm_tf;
output.u_pm_psi = mocapResult.u_pm_psi;
output.acc_array = mocapResult.acc_array;
close all
figure(1)
titlelist = {'theta1','lc1','theta2','lc2'};
legnedlist = {'M','C','G'};
for i  =1 :4
subplot(4,1,i)
plot(output.Mddq(i,:))
hold on 
plot(output.Cqdq(i,:))
hold on 
plot(output.Gq(i,:))
hold on 
% plot(output.u_pm_tf(:,i))
title(titlelist{i})
end 
legend('M','C','G');
unitlistleft = {'Nm','N','Nm','N'};
unitlistright = {'rad','m','rad','m'};
ylimleft = [-100 100; 0 1000; -100 100; 0 1000];
ylimright = [deg2rad(-45) deg2rad(45); 0.13 0.16; deg2rad(-45) deg2rad(45); 0.09 0.12];
figure(2)
for i  =1 :4
subplot(4,1,i)
yyaxis left
plot(output.u_pm_tf(:,i)' - output.Mddq(i,:) - output.Cqdq(i,:) - output.Gq(i,:))
ylabel(unitlistleft{i})
ylim(ylimleft(i,:))
hold on
yyaxis right
ylabel(unitlistright{i})
ylim(ylimright(i,:))
plot(output.state_array_wire(:,2*i-1))
title(titlelist{i})
end 
end
