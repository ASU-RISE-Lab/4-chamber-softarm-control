function [output] = funcKnownTerm_v7(testData,par_set)
output =[];
    m0 = (100 + 34*2 + 25*5)/1000; %kg
%     m1 = m2;
    h0 = 0.01; %m
    g = 9.8;
mocapResult = funcComputeStateVar_v1(testData,par_set);
stateVarArr = mocapResult.state_array(:,5:8)-mocapResult.state_array(1,5:8);
acc_arr = mocapResult.acc_array(:,3:4);
output.sum_mcgTauf2xn = zeros(2,length(acc_arr));
for i  =  1:length(stateVarArr)
thetai = stateVarArr(i,1);
dthetai = stateVarArr(i,2);
lci = stateVarArr(i,3);
dlci = stateVarArr(i,4);
ddthetai = acc_arr(i,1);
ddlci = acc_arr(i,2);
taufi1x2 = mocapResult.u_pm_tf(i,3:4);
M2x2i = [((lci*cos(thetai/2))/(2*thetai) - (lci*sin(thetai/2))/thetai^2)^2*(m0*cos(thetai/2)^2 + m0*sin(thetai/2)^2) + (lci^2*m0*sin(thetai/2)^2)/(4*thetai^2) + (lci^2*m0*sin(thetai/2)^4)/(4*thetai^2) + (lci^2*m0*cos(thetai/2)^2*sin(thetai/2)^2)/(4*thetai^2), (sin(thetai/2)*((lci*cos(thetai/2))/(2*thetai) - (lci*sin(thetai/2))/thetai^2)*(m0*cos(thetai/2)^2 + m0*sin(thetai/2)^2))/thetai;
(sin(thetai/2)*((lci*cos(thetai/2))/(2*thetai) - (lci*sin(thetai/2))/thetai^2)*(m0*cos(thetai/2)^2 + m0*sin(thetai/2)^2))/thetai, (sin(thetai/2)^2*(m0*cos(thetai/2)^2 + m0*sin(thetai/2)^2))/thetai^2];

C2x2i = [((dthetai*((lci*cos(thetai/2))/(2*thetai) - (lci*sin(thetai/2))/thetai^2) + (dlci*sin(thetai/2))/thetai)*((lci*m0*sin(thetai/2))/thetai + (lci*m0*sin(thetai/2)^3)/thetai + (lci*m0*cos(thetai/2)^2*sin(thetai/2))/thetai))/4, -(dthetai*sin(thetai/2)*((lci*m0*sin(thetai/2))/thetai + (lci*m0*sin(thetai/2)^3)/thetai + (lci*m0*cos(thetai/2)^2*sin(thetai/2))/thetai))/(4*thetai);
 (dthetai*sin(thetai/2)*((lci*m0*sin(thetai/2))/thetai + (lci*m0*sin(thetai/2)^3)/thetai + (lci*m0*cos(thetai/2)^2*sin(thetai/2))/thetai))/(4*thetai), 0;];

G2x1i = [g*m0*cos(thetai/2)*((lci*cos(thetai/2))/(2*thetai) - (lci*sin(thetai/2))/thetai^2) - (g*lci*m0*sin(thetai/2)^2)/(2*thetai)
 (g*m0*cos(thetai/2)*sin(thetai/2))/thetai];
output.sum_mcgTauf2xn(:,i) = taufi1x2' - (M2x2i*[ddthetai;ddlci;] + C2x2i* [dthetai;dlci] ...
    + G2x1i);
output.sum_mcg2xn(:,i) = (M2x2i*[ddthetai;ddlci;] + C2x2i* [dthetai;dlci] ...
    + G2x1i);
end
%%
output.state_array4xn = stateVarArr';
end 