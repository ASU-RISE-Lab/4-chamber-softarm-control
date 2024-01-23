function [nlgr1] = funcGreyboxRep(testData,par_set,nlgr,opt)
% testData = par_set.trial2;
outputKnown = funcComputeStateVar_v3(testData,par_set);
close all
spt=1;ept=length(testData.pd_MPa);
output_array = [outputKnown.state_array_wire(spt:ept,1:2:end)];
input_array = testData.pm_psi(spt:ept,:);
z1 = iddata(output_array,input_array,par_set.Ts,'Name','train');
nlgr1 = nlgreyest(z1, nlgr, opt);
end