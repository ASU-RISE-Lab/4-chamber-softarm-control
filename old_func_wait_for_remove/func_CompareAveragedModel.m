function [par_set]=func_CompareAveragedModel(par_set)

%% d_pm=a*pm+b*pd
a=1;b=1;
fcn_type='d';
Ts=0.05;
para=[];
para=[par_set.trail_0_25psi.pressCntrl;
    par_set.trail_0_20psi.pressCntrl;
    par_set.trail_0_15psi.pressCntrl;
    par_set.trail_0_10psi.pressCntrl;
    par_set.trail_0_5psi.pressCntrl;
    par_set.trail_5_25psi.pressCntrl;
    par_set.trail_10_25psi.pressCntrl;
    par_set.trail_15_25psi.pressCntrl;
    par_set.trail_20_25psi.pressCntrl;];
num_set=linspace(1,9,9);
figure('Position',[600,600,600,400])
subplot(2,1,1)
scatter(num_set,para(:,1))
legend('a')
subplot(2,1,2)
scatter(num_set,para(:,2))
legend('b')
xlabel('Index (N)')
par_set.mean_a=mean(para(:,1));
par_set.std_a=std(para(:,1));
par_set.mean_b=mean(para(:,2));
par_set.std_b=std(para(:,2));
fprintf('Averaged a is %.4f, b is %.4f',par_set.mean_a,par_set.mean_b)
averaged_model=idgrey('func_pressrueController',{par_set.mean_a,par_set.mean_b},'d');
%%

%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_0_25psi;
%%%%%%%%%%

testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,28])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_0_25psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_0_20psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_0_20psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_0_15psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_0_15psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_0_10psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_0_10psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_0_5psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_0_5psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_5_25psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_5_25psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_10_25psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_10_25psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_15_25psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_15_25psi=data_set;
%%%%%%%%%
%%
%%%%%%%%%%
data_set=[];testOutput=[];testIntput=[];
data_set=par_set.trail_20_25psi;
%%%%%%%%%%
% data_set=[];testOutput=[];testIntput=[];
testIntput=data_set.pd_psi(:,2);
testOutput=data_set.pm_psi(:,2);

test_data=iddata([testOutput],[testIntput],Ts);
test_data.OutputName='Pressure';
test_data.OutputUnit='psi';
test_data.Tstart = 0;
test_data.TimeUnit = 's';
controller_model=idgrey('func_pressrueController',{a,b},'d');
opt = greyestOptions('InitialState','estimate','Display','off');
opt.EnforceStability = false;
controller_model = greyest(test_data,controller_model,opt);
figure('Position',[600,600,600,400])
compare(test_data,controller_model,averaged_model)
ylim([0,25])
legend('Exp. data','Indi. model','Avg. model','Orientation','horizontal','Location','northoutside')
data_set.pressCntrl=[controller_model.Structure.Parameters(1).Value,controller_model.Structure.Parameters(2).Value];
%%%%%%%%
par_set.trail_20_25psi=data_set;
%%%%%%%%%
end