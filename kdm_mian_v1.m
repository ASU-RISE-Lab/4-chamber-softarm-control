%% Greybox main
clc;close all;clear all;
%% Initialize the system
par_set=[];
%flag for EOM deriviation
par_set.EOM=0;
%flag for plot
par_set.flag_plot_rawData =0;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;

%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 0;
% Check data readme.txt for detail input reference
par_set.Ts=1/60;

par_set.fz_a0 = (25/1000)*(60/1000);%m^2
par_set.tau_l0 =48/1000;%m

par_set.R1_stand_off = 0.05;% m
% par_set.R1_stand_off = 0.03;% m
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    for i = 1:11
    par_set= funcLoadExp2Seg(par_set,i);
    end
%     par_set= funcLoadExp2Seg(par_set,1);
%     par_set= funcLoadExp2Seg(par_set,2);
%     par_set= funcLoadExp2Seg(par_set,3);
    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
%%
%%% Use mocap for angle estimation  
%%% Use wire encoder for arc length estimation
%%% End %%%
testData =par_set.trial1;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);

input_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0]';
output_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

z = iddata(output_array(:,1:2:end),input_array',par_set.Ts,'Name','2-segArm');
z.InputName = {'$\tau_1$';'$\tau_2$'};
z.InputUnit = {'$Nm$';'$Nm$'};
z.OutputName = {'$\theta_1$';'$\theta_2$'};
z.OutputUnit = {'$rad$','$rad$'};
present(z)
return
%%
close all
figure('Name', [z.Name ': tau input 1 -> Angular position output 1']);
plot(z(:,1,1))
figure('Name', [z.Name ': f input 1 ->  arc length 1']);
plot(z(:,2,2))
figure('Name', [z.Name ': tau input 2 -> Angular position output 2']);
plot(z(:,3,3))
figure('Name', [z.Name ': f input 2 ->  arc length 2']);
plot(z(:,4,4))
%%
FileName      = 'func_base2segODE_m';       % File describing the model structure.
Order         = [2 2 4];           % Model orders [ny nu nx].
Parameters    = [9.182;11.79; 6.263;4.7];         % Initial parameters. Np = 3*4
InitialStates = zeros(4,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'Arm');
present(nlgr)
compare(nlgr,z)
return
%%
opt = nlgreyestOptions('Display', 'on');
nlgr1 = nlgreyest(z, nlgr, opt);
return
%%
Parameters    = [9.182;11.79;9.182;11.79;...
                6.263;4.7;9.182;11.79];         % Initial parameters. Np = 3*4
FileName      = 'func2segODE_m';       % File describing the model structure.
Order         = [2 2 4];           % Model orders [ny nu nx].
Parameters    = [9.182;11.79;9.182;11.79;...
                6.263;4.7;9.182;11.79];         % Initial parameters. Np = 3*4
InitialStates = zeros(8,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.

present(nlgr);
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(z, nlgr, opt);
%% Learning part
testData =par_set.trial1;
% output_struct = funcKnownTerm_v4(testData,par_set);
%%% optional update using alpha parameters
output_struct = funcKnownTerm_v5(testData,par_set);
st_pt = 1; ed_pt = length(testData.pm_psi);

input_array = [];output_array=[];
y_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]- output_struct.mcg_array';
input_array = output_struct.state_array(:,1:2:end);

z1 = iddata(y_array(:,1),input_array(:,1),par_set.Ts,'Name','2-segArm');
nn_pred1 = input_array(:,1);
nn_resp1 = y_array(:,1);

nn_pred2 = input_array(:,2)/0.2;
nn_resp2 = y_array(:,2)/(40*6894.76*par_set.fz_a0);

nn_pred3 = input_array(:,3);
nn_resp3 = y_array(:,3);
%%
 