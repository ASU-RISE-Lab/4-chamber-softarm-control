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
tauy1 = testData.pm_Pa(st_pt:ed_pt,1) - testData.pm_Pa(st_pt:ed_pt,2);
fz1 = testData.pm_Pa(st_pt:ed_pt,1) + testData.pm_Pa(st_pt:ed_pt,2) + testData.pm_Pa(st_pt:ed_pt,3);
tauy2 = testData.pm_Pa(st_pt:ed_pt,4) - testData.pm_Pa(st_pt:ed_pt,5);
fz2 = testData.pm_Pa(st_pt:ed_pt,4) + testData.pm_Pa(st_pt:ed_pt,5) + testData.pm_Pa(st_pt:ed_pt,6);

input_array= [tauy1*par_set.fz_a0*par_set.tau_l0,...
             tauy2*par_set.fz_a0*par_set.tau_l0]';
output_array = output_struct.output_array(1:2:end,st_pt:ed_pt);

z = iddata(output_array',-input_array',par_set.Ts,'Name','2-segArm');
z.InputName = {'$\tau_1$';'$\tau_2$'};
z.InputUnit = {'$Nm$';'$Nm$'};
z.OutputName = {'$\theta_1$';'$\theta_2$'};
z.OutputUnit = {'$Nm/rad$','$Nm/rad$'};
present(z)
FileName      = 'func_base2segODE_m';       % File describing the model structure.
Order         = [2 2 4];           % Model orders [ny nu nx].
ParName = {'k1';'k2';'d1';'d2';};
ParUnit = {'N/m';'N/m';'Nm';'Nm'};
ParVal    = {9.182;11.79; 6.263;4.7};         % Initial parameters. Np = 3*4
InitialStates = zeros(4,1);            % Initial initial states.
Ts            = 0;                 % Time-continuous system.

ParMin   = {eps(0);eps(0);eps(0);eps(0);};
ParMax   = Inf;   % No maximum constraint.
Parameters = struct('Name', ParName, 'Unit', ParUnit,'Value',ParVal,'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', false)
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'Arm');
present(nlgr)
return
compare(nlgr,z)

return
%%
figure('Name', [z.Name ': tau input 1 -> Angular position output 1']);
plot(z(:,1,1))
figure('Name', [z.Name ': tau input 2 -> Angular position output 2']);
plot(z(:,2,2))
%%
opt = nlgreyestOptions('SearchMethod', 'lm','Display', 'on');
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