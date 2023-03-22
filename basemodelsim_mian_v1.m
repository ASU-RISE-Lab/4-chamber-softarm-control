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
% tauy1 = testData.pm_Pa(st_pt:ed_pt,1) - testData.pm_Pa(st_pt:ed_pt,2);
% fz1 = testData.pm_Pa(st_pt:ed_pt,1) + testData.pm_Pa(st_pt:ed_pt,2) + testData.pm_Pa(st_pt:ed_pt,3);
% tauy2 = testData.pm_Pa(st_pt:ed_pt,4) - testData.pm_Pa(st_pt:ed_pt,5);
% fz2 = testData.pm_Pa(st_pt:ed_pt,4) + testData.pm_Pa(st_pt:ed_pt,5) + testData.pm_Pa(st_pt:ed_pt,6);
input_array = [];output_array=[];
input_array= [output_struct.u_pm_pa(:,1)*par_set.fz_a0*par_set.tau_l0,...
              output_struct.u_pm_pa(:,2)*par_set.fz_a0,...
             output_struct.u_pm_pa(:,3)*par_set.fz_a0*par_set.tau_l0,...
             output_struct.u_pm_pa(:,4)*par_set.fz_a0]';
output_array = output_struct.state_array(st_pt:ed_pt,1:2:end);

z = iddata(output_array,input_array',par_set.Ts,'Name','2-segArm');

z.InputName = {'$\tau_1$','$f1$','$\tau_2$','$f2$'};
z.InputUnit = {'$Nm$';'$N$';'$Nm$';'$N$'};
z.OutputName = {'$\theta_1$';'$l_{c1}$';'$\theta_2$';'$l_{c2}$';};
z.OutputUnit = {'$rad$';'$m$';'$rad$';'$m$'};
present(z)
%% For elongation motion
testData =par_set.trial1;
output_struct = funcKnownTerm_v6(testData,par_set);
lc1z = output_struct.u_pm_pa(:,2)*par_set.fz_a0 - output_struct.mcg_array(2,:)';
lc1x = output_struct.state_array(:,3);
lc1dotx = output_struct.state_array(:,4);
%%
testData =par_set.trial1;
output_struct = funcKnownTerm_v6(testData,par_set);
lc1x = output_struct.u_pm_pa(:,2)*par_set.fz_a0;
lc1y = output_struct.state_array(:,3); 

lc2x = output_struct.u_pm_pa(:,4)*par_set.fz_a0;
lc2y = output_struct.state_array(:,7); 
%%% lc1 = 6.093e-05 * fz1 + 0.1155
%%% lc2 = 2.319e-05 * fz2 + 0.09438
% lc1dotx = output_struct.state_array(:,4);
% 
% lc2z = output_struct.u_pm_pa(:,4)*par_set.fz_a0 - output_struct.mcg_array(4,:)';
% lc2x = output_struct.state_array(:,7);
% lc2dotx = output_struct.state_array(:,8);
%%
FileName      = 'func_fullstate2segODE_m';       % File describing the model structure.
Order         = [4 4 8];           % Model orders [ny nu nx].
ParName = {'k1';'k2';'k3';'k4';'d1';'d2';'d3';'d4';};
ParUnit = {'N/m';'N/m';'N/m';'N/m';'Nm';'Nm';'Nm';'Nm'};
ParVal    = {9.182;9.182;11.79;11.79; 6.263;6.263;4.7;4.7};         % Initial parameters. Np = 3*4
InitialStates = zeros(8,1);           % Initial initial states.
Ts            = 0;                 % Time-continuous system.

ParMin   = {eps(0);eps(0);eps(0);eps(0);...
    eps(0);eps(0);eps(0);eps(0);};
ParMax   = Inf;   % No maximum constraint.
ParFix = {1; 1; 0; 0; 1; 1; 0; 0;};
Parameters = struct('Name', ParName, 'Unit', ParUnit,'Value',ParVal,'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', ParFix);
nlgr = idnlgrey(FileName, Order, Parameters,InitialStates, Ts, ...
    'Name', 'Arm');
present(nlgr)
%%
compare(nlgr,z)
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
opt = nlgreyestOptions('Display', 'off');
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