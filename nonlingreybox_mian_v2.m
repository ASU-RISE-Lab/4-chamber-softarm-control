%% Greybox main
clc;close all;clear all;
load('greybox_test.mat');
% par_set.fz_a0 = (25/1000)*(60/1000);%m^2
% par_set.tau_l0 =48/1000;%m
% k1 = 26.04,d1 = 20.60
% k2 = -2.729e+04 d2 = -2.823e+04, a2 =3680
% k3 =42.66 d3 =33.87
% k4 = -3.285e+04 d2 =-3.256e+04 a4 = 3144
%% Calculate wire length
testData = par_set.trial3;
% temp_struct = funcKnownTerm_v3(testData);
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
close all
temp_struct = outputKnown;
z1 = iddata(outputKnown.state_array_wire(1:500,1:1:end),outputKnown.u_pm_tf(1:500,:),par_set.Ts,'Name','2-segArm');

FileName      = 'func2segODE_m';       % File describing the model structure.
Order         = [8 4 8];           % Model orders [ny nu nx].
Parameters    = [0, -3680, 0, -3144,... % u offset
                26.04, 2.729e+04,42.66,3.285e+04,... % k4x1
                20.60, 2.823e+04, 33.87, 3.285e+04]; % d4x1        % Initial parameters. Np = 3*4
InitialStates = outputKnown.state_array_wire(1,:)';            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
    'Name', 'Arm');
set(nlgr, 'InputName', {'tauz1','fy1','tauz2','fy2'}, ...
    'InputUnit', {'Nm','N','Nm','N'},               ...
    'OutputName', {'theta1','lc1','dtheta1','dlc1','theta2','lc2','dtheta2','dlc2'}, ...
    'OutputUnit', {'rad','m','rad/s','m/s','rad','m','rad/s','m/s'},                         ...
    'TimeUnit', 's');

nlgr = setinit(nlgr, 'Name', {'angle1'; 'velangle1'; 'clength1';'velclength1'; ...
    'angle2'; 'velangle2'; 'clength2';'velclength2';});             ...
    nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s';'m';'m/s'; ...
    'rad'; 'rad/s';'m';'m/s'});
nlgr = setpar(nlgr, 'Name', {'amptauy1';'ampfz1';'amptauy2';'ampfz2';...
                            'ktheta1';'klc1';'ktheta2';'klc2';...
                            'dtheta1';'dlc1';'dtheta2';'dlc2';});
nlgr = setpar(nlgr, 'Unit', {'None';'N';'None';'N';...
                             'Nm/rad';'N/m';'Nm/rad';'N/m';...
                             'Nm/(rad/s)';'N/(m/s)';'Nm/(rad/s)';'N/(m/s)'});
% nlgr = setpar(nlgr, 'Minimum',{eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;...
%                                eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;...
%                                eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;});   % All parameters > 0!
present(nlgr);
compare(z1,nlgr)
%%
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(z1, nlgr, opt);