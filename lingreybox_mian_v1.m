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
% testData = par_set.trial1;
% temp_struct = funcKnownTerm_v3(testData);
testData = par_set.trial3;
outputKnown = funcKnownTerm2seg_v2(testData,par_set);
temp_struct = outputKnown;
z = iddata([outputKnown.state_array_wire(:,1:2:end),outputKnown.u_pm_psi] ,testData.pd_psi,par_set.Ts,'Name','2-segArm');

%dpm = -0.0353*pm + 0.03768*pd
FileName      = 'func2segODE1storder_m';       % File describing the model structure.
Order         = [8 6 8];           % Model orders [ny nu nx].
Parameters    = [-0.0353,0.03768,... % dot p6x1  = a*I6x6*p6x1 + b*I6x6*u6x1
                1,0,0,0,... % A1x
                0,1,0,0,...%A2x
                0,0,1,0,...%A3x
                0,0,0,1]; % A4x        % Initial parameters. Np = 3*4
InitialStates = [outputKnown.state_array_wire(1,1:2:end),outputKnown.u_pm_psi(1,:)]';            % Initial initial states.
Ts            = 1/40;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
    'Name', 'Arm');
set(nlgr, 'InputName', {'pd1','pd2','pd3','pd4','pd5','pd6'}, ...
    'InputUnit', {'psi','psi','psi','psi','psi','psi'},               ...
    'OutputName', {'theta1','lc1','theta2','lc2','pm1','pm2','pm3','pm4','pm5','pm6'}, ...
    'OutputUnit', {'rad','m','rad','m','rad/s','m/s','psi','psi','psi','psi','psi','psi'},                         ...
    'TimeUnit', 's');

% nlgr = setinit(nlgr, 'Name', {'angle1'; 'velangle1'; 'clength1';'velclength1'; ...
%     'angle2'; 'velangle2'; 'clength2';'velclength2';});             ...
%     nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s';'m';'m/s'; ...
%     'rad'; 'rad/s';'m';'m/s'});
nlgr = setpar(nlgr, 'Name', {'a';'b';...
                            'a11';'a12';'a13';'a14';...
                            'a21';'';'dtheta2';'dlc2';});
nlgr = setpar(nlgr, 'Unit', {'None';'N';'None';'N';...
                             'Nm/rad';'N/m';'Nm/rad';'N/m';...
                             'Nm/(rad/s)';'N/(m/s)';'Nm/(rad/s)';'N/(m/s)'});
% nlgr = setpar(nlgr, 'Minimum',{eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;...
%                                eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;...
%                                eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;});   % All parameters > 0!
present(nlgr);
compare(z,nlgr)
%%
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(z, nlgr, opt);