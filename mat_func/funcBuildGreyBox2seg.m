function nlgr =funcBuildGreyBox2seg()
    FileName      = 'funcGreyBoxOde2seg';       % File describing the model structure.
    Order         = [4 4 8];           % Model orders [ny nu nx].
    Parameters    = ones(12,1);         % Initial parameters. Np = 12
    InitialStates = zeros(8,1);            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Arm');
    set(nlgr, 'InputName', {'tauy1','fz1','tauy2','fz2'}, ...
              'InputUnit', {'Nm','N','Nm','N'},               ...
              'OutputName', {'a1','lc1','a2','lc2',}, ...
              'OutputUnit', {'rad','m','rad','m'},                         ...
              'TimeUnit', 's');
          
    nlgr = setinit(nlgr, 'Name', {'angle1'; 'velangle1'; 'clength1';'velclength1'; ...
                       'angle2'; 'velangle2'; 'clength2';'velclength2';});             ... 
    nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s';'m';'m/s'; ...
                                  'rad'; 'rad/s';'m';'m/s'});
    nlgr = setpar(nlgr, 'Name', {'ka1';'kl1';'ka2';'kl2'; ...
                                 'da1';'dl1';'da2';'dl2'; ...
                                 'amptauy';'ampfz1';'amptauy2';'ampfz2'}); 
    nlgr = setpar(nlgr, 'Unit', {'Nm/rad'; 'N/m'; 'Nm/rad'; 'N/m'; ...
                                 'N/(m/s)';'Nm/(rad/s)';'N/(m/s)';'Nm/(rad/s)'; ...
                                 'None';'None';'None';'None'});
    nlgr = setpar(nlgr, 'Minimum',{eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1; ...
                                   eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1; ...
                                   eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1});   % All parameters > 0!
%     nlgr.Parameters(1).Maximum=5;
%     nlgr.Parameters(2).Maximum=5;
%     nlgr.Parameters(2).Maximum=5;
    present(nlgr);

