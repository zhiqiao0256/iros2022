function nlgr =funcBuildGreyBoxWithTorque()
    FileName      = 'funcGreyBoxOdeWithTorque';       % File describing the model structure.
    Order         = [2 2 2];           % Model orders [ny nu nx].
    Parameters    = [1; 0.5; 0.5];         % Initial parameters. Np = 3.
    InitialStates = [0; 0];            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Arm');
    set(nlgr, 'InputName', {'tau','r_0'}, ...
              'InputUnit', {'Nm','m'},               ...
              'OutputName', {'Angular position','vel'}, ...
              'OutputUnit', {'rad','rad/s'},                         ...
              'TimeUnit', 's');
          
    nlgr = setinit(nlgr, 'Name', {'Bending angle';        ... % x(1).
                       'Anguler vel'});             ... % x(2)
    nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s'});
    nlgr = setpar(nlgr, 'Name', {'Alpha';                         ... % k
                      'Stiffness';      ... % a
                      'Damper';});       ... % alpha
    nlgr = setpar(nlgr, 'Unit', {'None';'Nm/rad'; 'Nm/(rad/s)'});
    nlgr = setpar(nlgr, 'Minimum',{eps(0)*1;eps(0)*1;eps(0)*1});   % All parameters > 0!
%     nlgr.Parameters(1).Maximum=10;
%     nlgr.Parameters(2).Maximum=5;
%     nlgr.Parameters(2).Maximum=5;
%     present(nlgr);

