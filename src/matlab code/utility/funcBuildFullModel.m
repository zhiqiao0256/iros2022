function nlgr =funcBuildFullModel()
    FileName      = 'funcFullModelOde';       % File describing the model structure.
    Order         = [5 5 5];           % Model orders [ny nu nx].
    Parameters    = [1; 0.1;0.1;-0.1;0.1];         % Initial parameters. Np = 3.
    InitialStates = [0; 0;0 ;0;0];            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Arm');
    set(nlgr, 'InputName', {'Pd1','Pd2','Pd3','r_0','Phi'}, ...
              'InputUnit', {'MPa','MPa','MPa','m','rad'},               ...
              'OutputName', {'Angular position','vel','pm1','pm2','pm3'}, ...
              'OutputUnit', {'rad','rad/s','MPa','MPa','MPa'},                         ...
              'TimeUnit', 's');
          
%     nlgr = setinit(nlgr, 'Name', {'Bending angle';        ... % x(1).
%                        'Anguler vel';});             ... % x(2)
%     nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s'});
    nlgr = setpar(nlgr, 'Name', {'Alpha';                         ... % k
                      'Stiffness';      ... % a
                      'Damper';
                      'c';
                      'd'});       ... % alpha
    nlgr = setpar(nlgr, 'Unit', {'None';'Nm/rad'; 'Nm/(rad/s)';'None';'None'});
    nlgr.Parameters(1).Minimum=eps(0)*1;
    nlgr.Parameters(2).Minimum=eps(0)*1;
    nlgr.Parameters(3).Minimum=eps(0)*1;
    nlgr.Parameters(4).Maximum=eps(0)*-1;
    nlgr.Parameters(5).Minimum=eps(0)*1;
%     nlgr = setpar(nlgr, 'Minimum', num2cell(eps(0)*[]));   % All parameters > 0!
    present(nlgr);