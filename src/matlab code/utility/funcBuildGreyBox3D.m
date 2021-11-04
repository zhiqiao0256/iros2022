function nlgr =funcBuildGreyBox3D()
    FileName      = 'funcGreyBoxOde3D6link';       % File describing the model structure.
    Order         = [4 3 4];           % Model orders [ny nu nx].
    Parameters    = [0.001; 0.1; 0.001;0.1; 0.001;0.1;];         % Initial parameters. Np = 6.
    InitialStates = [0; 0;0;0];            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.
    nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Arm');
    set(nlgr, 'InputName', {'Pm1','Pm2','Pm3'}, ...
              'InputUnit', {'MPa','MPa','MPa'},               ...
              'OutputName', {'Phi','v_Phi','Theta','v_Theta'}, ...
              'OutputUnit', {'rad','rad/s','rad','rad/s'},                         ...
              'TimeUnit', 's');
          
    nlgr = setinit(nlgr, 'Name', {'phi angle';        ... % x(1).
                       'phi vel'; ... % x(2)
                       'theta angle';        ... % x(1).
                       'theta vel'; ... % x(2)
                       });            
    nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s';'rad'; 'rad/s'});
    nlgr = setpar(nlgr, 'Name', {'a11';'a22'; ...                        
                        'k11';'k22';      ...
                         'b11';'b22'; });      
    nlgr = setpar(nlgr, 'Unit', {'None';'None';'Nm/rad';'Nm/rad';'Nm/rad/s';'Nm/rad/s';});
    nlgr = setpar(nlgr, 'Minimum',{eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;eps(0)*1;});   % All parameters > 0!
    nlgr.Parameters(1).Maximum=5;
    nlgr.Parameters(2).Maximum=5;
    nlgr.Parameters(3).Maximum=5;
    nlgr.Parameters(4).Maximum=5;
    nlgr.Parameters(5).Maximum=5;
    nlgr.Parameters(6).Maximum=5;
%     present(nlgr);

