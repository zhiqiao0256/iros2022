function []=funcCompareTwoGreyBoxModel(dataSet1,dataSet2)
fprintf('Baseline alpha,k,b are %.4f, %.4f, %.4f \n',dataSet1.pi_grey(1),dataSet1.pi_grey(2),dataSet1.pi_grey(3))
FileName      = 'func_greyModel';       % File describing the model structure.
Order         = [2 5 2];           % Model orders [ny nu nx].
Parameters    = dataSet1.pi_grey';         % Initial parameters. Np = 2.
InitialStates = [dataSet2.theta_rad(1);dataSet2.velocity_theta_rad(1)];            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
baseModel = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'baseModel');
%%%% DataSet2
Exp2=iddata([dataSet2.theta_rad,dataSet2.velocity_theta_rad],...
    [dataSet2.pm_psi(:,2:4),dataSet2.beta,dataSet2.phi_rad],0.05);
set(Exp2, 'InputName', {'Pm1','Pm2','Pm3','r_0','Phi'}, ...
            'InputUnit', {'psi','psi','psi','m','rad'},               ...
          'OutputName', {'Angular position','vel'}, ...
          'OutputUnit', {'rad','rad/s'},                         ...
          'TimeUnit', 's');
figure('Position',[600,600,600,400])
compare(Exp2,baseModel)
end