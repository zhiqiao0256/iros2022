function testData=funcFullModelSysID(testData,par_set)
testData=func_getPhiThetaBfromXYZ(testData,par_set);    % get phi,theta,r0
testData= func3ptFilter(testData);  % get d_phi d_theta
nlgr =funcBuildFullModel();   % get grey box model
%%%% Estimation Options
opt = nlgreyestOptions;
opt.Display='on';
% opt.SearchOptions.MaxIterations = 20;
opt.SearchMethod='auto';

%%%% Input Output Data
    z=iddata([testData.theta_rad,testData.velocity_theta_rad,testData.pm_MPa(:,2:4)],...
        [testData.pd_MPa(:,2:4),testData.beta,testData.phi_rad],0.05);
    z.InputName=nlgr.InputName;
    z.InputUnit=nlgr.InputUnit;
    z.OutputName=nlgr.OutputName;
    z.OutputName=nlgr.OutputName;
    nlgr.initial(1).Value=z.OutputData(1,1);
    nlgr.initial(2).Value=z.OutputData(1,2);
    nlgr.initial(3).Value=z.OutputData(1,3);
    nlgr.initial(4).Value=z.OutputData(1,4);
    nlgr.initial(5).Value=z.OutputData(1,5);
%     test_x=[testData.theta_rad,testData.velocity_theta_rad]';
%     test_x0=test_x(:,1);
    
    figure
    h_gcf = gcf;
    set(h_gcf,'DefaultLegendLocation','southeast');
%     h_gcf.Position = [100 100 795 634];
for i = 1:z.Nu
   subplot(z.Nu, 1, i);
   plot(z.SamplingInstants, z.InputData(:,i));
   title(['Input #' num2str(i) ': ' z.InputName{i}]);
   xlabel('');
   axis tight;
end
    xlabel([z.Domain ' (' z.TimeUnit ')']);
%     
%     figure
%     h_gcf = gcf;
%     set(h_gcf,'DefaultLegendLocation','southeast');
%     h_gcf.Position = [100 100 795 634];
% for i = 1:z.Ny
%    subplot(z.Ny, 1, i);
%    plot(z.SamplingInstants, z.OutputData(:,i));
%    title(['Output #' num2str(i) ': ' z.OutputName{i}]);
%    xlabel('');
%    axis tight;
% end
% xlabel([z.Domain ' (' z.TimeUnit ')']);
%%%% Estimate
% figure
nlgr1=nlgr;
% compare(z,nlgr1)
% fprintf('Estimating\n')
% % opt = nlgreyestOptions('Display','off');
nlgr1 = nlgreyest(z,nlgr1,opt);
% %%
figure
compare(z,nlgr1)
% figure
% pe(z, nlgr1);
% test_data.pi_grey=[nlgreyModel.Parameters(1).Value,nlgreyModel.Parameters(2).Value,nlgreyModel.Parameters(3).Value];
% fprintf('Estimated [alpha,k,d] is [%.4f,%.4f,%.4f] \n',test_data.pi_grey(1),test_data.pi_grey(2),test_data.pi_grey(3))
end