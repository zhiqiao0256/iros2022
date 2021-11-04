function testData=funcGreyBoxSysID(testData,par_set)
testData= func3ptFilter(testData);  % get d_phi d_theta
nlgr =funcBuildGreyBox3D();   % get grey box model
%%%% Estimation Options
opt = nlgreyestOptions;
opt.Display='on';
% opt.SearchOptions.MaxIterations = 20;
opt.SearchMethod='lsqnonlin';
%%%% Segment data into half half
halfPt=uint32(length(testData.theta_rad)/2);
%%%% Input Output Data 1
nlgr1=nlgr;
nlgr2=nlgr;
% compare(z,nlgr1)
fprintf('Estimating\n')
% % opt = nlgreyestOptions('Display','off');

z=iddata([testData.phi_rad(1:halfPt),testData.velocity_phi_rad(1:halfPt),testData.theta_rad(1:halfPt),testData.velocity_theta_rad(1:halfPt)],...
    [testData.pm_MPa(1:halfPt,2:4),testData.beta(1:halfPt),testData.theta_rad(1:halfPt),testData.phi_rad(1:halfPt)],par_set.Ts);
% z=iddata([testData.theta_rad(1:halfPt),testData.velocity_theta_rad(1:halfPt)],...
%     [testData.pm_MPa(1:halfPt,2:4)*1000,testData.beta(1:halfPt),testData.phi_rad(1:halfPt)],par_set.Ts);
z.InputName=nlgr1.InputName;
z.InputUnit=nlgr1.InputUnit;
z.OutputName=nlgr1.OutputName;
z.OutputName=nlgr1.OutputName;
nlgr1.initial(1).Value=z.OutputData(1,1);
nlgr1.initial(2).Value=z.OutputData(1,2);
% figure
% plot(z);
nlgr1 = nlgreyest(z,nlgr1,opt);
figure
compare(z,nlgr1)
%%%%
z2=iddata([testData.theta_rad(halfPt:end),testData.velocity_theta_rad(halfPt:end)],...
    [testData.pm_MPa(halfPt:end,2:4),testData.beta(halfPt:end),testData.phi_rad(halfPt:end)],par_set.Ts);
% z2=iddata([testData.theta_rad(1:end),testData.velocity_theta_rad(1:end)],...
%     [testData.pm_MPa(1:end,2:4),testData.beta(1:end),testData.phi_rad(1:end)],par_set.Ts);
z2.InputName=nlgr1.InputName;
z2.InputUnit=nlgr1.InputUnit;
z2.OutputName=nlgr1.OutputName;
z2.OutputName=nlgr1.OutputName;
nlgr2.initial(1).Value=z2.OutputData(1,1);
nlgr2.initial(2).Value=z2.OutputData(1,2);
nlgr2 = nlgreyest(z2,nlgr2,opt);
figure
compare(z2,nlgr2)

%     figure
%     h_gcf = gcf;
%     set(h_gcf,'DefaultLegendLocation','southeast');
% for i = 1:z.Nu
%    subplot(z.Nu, 1, i);
%    plot(z.SamplingInstants, z.InputData(:,i));
%    title(['Input #' num2str(i) ': ' z.InputName{i}]);
%    xlabel('');
%    axis tight;
% end
%     xlabel([z.Domain ' (' z.TimeUnit ')']);
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

% %%

% figure
% pe(z, nlgr1);
testData.pi_grey=[nlgr1.Parameters(1).Value,nlgr1.Parameters(2).Value,nlgr1.Parameters(3).Value];
testData.pi_grey2=[nlgr2.Parameters(1).Value,nlgr2.Parameters(2).Value,nlgr2.Parameters(3).Value];
fprintf('Estimated [alpha,k,d] is [%.4f,%.4f,%.4f] \n',testData.pi_grey(1),testData.pi_grey(2),testData.pi_grey(3))
fprintf('Estimated [alpha,k,d] is [%.4f,%.4f,%.4f] \n',testData.pi_grey2(1),testData.pi_grey2(2),testData.pi_grey2(3))
end