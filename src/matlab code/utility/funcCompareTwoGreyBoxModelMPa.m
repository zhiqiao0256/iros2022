function []=funcCompareTwoGreyBoxModelMPa(dataSet1,dataSet2)
fprintf('Baseline alpha,k,b are %.4f, %.4f, %.4f \n',dataSet1.pi_grey(1),dataSet1.pi_grey(2),dataSet1.pi_grey(3))
baseModel=funcBuildGreyBox();
baseModel.Parameters(1).Value=dataSet1.pi_grey(1);
baseModel.Parameters(2).Value=dataSet1.pi_grey(2);
baseModel.Parameters(3).Value=dataSet1.pi_grey(3);
    baseModel.initial(1).Value=dataSet1.theta_rad(1,1);
    baseModel.initial(2).Value=dataSet1.velocity_theta_rad(1,1);
%%%% DataSet2
Exp2=iddata([dataSet2.theta_rad,dataSet2.velocity_theta_rad],...
    [dataSet2.pm_MPa(:,2:4),dataSet2.beta,dataSet2.phi_rad],0.05);
Exp2.InputName=baseModel.InputName;
Exp2.InputUnit=baseModel.InputUnit;
Exp2.OutputName=baseModel.OutputName;
Exp2.OutputName=baseModel.OutputName;
figure('Position',[600,600,600,400])
compare(Exp2,baseModel)
end