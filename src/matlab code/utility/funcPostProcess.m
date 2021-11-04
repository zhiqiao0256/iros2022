function testData = funcPostProcess(testData,s_pt,e_pt)
%%%% RMSE
x_i=[];x_i_est=[];
x_i=testData.xd_exp(s_pt:e_pt,2);
x_i_est=testData.x1_exp(s_pt:e_pt,2);
testData.rmse=sqrt(sum((x_i-x_i_est).^2)/length(x_i));
x_i=[];x_i_est=[];
x_i=testData.xdNew(s_pt:e_pt,1);
x_i_est=testData.x1_exp(s_pt:e_pt,2);
testData.rmseXdNew=sqrt(sum((x_i-x_i_est).^2)/length(x_i));
%%%% Energy of input signal
u_i=[];
u_i=testData.pm_MPa(s_pt:e_pt,2);
testData.inputEnergy=sum(u_i.^2);
fprintf( 'RMSE %d RMSE(NEW) %dand InputEnergy %d \n',testData.rmse,testData.rmseXdNew,testData.inputEnergy)

