
%%% Major chanages:
%%%
clear all
close all
clc
%% Add path
% addpath(genpath('/home/zhiqiao/Documents/GitHub/ICRA2022/data/0915'))
% addpath(genpath('/home/zhiqiao/Documents/GitHub/ICRA2022/src/matlab code'))
%% Initialize the system
par_set=[];
%flag for plot
par_set.flag_plot_rawData =0;
%flag for print symbolic eom
par_set.flag_read_exp = 1;
%flag for exp data is z up or not: 0: Y-up 1:Z-up 
par_set.flag_exp_z_up = 1;
par_set.Ts=1/40;
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcOpenloopExp2seg(par_set,1);
    par_set=funcOpenloopExp2seg(par_set,2);
    par_set=funcOpenloopExp2seg(par_set,3);
%     par_set=funcOpenloopExpEncoder(par_set,4);
%     par_set=funcOpenloopExpEncoder(par_set,5);
%     par_set=funcOpenloopExpEncoder(par_set,6);   
%     par_set=funcOpenloopExpEncoder(par_set,7);     
%     par_set=funcOpenloopExpEncoder(par_set,8);   
%     par_set=funcOpenloopExp(par_set,7);  
%     save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
return
%% Plot CamFrame results with testData
close all
testData=par_set.trial1;
par_set.flag_plot_rawData =1;
if par_set.flag_plot_rawData== 1
figure
scatter3(testData.base_exp(:,2),testData.base_exp(:,3),testData.base_exp(:,4))
hold on
scatter3(testData.tip_exp(:,2),testData.tip_exp(:,3),testData.tip_exp(:,4))
hold on
scatter3(testData.tip2_exp(:,2),testData.tip2_exp(:,3),testData.tip2_exp(:,4))
xlabel('x')
ylabel('y')
zlabel('z')
end
return
%% Plot ud u Euler angle results with testData
testData=par_set.trial1;
time_vec=testData.pm_MPa(:,1)
testData=funcCalculateuandR(testData) 
figure
subplot(3,1,1)
scatter(time_vec,testData.ud10)
hold on
scatter(time_vec,testData.u10)
ylabel('curvature')
xlabel('time sec')
legend('ud_y','u_y')
subplot(3,1,2)
scatter(time_vec,testData.pd_psi(:,3))
hold on
scatter(time_vec,testData.pm_psi(:,3))
hold on
ylabel('Pressure psi')
legend('pd','pm')
xlabel('time sec')
subplot(3,1,3)
% scatter(time_vec,testData.eul(:,1))
% hold on
scatter(time_vec,testData.eul(:,2))
hold on
% scatter(time_vec,testData.eul(:,3))
% hold on
ylabel('angle rad')
legend('y')
xlabel('time sec')