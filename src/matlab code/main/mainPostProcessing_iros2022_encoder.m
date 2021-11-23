
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
%flag for EOM deriviation
par_set.EOM=1;
%flag for plot
par_set.flag_plot_rawData =0;
%flag for print symbolic eom
par_set.publishEOM=1;
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;
%flag for exp data is z up or not: 0: Y-up 1:Z-up 
par_set.flag_exp_z_up = 1;
%flag for plotting moving constant layer
par_set.flag_plot_movingCC = 0;
%flag for plotting fwd kinematic results
par_set.plot_fwdKinematic = 1;
%flag for using fixed beta value
par_set.flag_fixed_beta =0;
% Check data readme.txt for detail input reference
par_set.Ts=1/40;
% Geometric para.
par_set.trianlge_length=0.07;% fabric triangle edge length
par_set.L=0.185;%actuator length
par_set.n=6;% # of joints for augmented rigid arm
par_set.m0=0.35;%kg segment weight
par_set.g=9.8;%% gravity constant
par_set.a0=15*1e-03;%% 1/2 of pillow width
par_set.r_f=sqrt(3)/6*par_set.trianlge_length+par_set.a0; % we assume the force are evenly spread on a cirlce with radius of r_f
par_set.fixed_beta=par_set.r_f; %% Change this value as needed
%% Update location of 3 chambers P1, P2, P3
par_set.p1_angle=240;%deg p1 position w/ the base frame
% update force position of p1 p2 and p3
for i =1:3
    par_set.r_p{i}=[par_set.r_f*cosd(par_set.p1_angle+120*(i-1)),par_set.r_f*sind(par_set.p1_angle+120*(i-1)),0].';
%     par_set.f_p{i}=588.31*par_set.pm_MPa(:,i+1);
end
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcOpenloopExpEncoder(par_set,1);
    par_set=funcOpenloopExpEncoder(par_set,2);
    par_set=funcOpenloopExpEncoder(par_set,3);
    par_set=funcOpenloopExpEncoder(par_set,4);
    par_set=funcOpenloopExpEncoder(par_set,5);
    par_set=funcOpenloopExpEncoder(par_set,6);   
    par_set=funcOpenloopExpEncoder(par_set,7);     
    par_set=funcOpenloopExpEncoder(par_set,8);   
%     par_set=funcOpenloopExp(par_set,7);  
%     save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
return
%% Plot CamFrame results
close all
testData=par_set.trial1;
par_set.flag_plot_rawData =1;
if par_set.flag_plot_rawData== 1
figure
plot(testData.tip_exp(:,2:4))
figure
plot(testData.base_exp(:,2:4))
% return
end
%% plot raw z changes and pressure changes pm
close all
figure
testData=par_set.trial1;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial2;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial3;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)

testData=par_set.trial4;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial5;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial6;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial7;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
testData=par_set.trial8;
testData.elogation=testData.tip_exp(:,4)-testData.tip_exp(1,4);
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.input_pressure,testData.elogation)
hold on
xlim([0 9])
ylim([0 0.07])
ylabel('l-l_0')

%% plot raw z changes and time
close all
figure
testData=par_set.trial1;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.pm_psi(:,1),testData.elogation)
hold on
testData=par_set.trial2;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.pm_psi(:,1),testData.elogation)
hold on
testData=par_set.trial3;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.pm_psi(:,1),testData.elogation)
hold on

testData=par_set.trial4;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.pm_psi(:,1),testData.elogation)
hold on
testData=par_set.trial5;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
scatter(testData.pm_psi(:,1),testData.elogation)
% hold on
% testData=par_set.trial3;
% testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
% testData.input_pressure=testData.pm_psi(:,2);
% scatter(testData.pm_psi(:,1),testData.elogation)
hold on
% plot(testData.pm_psi(:,1),testData.pd_psi(:,3))
% xlim([0 9])
ylim([0 0.7])
xlabel('sec')
ylabel('(l-l_0)/l_0')
legend('data1','data2','data3','Orientation','horizontal')
%% pressrue vs. delta l　20s-7mil
dl_matrix=[];
pd_matrix=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,1:100)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,1:100)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
    
end
% return
testData=par_set.trial2;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,101:200)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,101:200)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

testData=par_set.trial3;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,201:300)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,201:300)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

close all
figure
dl_mean=[];
dl_std=[];
pd_mean=[];
for i =1:size(dl_matrix,1)
    dl_mean(i,1)=mean(dl_matrix(i,:));
    dl_std(i,1)= std(dl_matrix(i,:));
    pd_mean(i,1)=pd_matrix(i,1)
end
er = errorbar(pd_mean,dl_mean,-dl_std,+dl_std);  
% er = errorbar(x,data,errlow,errhigh);    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
xlabel('Pressure psi')
ylabel('(l-l_0)/l_0')
legend('20shoreA-7mil')
mocap_20s7m=[];
mocap_20s7m.pd_mean=pd_mean;
mocap_20s7m.dl_mean=dl_mean;
mocap_20s7m.dl_std=dl_std;
save('mocap_20s7m.mat','mocap_20s7m');
%% pressrue vs. delta l　20s-10mil
dl_matrix=[];
pd_matrix=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,1:100)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,1:100)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
    
end
% return
testData=par_set.trial2;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,101:200)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,101:200)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

testData=par_set.trial3;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,201:300)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,201:300)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

close all
figure
dl_mean=[];
dl_std=[];
pd_mean=[];
for i =1:size(dl_matrix,1)
    dl_mean(i,1)=mean(dl_matrix(i,:));
    dl_std(i,1)= std(dl_matrix(i,:));
    pd_mean(i,1)=pd_matrix(i,1)
end
er = errorbar(pd_mean,dl_mean,-dl_std,+dl_std);  
% er = errorbar(x,data,errlow,errhigh);    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
xlabel('Pressure psi')
ylabel('(l-l_0)/l_0')
legend('20shoreA-10mil')
mocap_20s10m=[];
mocap_20s10m.pd_mean=pd_mean;
mocap_20s10m.dl_mean=dl_mean;
mocap_20s10m.dl_std=dl_std;
save('mocap_20s10m.mat','mocap_20s10m');
%% pressrue vs. delta l　30s-7mil
dl_matrix=[];
pd_matrix=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,1:100)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,1:100)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
    
end
% return
testData=par_set.trial2;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,101:200)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,101:200)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

testData=par_set.trial3;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,201:300)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,201:300)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

close all
figure
dl_mean=[];
dl_std=[];
pd_mean=[];
for i =1:size(dl_matrix,1)
    dl_mean(i,1)=mean(dl_matrix(i,:));
    dl_std(i,1)= std(dl_matrix(i,:));
    pd_mean(i,1)=pd_matrix(i,1)
end
er = errorbar(pd_mean,dl_mean,-dl_std,+dl_std);  
% er = errorbar(x,data,errlow,errhigh);    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
xlabel('Pressure psi')
ylabel('(l-l_0)/l_0')
legend('30shoreA-7mil')
mocap_30s7m=[];
mocap_30s7m.pd_mean=pd_mean;
mocap_30s7m.dl_mean=dl_mean;
mocap_30s7m.dl_std=dl_std;
save('mocap_30s7m.mat','mocap_30s7m');
%% pressrue vs. delta l　30s-10mil
dl_matrix=[];
pd_matrix=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,1:100)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,1:100)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
    
end
% return
testData=par_set.trial4;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,101:200)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,101:200)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

testData=par_set.trial5;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
       dl_matrix(index_j,201:300)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
       pd_matrix(index_j,201:300)=testData.pd_psi(i-100:i-1,2);
       stamp_pd=testData.pd_psi(i,2)
       index_j=index_j+1;
    end
end

close all
figure
dl_mean=[];
dl_std=[];
pd_mean=[];
for i =1:size(dl_matrix,1)
    dl_mean(i,1)=mean(dl_matrix(i,:));
    dl_std(i,1)= std(dl_matrix(i,:));
    pd_mean(i,1)=pd_matrix(i,1)
end
er = errorbar(pd_mean,dl_mean,-dl_std,+dl_std);  
% er = errorbar(x,data,errlow,errhigh);    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
xlabel('Pressure psi')
ylabel('(l-l_0)/l_0')
legend('30shoreA-10mil')
mocap_30s10m=[];
mocap_30s10m.pd_mean=pd_mean;
mocap_30s10m.dl_mean=dl_mean;
mocap_30s10m.dl_std=dl_std;
save('mocap_30s10m.mat','mocap_30s10m');
%% sum of figure
    load('mocap_20s7m.mat');
    load('mocap_30s7m.mat'); 
    load('mocap_30s10m.mat'); 
    load('mocap_20s10m.mat');
close all
fig_width=8.25/3;
fig_height=8.25/4;
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
er1 = errorbar(mocap_20s7m.pd_mean,mocap_20s7m.dl_mean,-mocap_20s7m.dl_std,+mocap_20s7m.dl_std);  
er1.Color = 'red';
er1.LineStyle = '-';

hold on
er2 = errorbar(mocap_30s7m.pd_mean,mocap_30s7m.dl_mean,-mocap_30s7m.dl_std,+mocap_30s7m.dl_std);  
er2.Color = 'blue';
er2.LineStyle = '-.';
hold on

er3 = errorbar(mocap_30s10m.pd_mean,mocap_30s10m.dl_mean,-mocap_30s10m.dl_std,+mocap_30s10m.dl_std);  
er3.Color = 'black';
er3.LineStyle = ':';

er4 = errorbar(mocap_20s10m.pd_mean,mocap_20s10m.dl_mean,-mocap_20s10m.dl_std,+mocap_20s10m.dl_std);  
er4.Color = 'g';
er4.LineStyle = '--';
hold on
xlim([0,9])
ylim([0,0.8])
xlabel('Pressure (psi)')
ylabel('Extension ratio')
leg=legend('20s7m','30s7m','30s10m','20s10m','Location',"southeast",'Orientation','vertical');
leg.ItemTokenSize=ones(2,1)*0.5;
hold on
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
%% time plot
fig_width=8.25/3;
fig_height=8.25/4;
close all
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
testData=par_set.trial1;
testData.elogation=(testData.tip_exp(:,4)-testData.tip_exp(1,4))/-(testData.tip_exp(1,4)-testData.base_exp(1,4));
testData.input_pressure=testData.pm_psi(:,2);
plot(testData.pm_psi(:,1),testData.elogation,'LineWidth',2,'Color','r')
hold on
xlim([0,95])
ylim([0,0.8])
xlabel('Time (sec)')
ylabel('Extension ratio')
leg=legend('20s7m','Location',"southeast",'Orientation','vertical');
% leg.ItemTokenSize=ones(2,1)*0.5;
hold on
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;