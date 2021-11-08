
%%% Major chanages:
%%%
clear all
close all
clc
%% Initialize the system
par_set=[];
par_set.flag_read_exp=1;
par_set.Ts=1/100;
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcInstronExpcsv(par_set,1);
    par_set=funcInstronExpcsv(par_set,2);
    par_set=funcInstronExpcsv(par_set,3);
    %     par_set=funcInstronExp(par_set,4);/
    %     par_set=funcInstronExp(par_set,5);
%     save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
return
%% train data 
train_1=[];train_2=[];valid_1=[];
testData=par_set.trial1;
train_1=iddata(testData.dlength/1000,testData.df_N,par_set.Ts);
testData=par_set.trial2;
train_2=iddata(testData.dlength/1000,testData.df_N,par_set.Ts);
testData=par_set.trial3;
valid_1=iddata(testData.dlength/1000,testData.df_N,par_set.Ts);

data=train_2;
data.InputName='force(N)';
data.OutputName='dl(m)';
m=0.1;b=0;
A_mat=[0 1;-3000/m -10000/m];% A22 = - b/0.1
B_mat=[0;1];
C_mat=[1 0];
D_mat=0;
ms = idss(A_mat,B_mat,C_mat,D_mat);
ms.Structure.a.Free = [0 0; 1 1];
ms.Structure.b.Free = [0 0 ]';
ms.Structure.c.Free = 0; % scalar expansion used
ms.Structure.d.Free = 0;
ms.Ts = 0;  % This defines the model to be continuous
ms % Initial model
dcmodel = ssest(data,ms,ssestOptions('Display','off'));
% bahaha=ss(dcmodel.A,dcmodel.B,dcmodel.C,dcmodel.D,dcmodel.Ts);
% -dcmodel.A(2,1)*0.1
% -dcmodel.A(2,2)*0.1
close all
compare(data,dcmodel);
dcmodel
return
%% results 1mms
comp_k_mat=[8264,8258];%k/m
comp_b_mat=[2635,2589];%b/m
save('1mms.mat','comp_b_mat','comp_k_mat');
%% results 3mms
comp_k_mat=[8358,8349];%k/m
comp_b_mat=[972,982];%b/m
save('3mms.mat','comp_b_mat','comp_k_mat');
%% results 5mms
comp_k_mat=[8339,8454];%k/m
comp_b_mat=[690,636];%b/m
save('5mms.mat','comp_b_mat','comp_k_mat');
%% lpv MODEL use all data2
comp_k_mat=[4.784,3.863,2.853,2.231]*1000;
comp_b_mat=[0.4521,61.44,200.8,259.0];
comp_alpha_mat=-[0.9923,1.182,1.317,1.416];
comp_beta_mat=[1.267,1.191,1.355,1.393];
comp_u_mat=[1,2,3,4];

na_k_mat=[2.448,2.079,1.962,1.789,1.726,1.647,1.625,1.663]*1000;
na_b_mat=[0.3557,135.5,244.9,311.1,375.0,427.8,480.6,540.6];
na_alpha_mat=-[0.9217,1.139,1.28,1.424,1.522,1.668,1.822,1.987];
na_beta_mat=[1.171,1.134,1.322,1.384,1.508,1.607,1.776,1.607];
na_u_mat=[1,2,3,4,5,6,7,8];
save('lpv_trial2.mat','comp_alpha_mat','comp_beta_mat','comp_b_mat','comp_k_mat',...
    'comp_u_mat','na_alpha_mat','na_beta_mat','na_b_mat','na_k_mat','na_u_mat');
%% plot lpv data1
close all
t1=load('lpv_trial1.mat');
% figure('Name','comp')
% subplot(2,2,1)
% scatter(comp_u_mat,comp_k_mat)
% legend('pd2k')
%
% subplot(2,2,2)
% scatter(comp_u_mat,comp_b_mat)
% legend('pd2b')
%
% subplot(2,2,3)
% scatter(comp_u_mat,comp_alpha_mat)
% legend('pd2alpha')
%
% subplot(2,2,4)
% scatter(comp_u_mat,comp_beta_mat)
% legend('pd2beta')
%
% figure('Name','na')
%
% subplot(2,2,1)
% scatter(na_u_mat,na_k_mat)
% legend('pd2k')
%
% subplot(2,2,2)
% scatter(na_u_mat,na_b_mat)
% legend('pd2b')
%
% subplot(2,2,3)
% scatter(na_u_mat,na_alpha_mat)
% legend('pd2alpha')
%
% subplot(2,2,4)
% scatter(na_u_mat,na_beta_mat)
% legend('pd2beta')
close all
figure('Name','all')

subplot(2,2,1)
scatter(na_u_mat,na_k_mat,'b')
hold on
scatter(comp_u_mat,comp_k_mat)
title('pd2k')
legend('natural','compressed')

subplot(2,2,2)
scatter(na_u_mat,na_b_mat,'b')
hold on
scatter(comp_u_mat,comp_b_mat)
title('pd2b')
legend('natural','compressed')
subplot(2,2,3)
scatter(na_u_mat,na_alpha_mat,'b')
hold on
scatter(comp_u_mat,comp_alpha_mat)
title('pd2alpha')
legend('natural','compressed')
subplot(2,2,4)
scatter(na_u_mat,na_beta_mat,'b')
hold on
scatter(comp_u_mat,comp_beta_mat)
title('pd2beta')
legend('natural','compressed')
% title('comp')
% hold on
%% plot lpv data1 and data2
close all
figure('Name','all')
load('lpv_trial1.mat');
subplot(2,2,1)
scatter(na_u_mat,na_k_mat,'b')
hold on
scatter(comp_u_mat,comp_k_mat)
title('pd2k')
legend('natural','compressed')

subplot(2,2,2)
scatter(na_u_mat,na_b_mat,'b')
hold on
scatter(comp_u_mat,comp_b_mat)
title('pd2b')
legend('natural','compressed')
subplot(2,2,3)
scatter(na_u_mat,na_alpha_mat,'b')
hold on
scatter(comp_u_mat,comp_alpha_mat)
title('pd2alpha')
legend('natural','compressed')
subplot(2,2,4)
scatter(na_u_mat,na_beta_mat,'b')
hold on
scatter(comp_u_mat,comp_beta_mat)
title('pd2beta')
legend('natural','compressed')

load('lpv_trial2.mat');
subplot(2,2,1)
scatter(na_u_mat,na_k_mat,'b')
hold on
scatter(comp_u_mat,comp_k_mat)
title('pd2k')
legend('natural','compressed')

subplot(2,2,2)
scatter(na_u_mat,na_b_mat,'b')
hold on
scatter(comp_u_mat,comp_b_mat)
title('pd2b')
legend('natural','compressed')
subplot(2,2,3)
scatter(na_u_mat,na_alpha_mat,'b')
hold on
scatter(comp_u_mat,comp_alpha_mat)
title('pd2alpha')
legend('natural','compressed')
subplot(2,2,4)
scatter(na_u_mat,na_beta_mat,'b')
hold on
scatter(comp_u_mat,comp_beta_mat)
title('pd2beta')
legend('natural','compressed')
%% LPV linear fit
t1=load('lpv_trial1.mat');
t2=load('lpv_trial2.mat');
lpv_low_input=[t1.comp_u_mat;t2.comp_u_mat];
lpv_low_k=[t1.comp_k_mat;t2.comp_k_mat];
lpv_low_b=[t1.comp_b_mat;t2.comp_b_mat];
lpv_low_alpha=[t1.comp_alpha_mat;t2.comp_alpha_mat];
lpv_low_beta=[t1.comp_beta_mat;t2.comp_beta_mat];

lpv_high_input=[t1.na_u_mat;t2.na_u_mat];
lpv_high_k=[t1.na_k_mat;t2.na_k_mat];
lpv_high_b=[t1.na_b_mat;t2.na_b_mat];
lpv_high_alpha=[t1.na_alpha_mat;t2.na_alpha_mat];
lpv_high_beta=[t1.na_beta_mat;t2.na_beta_mat];
%% lpv_simulation 5-8 psi
testData=par_set.trial1;
sp=2500;
ep=2900;
u=[];x=[];sim_t=[];
dt=1/5000;
% u=[ones(10/dt,1)*5;ones(10/dt,1)*6;ones(10/dt,1)*7;ones(10/dt,1)*8;];
u=[ones(10/dt,1)*0;ones(10/dt,1)*8;ones(10/dt,1)*1];
% u=testData.pd_psi(sp:ep,2);
% x=[(testData.tip_exp(sp,4)-testData.tip_exp(1,4)),0,testData.pm_psi(sp,2)]
x=[0.,0,0];
m=0.1;

sim_t(1)=0.0;
k=1;
for i =1:length(u)%high
    dx=zeros(1,3);%ddl dl dpm
    lpv_k=24.6*u(i).^2 - 323.2*u(i) + 2693;
    lpv_b=-6.121*u(i).^2 + 128*u(i) - 106.2;
    lpv_beta=0.1089*u(i) + 0.9827;
    lpv_alpha= -0.14428*u(i)-0.8181;
    dx(1)=x(k,2);
    dx(2)=1/m*(-lpv_k*x(k,1)-lpv_b*x(k,2)+5.766*x(k,3));
    dx(3)=lpv_alpha*x(k,3)+ lpv_beta*u(i);
    x(k+1,1:3)=x(k,1:3)+dx*dt;
    sim_t(k+1)=k*dt;
    k=k+1;
end
close all
figure
subplot(2,1,1)
plot(sim_t,x(:,1)+0.033,'r')
hold on
plot(testData.tip_exp(:,1),testData.tip_exp(:,4)-testData.tip_exp(1,4),'b')
ylim([0,0.08])
subplot(2,1,2)
plot(sim_t(1:end-1),u,'r')
hold on
%% lpv_simulation 1-4 psi
testData=par_set.trial1;
sp=2500;
ep=2900;
u=[];x=[];sim_t=[];
dt=1/5000;
u=[ones(10/dt,1)*2;ones(10/dt,1)*3;ones(10/dt,1)*4;ones(10/dt,1)*5;];
x=zeros(length(u)+1,3);
sim_t=zeros(length(u)+1,1);
% u=testData.pd_psi(sp:ep,2);
% x=[(testData.tip_exp(sp,4)-testData.tip_exp(1,4)),0,testData.pm_psi(sp,2)]
x=[0,0,0];
m=0.1;

sim_t(1)=0.0;
k=1;
for i =1:length(u)%high
    dx=zeros(1,3);%ddl dl dpm
    lpv_k=32.5*u(i).^2 - 968.4*u(i) + 5537;
    lpv_b=-4.059*u(i).^2 + 112.9*u(i) - 114.4;
    lpv_beta=0.06*u(i) + 1.144;
    lpv_alpha= -0.1375*u(i)-0.8959;
    dx(1)=x(k,2);
    dx(2)=1/m*(-lpv_k*x(k,1)-lpv_b*x(k,2)+5.766*x(k,3));
    dx(3)=lpv_alpha*x(k,3)+ lpv_beta*u(i);
    x(k+1,1:3)=x(k,1:3)+dx*dt;
    sim_t(k+1)=k*dt;
    k=k+1;
end
close all
figure
subplot(2,1,1)
plot(sim_t,x(:,1),'r')
hold on
plot(testData.tip_exp(:,1),testData.tip_exp(:,4)-testData.tip_exp(1,4),'b')
ylim([0,0.08])
subplot(2,1,2)
plot(sim_t(1:end-1),u,'r')
hold on
%% lpv_simulation 4-5 psi jump
testData=par_set.trial3;
sp=1830;
ep=2200;
u=[];x=[];sim_t=[];
dt=1/5000;
ts=timeseries(testData.pd_psi(:,2),testData.pd_psi(:,1));

timevec=0:dt:testData.pd_psi(end,1);
tsout=resample(ts,timevec);
u=tsout.Data;
for i =1:length(u)
    if u(i)<2.0
        u(i)=0.0;
    end
end
% velocity estimation
vel=zeros(length(testData.pd_psi(:,2)),1);
vel(2:end)=(testData.tip_exp(2:end,4)-testData.tip_exp(1:end-1,4))/par_set.Ts;
sp=1;

ep=length(testData.tip_exp);
close all
figure
subplot(3,1,1)
% plot(sim_t,jump_array(:,1)*0.01,'b')
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4),'b')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),ones(length(testData.tip_exp(sp:ep,1)),1)*0.033)
ylim([0,0.08])
legend('exp')
ylabel('length (m)')
subplot(3,1,2)
% plot(sim_t,x(:,3),'r')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.pm_psi(:,2),'b')
legend('sim','exp')
% ylim([0,0.08])
ylabel('pm (psi)')
subplot(3,1,3)
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),smooth(vel),'r')
ylabel('vel m/s')
hold on
xlabel('time sec')
%%
% u=[ones(5/dt,1)*4;ones(10/dt,1)*5;ones(10/dt,1)*6;ones(10/dt,1)*7;ones(10/dt,1)*8;];
% u=u2;
x=zeros(length(u)+1,3);
sim_t=zeros(length(u)+1,1);
% u=testData.pd_psi(sp:ep,2);
% x=[(testData.tip_exp(sp,4)-testData.tip_exp(1,4)),0,testData.pm_psi(sp,2)]
x=[0,0,0];
m=0.1;
flag_jump=0;
jump_array=sim_t;
sim_t(1)=0.0;
k=1;
jump_th=0.014;
for i =1:length(u)%high
    dx=zeros(1,3);%ddl dl dpm
    if flag_jump == 0 % before jump
        if x(k,1)<=jump_th
            lpv_k=32.5*u(i).^2 - 968.4*u(i) + 5537;
%             lpv_b=-4.059*u(i).^2 + 112.9*u(i) - 114.4;
%             lpv_b=14.4*u(i).^2 + 13.28*u(i);
                        lpv_b=-21.1*u(i).^3+154.2*u(i).^2 -239.5*u(i) +107.2;
            lpv_beta=0.06*u(i) + 1.144;
            lpv_alpha= -0.1375*u(i)-0.8959;
            dx(1)=x(k,2);
            dx(2)=1/m*(-lpv_k*x(k,1)-lpv_b*x(k,2)+5.766*x(k,3));
            dx(3)=lpv_alpha*x(k,3)+ lpv_beta*u(i);
            x(k+1,1:3)=x(k,1:3)+dx*dt;
            sim_t(k+1)=k*dt;
            jump_array(k+1)=flag_jump;
            k=k+1;
        elseif x(k,1)>jump_th
            x(k+1,1)=0.04466;x(k+1,2)=0.1067;x(k+1,3)=5.6;
            sim_t(k+1)=k*dt;
            flag_jump =1;
%             jump_array(k)=flag_jump;
            jump_array(k+1)=flag_jump;
              k=k+1;
        end% u2=[ones(10/dt,1)*0;ones(10/dt,1)*0;ones(10/dt,1)*2;ones(10/dt,1)*3;...
%     ones(10/dt,1)*4;ones(10/dt,1)*5;ones(10/dt,1)*6;ones(10/dt,1)*7;ones(10/dt,1)*8;];
% figure
% plot(u)
% hold on
% plot(u2)

    else
    lpv_k=24.6*u(i).^2 - 323.2*u(i) + 2693;
%     lpv_b=-6.121*u(i).^2 + 128*u(i) - 106.2;
%     lpv_b=-1.251*u(i).^2 + 78.31*u(i);
% lpv_b=-0.9973*u(i).^3 + 9.948*u(i).^2 + 50.05*u(i);
    lpv_b=0.1161*u(i).^6 - 3.284*u(i).^5 + 36.33*u(i).^4 - 196.7*u(i).^3 + 528.2*u(i).^2 - 518.2*u(i) +157.3;
    lpv_beta=0.1089*u(i) + 0.9827;
    lpv_alpha= -0.14428*u(i)-0.8181;
    dx(1)=x(k,2);
    dx(2)=1/m*(-lpv_k*(x(k,1)-0.033)-lpv_b*(x(k,2))+5.766*x(k,3));
    dx(3)=lpv_alpha*x(k,3)+ lpv_beta*u(i);
    x(k+1,1:3)=x(k,1:3)+dx*dt;
    sim_t(k+1)=k*dt;
    jump_array(k+1)=flag_jump;
        k=k+1;

    end
    
end

%%
ts=timeseries(sim_t,x);

timevec=0:par_set.Ts:testData.pd_psi(end,1);
tsout=resample(ts,timevec);
sim_l=tsout.Data(:,1);
sim_pm=tsout.Data(:,2);
sim_time=timevec;
sp=1;


ep=length(testData.tip_exp);
close all
figure
subplot(3,1,1)
plot(sim_t,x(:,1),'r')
hold on
% plot(sim_t,jump_array(:,1)*0.01,'b')
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4),'b')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),ones(length(testData.tip_exp(sp:ep,1)),1)*0.033)
ylim([0,0.08])
legend('sim','exp','rest')
ylabel('length (m)')
subplot(3,1,2)
plot(sim_t,x(:,3),'r')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.pm_psi(:,2),'b')
legend('sim','exp')
% ylim([0,0.08])
ylabel('pm (psi)')
subplot(3,1,3)
plot(sim_t(1:end-1),u,'r')
ylabel('pd psi')
hold on
xlabel('time sec')
%%
sp=1;
testData=par_set.trial3
ep=length(testData.tip_exp);
close all
figure
subplot(3,1,1)
plot(sim_t,x(:,1),'r')
hold on
% plot(sim_t,jump_array(:,1)*0.01,'b')
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4),'b')
legend('sim','exp')
ylim([0,0.08])
ylabel('length (m)')
xlim([49 53])
subplot(3,1,2)
plot(sim_t,x(:,3),'r')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.pm_psi(:,2),'b')
legend('sim','exp')
% ylim([0,0.08])
ylabel('length (m)')
xlim([49 53])
subplot(3,1,3)
plot(sim_t(1:end-1),u,'r')
ylabel('pd psi')
hold on
xlim([49 53])
xlabel('time sec')
%% Final Plot
sp=1;
testData=par_set.trial3
ep=length(testData.tip_exp);
close all
fig_width=8.25/2;
fig_height=8.25/2;
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
subplot(2,1,1)
plot(sim_t,x(:,1),'Color','b','LineWidth',2,'LineStyle',':')
hold on
% plot(sim_t,jump_array(:,1)*0.01,'b')
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4),'Color','r','LineWidth',2,'LineStyle','-')
legend('sim','exp','Orientation','horizontal','Location','southeast')
ylim([0,0.08])
ylabel('x_1(m)')
% xlim([0 58])
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
subplot(2,1,2)
hold on
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
plot(sim_t,x(:,3),'Color','b','LineWidth',2,'LineStyle',':')
hold on
plot(testData.tip_exp(sp:ep,1)-testData.tip_exp(sp,1),testData.pm_psi(:,2),'Color','r','LineWidth',2,'LineStyle','-')
legend('sim','exp','Orientation','horizontal','Location','southeast')
% ylim([0,0.08])
ylabel('x_3(psi)')
% xlim([0 58])
% subplot(3,1,3)
% plot(sim_t(1:end-1),u,'r')
% ylabel('pd psi')
% hold on
% xlim([49 53])
xlabel('time (sec)')
box on
hold on
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
%%
figure
subplot(2,1,1)
plot(testData.pd_psi(sp:ep,1)-testData.pd_psi(sp,1),(testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4)),'b')
hold on
plot(sim_t,x(:,1),'r')
ylim([0,0.08])
subplot(2,1,2)
plot(testData.pd_psi(sp:ep,1)-testData.pd_psi(sp,1),u,'b')
hold on

% xlim()
%% Nonlinear
% low force = (-0.2594e05 * x + 4869)*x
clc
close all

FileName      = 'lowp_m';       % File describing the model structure.
Order         = [1 1 2];           % Model orders [ny nu nx].
Parameters    = [1e05];         % Initial parameters. Np = 2.
InitialStates = [0; 0];            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
    'Name', 'lowp');
nlgr
nlgr.SimulationOptions.AbsTol = 1e-6;
nlgr.SimulationOptions.RelTol = 1e-5;
nlgr = setinit(nlgr, 'Fixed', {false false}); % Estimate the initial states.
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(data, nlgr, opt);