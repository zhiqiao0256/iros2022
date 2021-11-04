
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
%flag for read txt file or mat file 1: txt 0: mat
par_set.flag_read_exp = 1;
%flag for exp data is z up or not: 0: Y-up 1:Z-up
par_set.flag_exp_z_up = 1;
% Check data readme.txt for detail input reference
par_set.Ts=1/40;
% Geometric para.
par_set.m0=0.1;%kg segment weight
par_set.g=9.8;%% gravity constant

%% Update location of 3 chambers P1, P2, P3
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcOpenloopExp(par_set,1);
    par_set=funcOpenloopExp(par_set,2);
    par_set=funcOpenloopExp(par_set,3);
    par_set=funcOpenloopExp(par_set,4);
    par_set=funcOpenloopExp(par_set,5);
    par_set=funcOpenloopExp(par_set,6);
    par_set=funcOpenloopExp(par_set,7);
    par_set=funcOpenloopExp(par_set,8);
    %     par_set=funcOpenloopExp(par_set,7);
    save('raw_id_data.mat','par_set');
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
ylabel('l')


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
%% pressrue vs. delta
dl_matrix=[];
pd_matrix=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
        dl_matrix(index_j,1:100)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4));
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
        dl_matrix(index_j,101:200)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4));
        pd_matrix(index_j,101:200)=testData.pd_psi(i-100:i-1,2);
        stamp_pd=testData.pd_psi(i,2)
        index_j=index_j+1;
    end
end

% testData=par_set.trial3;
% stamp_pd=testData.pd_psi(2,2);
% index_j=1;
% for i =2:length(testData.pm_psi(:,1))
%     if testData.pd_psi(i,2)~= stamp_pd && mod(testData.pd_psi(i,2)-stamp_pd,1.0 )== 0.0
%        dl_matrix(index_j,201:300)=(testData.tip_exp(i-100:i-1,4)-testData.tip_exp(1,4));
%        pd_matrix(index_j,201:300)=testData.pd_psi(i-100:i-1,2);
%        stamp_pd=testData.pd_psi(i,2)
%        index_j=index_j+1;
%     end
% end

%% segment data
k=1;j=1;
input_low_mm=[];
output_low_newton=[];
input_high_mm=[];
output_high_newton=[];
input_low_meter=[];
input_high_meter=[];
d=0.32;% m
A=5.776;
for i =1:size(dl_matrix,1)
    temp_mat=[];
    temp_mat(:,1)=pd_matrix(i,:)';
    temp_mat(:,2)=dl_matrix(i,:)';
    if pd_matrix(i,1)<=4
        input_low_mm(k:k+300-1,1)=temp_mat(:,2)*1000;
        output_low_newton(k:k+300-1,1)=temp_mat(:,1)*A;
        input_low_meter(k:k+300-1,1)=temp_mat(:,2);
        k=k+300;
    else
        input_high_mm(j:j+300-1,1)=temp_mat(:,2)*1000;
        output_high_newton(j:j+300-1,1)=temp_mat(:,1)*A;
        input_high_meter(j:j+300-1,1)=temp_mat(:,2);
        j=j+300;
    end
end

%% linear identify k values for
%low session: force(N) = 2866 * dl(m) r-square= 0.88

% high session force (N)= 1765* (dl(m)-0.033)r-square=0.98

% instron force(N)= 5.776* pm (psi) analytical 6894.76*pi*0.0125^2
%% training data
train_dl_matrix=[];
train_f_matrix=[];
train_t=[];
train_dl_matrix2=[];
train_f_matrix2=[];
train_t2=[];
testData=par_set.trial1;
stamp_pd=testData.pd_psi(2,2);
index_j=1;index_j2=1;
for i =2:length(testData.pm_psi(:,1))
    if testData.pd_psi(i,2)<=4 && i<=2000
        train_dl_matrix(index_j,1)=(testData.tip_exp(i,4)-testData.tip_exp(1,4));
        %        if abs(train_dl_matrix(index_j,1))<= 1e-04
        %            train_dl_matrix(index_j,1)=0;
        %        end
        %        train_f_matrix(index_j,1)=testData.pd_psi(i,2)*5.776;
        train_f_matrix(index_j,1)=testData.pm_psi(i,2)*5.776;
        train_t_matrix(index_j,1)=testData.pd_psi(i,1);
        index_j=index_j+1;
    elseif testData.pd_psi(i,2)>=5 && i>=2200
        train_dl_matrix2(index_j2,1)=(testData.tip_exp(i,4)-testData.tip_exp(1,4));
        train_f_matrix2(index_j2,1)=testData.pd_psi(i,2)*5.776;
        train_t_matrix2(index_j2,1)=testData.pd_psi(i,1);
        index_j2=index_j2+1;
    end
end
train_dl_matrix(end)=[];
train_f_matrix(end)=[];
train_dl_matrix2(end)=[];
% train_dl_matrix2=train_dl_matrix2-train_dl_matrix2(1);
train_f_matrix2(end)=[];
train_t_matrix=train_t_matrix-train_t_matrix(1);
train_t_matrix2=train_t_matrix2-train_t_matrix2(1);
train_t_matrix(end)=[];
train_t_matrix2(end)=[];
close all
figure
plot(train_dl_matrix,'r')
hold on
plot(train_dl_matrix2,'b')
figure
plot(train_f_matrix,'r')
hold on
plot(train_f_matrix2,'b')


%% linear sysID
clc
m=0.1;b=0;
sp=1;
ep=900;
train_in=[];train_out=[];train_t=[];
train_in=train_f_matrix(sp:ep);
train_out=train_dl_matrix(sp:ep);
train_t=train_t_matrix(sp:ep);

% train_in=train_f_matrix2;
% train_out=train_dl_matrix2;
% train_t=train_t_matrix2;

data=iddata(train_out,train_in,1/40);
data.InputName='force';
data.OutputName='Angle';
A_mat=[0 1;-3000/m 0];% A22 = - b/0.1
B_mat=[0;1/m];
C_mat=[1 0];
D_mat=0;
ms = idss(A_mat,B_mat,C_mat,D_mat);
ms.Structure.a
ms.Structure.a.Free = [0 0; 1 1];
ms.Structure.b.Free = 0;
ms.Structure.c.Free = 0; % scalar expansion used
ms.Structure.d.Free = 0;
ms.Ts = 0;  % This defines the model to be continuous
ms % Initial model
dcmodel = ssest(data,ms,ssestOptions('Display','on'));
% bahaha=ss(dcmodel.A,dcmodel.B,dcmodel.C,dcmodel.D,dcmodel.Ts);
-dcmodel.A(2,1)*0.1
-dcmodel.A(2,2)*0.1
close all
compare(data,dcmodel);
dcmodel2 = dcmodel;
dcmodel2.Structure.a.Free(2,1) = 1;
dcmodel2 = pem(data,dcmodel2);
compare(data,dcmodel,dcmodel2)
% dcmodel2.sys
%% linear sysID with pressure dynamic
clc
testData=par_set.trial2;
m=0.1;b=0;
sp=1;
ep=length(testData.pm_psi(:,1));
train_in=[];train_out=[];train_t=[];
train_in=testData.pd_psi(sp:ep,2);
% train_out=[testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4)];
train_out=[testData.tip_exp(sp:ep,4)-testData.tip_exp(1,4),testData.pm_psi(sp:ep,2)];
train_t=testData.pm_psi(sp:ep,1);

data=iddata(train_out,train_in,1/40);
data.InputName='pd';
% data.OutputName={'Angle','pm']};
A_mat=[0 1 0;-3000/m -10000/m 5.766/m;0 0 -1];% A22 = - b/0.1
B_mat=[0;0;1];
C_mat=[1 0 0;0 0 1];
D_mat=0;
ms = idss(A_mat,B_mat,C_mat,D_mat);
ms.Structure.a.Free = [0 0 0; 1 1 0; 0 0 1];
ms.Structure.b.Free = [0 0 1]';
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
% dcmodel2 = dcmodel;
% dcmodel2.Structure.a.Free(2,1) = 1;
% dcmodel2 = pem(data,dcmodel2);
% compare(data,dcmodel,dcmodel2)
% dcmodel2.sys
%% lpv MODEL use all data1
comp_k_mat=[4.342, 3.83,2.765,2.214]*1000;
comp_b_mat=[1.119,91,212.9,273.1];
comp_alpha_mat=-[1.051,1.168,1.354,1.437];
comp_beta_mat=[1.221,1.187,1.35,1.386];
comp_u_mat=[1,2,3,4];

na_k_mat=[2.393, 2.147,1.896,1.798,1.711,1.642,1.639,1.687]*1000;
na_b_mat=[1.118,128.2,250.4,313.4,354.8,431.4,488.2,535.9];
na_alpha_mat=-[0.9229,1.107,1.304,1.407,1.516,1.664,1.808,1.98];
na_beta_mat=[1.184,1.141,1.312,1.38,1.493,1.609,1.756,1.934];
na_u_mat=[1,2,3,4,5,6,7,8];
save('lpv_trial1.mat','comp_alpha_mat','comp_beta_mat','comp_b_mat','comp_k_mat',...
    'comp_u_mat','na_alpha_mat','na_beta_mat','na_b_mat','na_k_mat','na_u_mat');
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