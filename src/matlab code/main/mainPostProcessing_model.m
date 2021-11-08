
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
A_mat=[0 1;-3000/m -10000/m];% estimating a21 and a22 k/m and b/m
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
compare(valid_1,dcmodel);
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