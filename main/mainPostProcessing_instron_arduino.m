
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
par_set.flag_read_exp=1;
par_set.Ts=1/40;
fprintf('System initialization done \n')
%% Read txt file or mat file
if par_set.flag_read_exp==1
    par_set=funcInstronExp(par_set,1);
    par_set=funcInstronExp(par_set,2);
    par_set=funcInstronExp(par_set,3);
    %     par_set=funcInstronExp(par_set,4);/
    %     par_set=funcInstronExp(par_set,5);
    save('raw_id_data.mat','par_set');
    fprintf( 'Saved \n' )
else
    fprintf( 'Loading... \n' );
    load('raw_id_data.mat');
    fprintf( 'Data loaded \n' );
end
return
%% Plot time results
close all
testData=par_set.trial1;
figure
plot(testData.pm_psi(:,1),testData.pm_psi(:,2)-8)
figure
plot(testData.pm_psi(:,1),testData.force(:,1))
%% force vs. time
figure
testData=par_set.trial1;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on
testData=par_set.trial2;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on
testData=par_set.trial3;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on
% testData=par_set.trial4;
% scatter(testData.pm_psi(:,2),testData.force(:,1))
% hold on
% testData=par_set.trial5;
% scatter(testData.pm_psi(:,2),testData.force(:,1))
% hold on
%% 20s7mil
close all
figure
testData=par_set.trial1;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial2;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial3;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on

t_start=5;
t_end=10;
testData=par_set.trial1;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(1:100,1:8)=output;

testData=par_set.trial2;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(101:200,1:8)=output;

testData=par_set.trial3;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(201:300,1:8)=output;

% close all
figure
f_mean=[];
f_std=[];
pd_mean=[];
for i =1:size(f_matrix,2)
    f_mean(i,1)=mean(f_matrix(:,i));
    f_std(i,1)= std(f_matrix(:,i));
    pd_mean(i,1)=input(1,i)
end
er = errorbar(pd_mean,f_mean,-f_std,+f_std);
% er = errorbar(x,data,errlow,errhigh);
er.Color = [0 0 0];
er.LineStyle = 'none';
xlabel('Pressure psi')
ylabel('Force (N)')
legend('20shoreA-7mil')
xlim([0,9])
ylim([0,50])
ins_20s7m=[];
ins_20s7m.pd_mean=pd_mean;
ins_20s7m.f_mean=f_mean;
ins_20s7m.f_std=f_std;
save('ins_20s7m.mat','ins_20s7m');

%% 30s7mil
close all
figure
testData=par_set.trial1;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial2;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial3;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on

t_start=5;
t_end=10;
testData=par_set.trial1;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(1:100,1:8)=output;

testData=par_set.trial2;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(101:200,1:8)=output;

testData=par_set.trial3;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(201:300,1:8)=output;

% close all
figure
f_mean=[];
f_std=[];
pd_mean=[];
for i =1:size(f_matrix,2)
    f_mean(i,1)=mean(f_matrix(:,i));
    f_std(i,1)= std(f_matrix(:,i));
    pd_mean(i,1)=input(1,i)
end
er = errorbar(pd_mean,f_mean,-f_std,+f_std);
% er = errorbar(x,data,errlow,errhigh);
er.Color = [0 0 0];
er.LineStyle = 'none';
xlabel('Pressure psi')
ylabel('Force (N)')
legend('20shoreA-7mil')
xlim([0,9])
ylim([0,50])
ins_30s7m=[];
ins_30s7m.pd_mean=pd_mean;
ins_30s7m.f_mean=f_mean;
ins_30s7m.f_std=f_std;
save('ins_30s7m.mat','ins_30s7m');
%% 30s7mil
close all
figure
testData=par_set.trial1;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial2;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on
testData=par_set.trial3;
scatter(testData.pm_psi(:,1),testData.pm_psi(:,2))
hold on

t_start=5;
t_end=10;
testData=par_set.trial1;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(1:100,1:8)=output;

testData=par_set.trial2;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(101:200,1:8)=output;

testData=par_set.trial3;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(201:300,1:8)=output;

% close all
figure
f_mean=[];
f_std=[];
pd_mean=[];
for i =1:size(f_matrix,2)
    f_mean(i,1)=mean(f_matrix(:,i));
    f_std(i,1)= std(f_matrix(:,i));
    pd_mean(i,1)=input(1,i)
end
er = errorbar(pd_mean,f_mean,-f_std,+f_std);
% er = errorbar(x,data,errlow,errhigh);
er.Color = [0 0 0];
er.LineStyle = 'none';
xlabel('Pressure psi')
ylabel('Force (N)')
legend('30shoreA-10mil')
xlim([0,9])
ylim([0,50])
ins_30s10m=[];
ins_30s10m.pd_mean=pd_mean;
ins_30s10m.f_mean=f_mean;
ins_30s10m.f_std=f_std;
save('ins_30s10m.mat','ins_30s10m');
%% 20s10mil
close all
figure
testData=par_set.trial1;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on
testData=par_set.trial2;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on
testData=par_set.trial3;
scatter(testData.pm_psi(:,1),testData.force(:,1))
hold on

t_start=5;
t_end=10;
testData=par_set.trial1;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(1:100,1:8)=output;

testData=par_set.trial2;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(101:200,1:8)=output;

testData=par_set.trial3;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=testData.pm_psi(i,2);
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(201:300,1:8)=output;

% close all
figure
f_mean=[];
f_std=[];
pd_mean=[];
for i =1:size(f_matrix,2)
    f_mean(i,1)=mean(f_matrix(:,i));
    f_std(i,1)= std(f_matrix(:,i));
    pd_mean(i,1)=input(1,i)
end
er = errorbar(pd_mean,f_mean,-f_std,+f_std);
% er = errorbar(x,data,errlow,errhigh);
er.Color = [0 0 0];
er.LineStyle = 'none';
xlabel('Pressure psi')
ylabel('Force (N)')
legend('20shoreA-7mil')
xlim([0,9])
ylim([0,50])
ins_20s10m=[];
ins_20s10m.pd_mean=pd_mean;
ins_20s10m.f_mean=f_mean;
ins_20s10m.f_std=f_std;
save('ins_20s10m.mat','ins_20s10m');

%% sum of figure
    load('ins_20s7m.mat');
    load('ins_30s7m.mat');
    load('ins_30s10m.mat');
    load('ins_20s10m.mat');
close all
fig_width=8.25/2;
fig_height=8.25/3;
fp=figure('units','inches','Position',[4,4,fig_width,fig_height]);
er1 = errorbar(ins_20s7m.pd_mean,ins_20s7m.f_mean,-ins_20s7m.f_std,+ins_20s7m.f_std);  
er1.Color = 'r';
er1.LineStyle = '-';
hold on
er2 = errorbar(ins_30s7m.pd_mean,ins_30s7m.f_mean,-ins_30s7m.f_std,+ins_30s7m.f_std);  
er2.Color = 'b';
er2.LineStyle = '-.';
hold on
er3 = errorbar(ins_30s10m.pd_mean,ins_30s10m.f_mean,-ins_30s10m.f_std,+ins_30s10m.f_std);  
er3.Color = 'k';
er3.LineStyle = ':';
er4= errorbar(ins_20s10m.pd_mean,ins_20s10m.f_mean,-ins_20s10m.f_std,+ins_20s10m.f_std);  
er4.Color = 'g';
er4.LineStyle = '--';
hold on
xlim([0,9])
ylim([0,50])
xlabel('Pressure (psi)')
ylabel('Force (N)')
leg=legend('20shoreA-7mil','30shoreA-7mil','30shoreA-10mil','20shoreA-10mil','Location',"southeast");
hold on
leg.ItemTokenSize=ones(2,1)*0.5;
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
return

%% segment data
%% 20s10mil

t_start=5;
t_end=10;
testData=par_set.trial1;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=index_pd;
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(1:100,1:8)=output;
pm_matrix(1:100,1:8)=pm;

testData=par_set.trial2;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=index_pd;
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(101:200,1:8)=output;
pm_matrix(101:200,1:8)=pm;

testData=par_set.trial3;
index_k=1;
index_pd=1;
index_c=1;
input=zeros(100,8);
output=zeros(100,8);
for j =1:8
    for i =1:length(testData.pm_psi(:,1))
        if testData.pm_psi(i,1)>=j*10-t_start &&  index_k <=100% manual stable
                input(index_k,index_c)=index_pd;
                output(index_k,index_c)=testData.force(i,1);
                pm(index_k,index_c)=index_pd;
                index_k=index_k+1;
            end
        end
        if index_k==101
            index_k=1;
            index_pd=index_pd+1;
            index_c=index_c+1;
        end
end
f_matrix(201:300,1:8)=output;
pm_matrix(201:300,1:8)=pm;


k=1;j=1;
d=0.25;% m
A=3.1415926*(d/2)^2;
for i =1:size(f_matrix,1)
    temp_mat=[];
    temp_mat(:,1)=pm_matrix(:,i);
    temp_mat(:,2)=f_matrix(:,i);

        output_newton(k:k+300-1,1)=temp_mat(:,2);
        input_psi(k:k+300-1,1)=temp_mat(:,1);
        k=k+300;


end