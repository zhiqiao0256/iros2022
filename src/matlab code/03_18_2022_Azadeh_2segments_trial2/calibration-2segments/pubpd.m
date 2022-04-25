clear all;close all;clc;
sen = tcpclient("10.203.48.122",7777)
% pd_data=1.0;
load('pressure_of_tubes.mat');
pd_data= [3*ones(100,1),3*ones(100,1),3*ones(100,1),3*ones(100,1)];
for i =1:100
    i
    write(sen,pd_data(i,:))
    java.lang.Thread.sleep(100)  % in mysec!
end
