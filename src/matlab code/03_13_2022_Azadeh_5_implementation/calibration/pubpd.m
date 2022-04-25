clear all;close all;clc;
sen = tcpclient("10.203.48.122",7777)
% pd_data=1.0;
load('pressure_of_tubes.mat');
pd_data= pressure_of_tubes;
for i =1:100
    i
    write(sen,pd_data(i))
    java.lang.Thread.sleep(2000)  % in mysec!
end
