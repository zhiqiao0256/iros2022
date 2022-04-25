clear all;close all;clc;
sen = tcpclient("10.203.49.209",6666)
% pd_data=1.0;
load('pressure_of_tubes.mat');
pd_data = pressure_of_tubes;
for i =1:110
    i
    write(sen,pd_data(i))
    java.lang.Thread.sleep(100)  % in mysec!
end
