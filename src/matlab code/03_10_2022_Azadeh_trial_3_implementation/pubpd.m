clear all;close all;clc;
sen= tcpclient("10.203.49.209",6666)
pd_data=1.0;
for i =1:1000
    i
    write(sen,pd_data)
    java.lang.Thread.sleep(2000)  % in mysec!
end
