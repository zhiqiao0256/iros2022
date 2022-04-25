clear all;close all;clc;

% pd_data=1.0;
load('pressure_of_tubes.mat');
p1=pressure_of_tubes';
p2=pressure_of_tubes';
p3=pressure_of_tubes';
p4=pressure_of_tubes';
pd_data =zeros(400,4);
sen = tcpclient("10.203.48.122",7777)

for i =1:400
    i
    if i<=100
        pd_data= [1*p1,0*p2,1*p3,0*p4];
        write(sen,pd_data(i,:))
          java.lang.Thread.sleep(100)  % in mysec!
    elseif i<=200
        pd_data= [0*p1,1*p2,0*p3,1*p4];
        write(sen,pd_data(i-100,:))
         java.lang.Thread.sleep(100)  % in mysec!
    elseif i<=300
        pd_data= [1*p1,0*p2,0*p3,1*p4];
        write(sen,pd_data(i-200,:))
          java.lang.Thread.sleep(100)  % in mysec!
    else
        pd_data= [0*p1,1*p2,1*p3,0*p4];
        write(sen,pd_data(i-300,:))
         java.lang.Thread.sleep(100)  % in mysec!
    end
%     write(sen,pd_data(i,:))
%     java.lang.Thread.sleep(100)  % in mysec!
end
