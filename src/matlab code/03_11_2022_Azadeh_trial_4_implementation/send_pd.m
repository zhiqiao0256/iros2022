clear all;close all;clc;
sen = tcpclient("10.203.48.122",7777)
% pd_data=1.0;
load('pressure_of_tubes.mat');
pd_data= pressure_of_tubes;
for i =1:100
    i
%     write(sen,pd_data(i))
%     java.lang.Thread.sleep(100)  % in mysec!
     if i==1
           pd_data(i) = 1;
     else
           pd_data(i) = pressure_of_tubes(i-1);
     end
     if pd_data(i)>=20
        pd_data(i)=20;
     elseif pd_data(i)<=0
        pd_data(i)=0;
     end
     write(sen,pd_data(i))
     java.lang.Thread.sleep(100)  % in mysec!

end
plot(1+pd_data)