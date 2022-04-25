clear all;close all;clc;
load('pressure_of_tubes.mat');
pd_data_raw= pressure_of_tubes;
pd_data=pd_data_raw+1;
sen = tcpclient("10.203.48.122",7777)
recv= tcpserver("10.203.49.203",6666)
% pd_data=1.0;
datasize = 14;
x_bias = 0.0022;
R_initial = [ 1.0000   0.0001   0.0008
             -0.0001   1.0000  -0.0027
             -0.0008   0.0027   1.0000];
%%
for i =1:100
    i
%     write(sen,pd_data(i))
%     java.lang.Thread.sleep(100)  % in mysec!
%      if i==1
%            pd_data(i) = 1;
%      else
%            pd_data(i) = pressure_of_tubes(i-1);
%      end
     if pd_data(i)>=20
        pd_data(i)=20;
     elseif pd_data(i)<=0
        pd_data(i)=0;
     end
     write(sen,pd_data(i))
     java.lang.Thread.sleep(100)  % in mysec!
     mocap_data = read(recv,datasize,"double")
     if size(mocap_data)>0
        position_global = (mocap_data(8:10)-mocap_data(1:3))';
        position(1) = position_global(1,1)-x_bias;
        position(2) = position_global(2,1); 
        position(3) = position_global(3,1); 
%         print('position tip=',position)
        h = mocap_data(11:14)';
        h1 = h(1); h2 = h(2); h3 = h(3); h4 = h(4);
        R = eye(3)+ 2/( h'*h )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2];
        position_local = (R_initial'*R)*position'
     end
end
plot(pd_data)
