clear all;close all;clc;
recv= tcpserver("10.203.49.203",6666)
datasize=14;
for i =1:100
    i
    mocap_data = read(recv,datasize,"double")
end

% 0.1197   -0.2168   -0.0146    1.0000   -0.0002    0.0003   -0.0000
% 
%   
% 
%     0.1218   -0.2165    0.1699   -1.0000   -0.0005    0.0003    0.0001