clear all;close all;clc;
recv= tcpserver("10.203.49.116",55555)
datasize=14;
for i =1:1000
    i
    read(recv,datasize,"double")
end