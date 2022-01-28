%Post processing the Instron data to calculate the Elasticity of the silicone
%segment
close all;
clear all;
clc;
%% Load data
% read.csv('Specimen_RawData_1.csv');
% figure
% hold on
FolderName = ["1mms_0.is_tcyclic_RawData","1mms_1.is_tcyclic_RawData","1mms_2.is_tcyclic_RawData";
              "3mms_0.is_tcyclic_RawData","3mms_1.is_tcyclic_RawData","3mms_2.is_tcyclic_RawData";
              "5mms_0.is_tcyclic_RawData","5mms_1.is_tcyclic_RawData","5mms_2.is_tcyclic_RawData"];
for i=1:3
    figure
    hold on
    for j=1:3
        FileName1 = append(FolderName(i,j),"\Specimen_RawData_1.csv");
        my_field = strcat('T',num2str(i),num2str(j));
        genvarname(my_field);
        my_field = table2array(readtable(FileName1));
        plot(my_field(:,2),my_field(:,3))
    end
end