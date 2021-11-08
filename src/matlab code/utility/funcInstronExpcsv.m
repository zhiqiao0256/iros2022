function par_set = funcInstronExpcsv( par_set,exp_case )
fprintf( 'Loading exp. data %d ... \n',exp_case )
switch exp_case
    case 1
        set1=table2cell(readtable('data_1.csv'));
    case 2
        set1=table2cell(readtable('data_2.csv'));
    case 3
        set1=table2cell(readtable('data_3.csv'));
    case 4
        set1=table2cell(readtable('data_4.csv'));
    case 5
        set1=table2cell(readtable('data_5.csv'));
    case 6
        set1=importdata('data_6.txt');
    case 7
        set1=importdata('data_7.txt');
    case 8
        set1=importdata('data_8.txt');
    case 9
        set1=importdata('data_9.txt');
    case 10
        set1=importdata('data_10.txt');
    case 11
        set1=importdata('data_11.txt');
    case 12
        set1=importdata('data_12.txt');
    case 13
        set1=importdata('data_13.txt');
    case 14
        set1=importdata('data_14.txt');
    case 15
        set1=importdata('data_15.txt');
end
par=[];
NoLoad=[];
for j=1:size(set1,2)
    for i =1:size(set1,1)
%         set1{i,j}
        NoLoad(i,j)=set1{i,j};
    end
end

% c1-c6
% par.pd_Pa = NoLoad(1:end,1) ; par.pd_Pa(:,2:4) = 1e5 *resampleData(:,1:3)* 0.0689476;
% par.pd_MPa=NoLoad(1:end,1) ; par.pd_MPa(:,2:4) = 1e-1 *resampleData(:,1:3)* 0.0689476;
% par.pd_psi = NoLoad(1:end,1) ; par.pd_psi(:,2:4) = resampleData(:,1:3);
par.timestamp=NoLoad(1:end,1);
par.length=NoLoad(1:end,2); par.dlength=NoLoad(1:end,2)-NoLoad(1,2);
par.f_N=NoLoad(1:end,3); par.df_N=NoLoad(1:end,3)-NoLoad(1,3);
% end
switch exp_case
    case 1
        par_set.trial1=par;
    case 2
        par_set.trial2=par;
    case 3
        par_set.trial3=par;
    case 4
        par_set.trial4=par;
    case 5
        par_set.trial5=par;
    case 6
        par_set.trial6=par;
    case 7
        par_set.trial7=par;
    case 8
        par_set.trial8=par;
    case 9
        par_set.trial9=par;
    case 10
        par_set.trial10=par;
    case 11
        par_set.trial11=par;
    case 12
        par_set.trial12=par;
    case 13
        par_set.trial13=par;
    case 14
        par_set.trial14=par;
    case 15
        par_set.trial15=par;
end
end