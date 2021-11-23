function par_set = funcOpenloopExpEncoder( par_set,exp_case )
fprintf( 'Loading exp. data %d ... \n',exp_case )
switch exp_case
    case 1
        set1=importdata('data_collect_1.txt');
    case 2
        set1=importdata('data_collect_2.txt');
    case 3
        set1=importdata('data_collect_3.txt');
    case 4
        set1=importdata('data_collect_4.txt');
    case 5
        set1=importdata('data_collect_5.txt');
    case 6
        set1=importdata('data_collect_6.txt');
    case 7
        set1=importdata('data_collect_7.txt');
    case 8
        set1=importdata('data_collect_8.txt');
    case 9
        set1=importdata('data_collect_9.txt');
    case 10
        set1=importdata('data_collect_10.txt');
    case 11
        set1=importdata('data_collect_11.txt');
    case 12
        set1=importdata('data_collect_12.txt');
    case 13
        set1=importdata('data_collect_13.txt');
    case 14
        set1=importdata('data_collect_14.txt');
    case 15
        set1=importdata('data_collect_15.txt');
end
par=[];
time_set=set1(1:end,1)-set1(1,1);
fprintf( 'Raw Sample Freq is %d, Std. is %d\n',1/mean(time_set(2:end)-time_set(1:end-1)),1/std(time_set(2:end)-time_set(1:end-1)))
NoLoad=[];
ts=timeseries(set1(:,2:end),time_set);
%     Fs=40;T=1/Fs;
T=par_set.Ts;
timevec=0:T:time_set(end);
tsout=resample(ts,timevec);
resampleData=tsout.Data;
time_set=[];
time_set=timevec';
NoLoad(:,1)=time_set;NoLoad(:,2:size(resampleData,2)-2)=resampleData(:,4:end);
par.p_offset=(resampleData(end,1:3)-NoLoad(end,2:4));
% c1-c7
par.pd_Pa = NoLoad(1:end,1) ; par.pd_Pa(:,2:4) = 1e5 *resampleData(:,1:3)* 0.0689476;
par.pd_MPa=NoLoad(1:end,1) ; par.pd_MPa(:,2:4) = 1e-1 *resampleData(:,1:3)* 0.0689476;
par.pd_psi = NoLoad(1:end,1) ; par.pd_psi(:,2:4) = resampleData(:,1:3);
par.pm_Pa = NoLoad(1:end,1) ; par.pm_Pa(:,2:4) = 1e5 *( NoLoad(1:end,2:4))* 0.0689476;
par.pm_psi = NoLoad(1:end,1) ; par.pm_psi(:,2:4) = ( NoLoad(1:end,2:4));
par.pm_MPa=NoLoad(1:end,1) ; par.pm_MPa(:,2:4) = 1e-1 *( NoLoad(1:end,2:4))* 0.0689476;
par.encoder_mm=NoLoad(1:end,5);
% c8-c119
%     par.f_ex = NoLoad(1:end,1) ; par.f_ex(:,2:7) = 0 ;
par.tip_exp = NoLoad(1:end,1) ; par.tip_exp(:,2:4) = ( NoLoad(1:end,13:15) - mean(NoLoad(1:end,6:8)) ) ;
par.base_exp = NoLoad(1:end,1) ; par.base_exp(:,2:4) = ( NoLoad(1:end,6:8) ) ;
par.tip_RQ = NoLoad(1:end,1) ; par.tip_RQ(:,2:5) = ( NoLoad(1:end,16:19) ) ;
if size(NoLoad,2)>=20
    % c19-c22
    par.tau_x=NoLoad(1:end,19) ; par.tau_y=NoLoad(1:end,20);
    par.tau_theta=NoLoad(1:end,21);par.tau_phi=NoLoad(1:end,22);
    % c23-c27
    par.theta_code_rad=NoLoad(1:end,23);par.dtheta_code_rad=NoLoad(1:end,24);
    par.phi_code_rad=NoLoad(1:end,25);par.dphi_code_rad=NoLoad(1:end,26);
end
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