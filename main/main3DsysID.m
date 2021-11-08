%% Sys ID for 6link
% Fisrt run mainPostProcessing_openloop3D.m
%% Grey-box system ID
testData=[];
testData=par_set.trial1;
close all
clc
% testData=par_set.trial1;
testData=funcGetPhiThetaRifromXYZ(testData,par_set);
testData= func3ptFilter(testData);
T_i=[];tau_xy=[];
alpha0=1.1055;
T_matrix= [0  sqrt(3)/2;
            1 -1/2;];

for i =1: length(testData.phi_rad)
    sphi=sin(testData.phi_rad(i));
    cphi=cos(testData.phi_rad(i));
    T_i=[cphi sphi;
        -sphi cphi];
    pm1_MPa=testData.pm_MPa(i,2);
    pm2_MPa=testData.pm_MPa(i,3);
    pm3_MPa=testData.pm_MPa(i,4);
    d_p21=pm2_MPa-pm1_MPa;
    d_p13=pm1_MPa-pm3_MPa;
    tau_phi_theta(:,i)=T_i*T_matrix*[d_p13;d_p21];
end
testData.tau_phi=tau_phi_theta(1,:);
testData.tau_theta=tau_phi_theta(2,:);
% return
% Split data into rising and steady state cases
% return
figure
subplot(4,1,1)
plot(rad2deg(testData.pos_phi_rad),'r')
hold on
plot(rad2deg(testData.pos_theta_rad),'b')
legend('\phi','\theta')
ylabel('deg')
subplot(4,1,2)
plot(rad2deg(testData.velocity_phi_rad),'r')
hold on
plot(rad2deg(testData.velocity_theta_rad),'b')
hold on
plot(rad2deg(testData.velocity_phi_rad),'r')
hold on
plot(rad2deg(testData.velocity_theta_rad),'b')
hold on
ylabel('rad/s')
% legend('\phi','\theta')
ylim([-20,20])
subplot(4,1,3)
plot(testData.pm_psi(:,2),'r')
hold on
plot(testData.pm_psi(:,3),'b')
hold on
plot(testData.pm_psi(:,4),'k')
legend('pm_1','pm_2','pm_3')
subplot(4,1,4)
plot(testData.tau_phi,'r')
hold on
plot(testData.tau_theta,'b')
legend('\tau_\phi','\tau_\theta')
return
%% System id with only k term sample 3000-4000
clc
testData=funcSysIDGreyBoxK(testData,par_set);
beep;
return
testData=[];
%% System id with k,b term sample 3000-4000
clc
testData=funcSysIDGreyBoxKandD(testData,par_set);
beep;
return
testData=[];
%% System id with k,b term sample 8000-9000 with velocity
clc;
testData=funcSysIDGreyBoxKandDwithVel(testData,par_set);
beep;
return
%% System ID Debug with manipulated data
clc
% for i =500:1000:length(testData.pm_MPa(:,1))
%     i
% end
% return
index_k=1;
for i = 500:200:length(testData.pm_MPa(:,1))-1000
testData.spt=i;
testData.ept=testData.spt+500;
est_value=funcSysIDLeastSquare(testData,par_set);
testData.est_a11(index_k,1)=est_value(1);
    testData.est_a22(index_k,1)=est_value(2);
    testData.est_k22(index_k,1)=est_value(3);
index_k=index_k+1;
end
beep;
figure
subplot(3,1,1)
plot(testData.est_a11)
ylim([0,2])
legend('a11')
subplot(3,1,2)
plot(testData.est_a22)
ylim([0,6])
legend('a22')
subplot(3,1,3)
plot(testData.est_k22)
ylim([0,2])
legend('k22')

return

%% crossValidation MPa
testData=par_set.trial2;
testData2=par_set.trial1;

funcCompareTwoGreyBoxModelMPa(testData,testData2)

testData2=par_set.trial7;
funcCompareTwoGreyBoxModelMPa(testData,testData2)

testData2=par_set.trial3;
funcCompareTwoGreyBoxModelMPa(testData,testData2)

testData2=par_set.trial5;
funcCompareTwoGreyBoxModelMPa(testData,testData2)

%% parameter distribution Mpa
% figure
% subplot(3,1,1)
% plot([par_set.trial1.pi_grey(1);par_set.trial2.pi_grey(1);par_set.trial3.pi_grey(1);par_set.trial4.pi_grey(1);par_set.trial5.pi_grey(1);...
%                 par_set.trial6.pi_grey(1);par_set.trial7.pi_grey(1);par_set.trial8.pi_grey(1);par_set.trial9.pi_grey(1);],'o');
% ylabel('\alpha')
% subplot(3,1,2)
% plot([par_set.trial1.pi_grey(2);par_set.trial2.pi_grey(2);par_set.trial3.pi_grey(2);par_set.trial4.pi_grey(2);par_set.trial5.pi_grey(2);...
%                 par_set.trial6.pi_grey(2);par_set.trial7.pi_grey(2);par_set.trial8.pi_grey(2);par_set.trial9.pi_grey(2);],'o');
% ylabel('k')
% subplot(3,1,3)
% plot([par_set.trial1.pi_grey(3);par_set.trial2.pi_grey(3);par_set.trial3.pi_grey(3);par_set.trial4.pi_grey(3);par_set.trial5.pi_grey(3);...
%                 par_set.trial6.pi_grey(3);par_set.trial7.pi_grey(3);par_set.trial8.pi_grey(3);par_set.trial9.pi_grey(3);],'o');
% ylabel('b')

% %% Average Model MPa
% meanAlpha=mean([par_set.trial1.pi_grey(1);par_set.trial2.pi_grey(1);par_set.trial3.pi_grey(1);par_set.trial4.pi_grey(1);par_set.trial5.pi_grey(1);...
%                 par_set.trial6.pi_grey(1);par_set.trial7.pi_grey(1);par_set.trial8.pi_grey(1);par_set.trial9.pi_grey(1);]);
% meanK=    mean([par_set.trial1.pi_grey(2);par_set.trial2.pi_grey(2);par_set.trial3.pi_grey(2);par_set.trial4.pi_grey(2);par_set.trial5.pi_grey(2);...
%                 par_set.trial6.pi_grey(2);par_set.trial7.pi_grey(2);par_set.trial8.pi_grey(2);par_set.trial9.pi_grey(2);]);
% meanB=    mean([par_set.trial1.pi_grey(3);par_set.trial2.pi_grey(3);par_set.trial3.pi_grey(3);par_set.trial4.pi_grey(3);par_set.trial5.pi_grey(3);...
%                 par_set.trial6.pi_grey(3);par_set.trial7.pi_grey(3);par_set.trial8.pi_grey(3);par_set.trial9.pi_grey(3);]);
% avgModel.pi_grey=[meanAlpha,meanK,meanB];
% testData2=par_set.trial1;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial2;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial3;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial4;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial5;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial6;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial7;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial8;
% funcCompareAverageModel(avgModel,testData2);
% testData2=par_set.trial9;
% funcCompareAverageModel(avgModel,testData2);

%% bar plot for SMC
par_set.allAlpha=([par_set.trial1.pi_grey(1);par_set.trial2.pi_grey(1);par_set.trial3.pi_grey(1);...
                par_set.trial4.pi_grey(1);par_set.trial5.pi_grey(1);par_set.trial6.pi_grey(1);...
                par_set.trial7.pi_grey(1);par_set.trial8.pi_grey(1);par_set.trial9.pi_grey(1);...
                par_set.trial10.pi_grey(1);par_set.trial11.pi_grey(1);par_set.trial12.pi_grey(1);...
                par_set.trial13.pi_grey(1);par_set.trial14.pi_grey(1);par_set.trial15.pi_grey(1);]);
            
par_set.allK=([par_set.trial1.pi_grey(2);par_set.trial2.pi_grey(2);par_set.trial3.pi_grey(2);...
            par_set.trial4.pi_grey(2);par_set.trial5.pi_grey(2);par_set.trial6.pi_grey(2);...
            par_set.trial7.pi_grey(2);par_set.trial8.pi_grey(2);par_set.trial9.pi_grey(2);...
            par_set.trial10.pi_grey(2);par_set.trial11.pi_grey(2);par_set.trial12.pi_grey(2);...
            par_set.trial13.pi_grey(2);par_set.trial14.pi_grey(2);par_set.trial15.pi_grey(2);]);
        
par_set.allB=([par_set.trial1.pi_grey(3);par_set.trial2.pi_grey(3);par_set.trial3.pi_grey(3);...
            par_set.trial4.pi_grey(3);par_set.trial5.pi_grey(3);par_set.trial6.pi_grey(3);...
            par_set.trial7.pi_grey(3);par_set.trial8.pi_grey(3);par_set.trial9.pi_grey(3);...
            par_set.trial10.pi_grey(3);par_set.trial11.pi_grey(3);par_set.trial12.pi_grey(3);...
            par_set.trial13.pi_grey(3);par_set.trial14.pi_grey(3);par_set.trial15.pi_grey(3);]);

par_set.allAlpha2=([par_set.trial1.pi_grey2(1);par_set.trial2.pi_grey2(1);par_set.trial3.pi_grey2(1);...
                par_set.trial4.pi_grey2(1);par_set.trial5.pi_grey2(1);par_set.trial6.pi_grey2(1);...
                par_set.trial7.pi_grey2(1);par_set.trial8.pi_grey2(1);par_set.trial9.pi_grey2(1);...
                par_set.trial10.pi_grey2(1);par_set.trial11.pi_grey2(1);par_set.trial12.pi_grey2(1);...
                par_set.trial13.pi_grey2(1);par_set.trial14.pi_grey2(1);par_set.trial15.pi_grey2(1);]);
            
par_set.allK2=([par_set.trial1.pi_grey2(2);par_set.trial2.pi_grey2(2);par_set.trial3.pi_grey2(2);...
            par_set.trial4.pi_grey2(2);par_set.trial5.pi_grey2(2);par_set.trial6.pi_grey2(2);...
            par_set.trial7.pi_grey2(2);par_set.trial8.pi_grey2(2);par_set.trial9.pi_grey2(2);...
            par_set.trial10.pi_grey2(2);par_set.trial11.pi_grey2(2);par_set.trial12.pi_grey2(2);...
            par_set.trial13.pi_grey2(2);par_set.trial14.pi_grey2(2);par_set.trial15.pi_grey2(2);]);
        
par_set.allB2=([par_set.trial1.pi_grey2(3);par_set.trial2.pi_grey2(3);par_set.trial3.pi_grey2(3);...
            par_set.trial4.pi_grey2(3);par_set.trial5.pi_grey2(3);par_set.trial6.pi_grey2(3);...
            par_set.trial7.pi_grey2(3);par_set.trial8.pi_grey2(3);par_set.trial9.pi_grey2(3);...
            par_set.trial10.pi_grey2(3);par_set.trial11.pi_grey2(3);par_set.trial12.pi_grey2(3);...
            par_set.trial13.pi_grey2(3);par_set.trial14.pi_grey2(3);par_set.trial15.pi_grey2(3);]);        
x1=[par_set.allK;par_set.allK2];
SEM = std(x1)/sqrt(length(x1));               % Standard Error
ts = tinv([0.025  0.975],length(x1)-1);% T-Score
CI951 = max(ts*SEM);% Confidence Intervals
CI951=mean(x1)-std(x1)*1.96/sqrt(length(x1));
meanK=mean(x1);

x2=[par_set.allB;par_set.allB2];
SEM = std(x2)/sqrt(length(x2));               % Standard Error
ts = tinv([0.025  0.975],length(x2)-1);% T-Score
CI952 =max(ts*SEM);% Confidence Intervals
CI952=mean(x2)-std(x2)*1.96/sqrt(length(x2));
meanB=mean(x2);

x3=[par_set.allAlpha;par_set.allAlpha2];
SEM = std(x3)/sqrt(length(x3));               % Standard Error
ts = tinv([0.025  0.975],length(x3)-1);% T-Score
CI953 =max(ts*SEM);% Confidence Intervals
CI953=mean(x3)-std(x3)*1.96/sqrt(length(x3));
meanAlpha=mean(x3);
%%
par_set.meanK=meanK;
par_set.meanB=meanB;
par_set.meanAlpha=meanAlpha;
par_set.maxK=meanK+CI951;
par_set.maxB=meanB+CI952;
par_set.maxAlpha=meanAlpha+CI953;
%% CI95%
fp=figure('Position',[100,100,600,300]);
f3=plot(1,x1,'k*');
        hold on
plot(2,x2,'k*')
        hold on
plot(3,x3,'k*')
        hold on
        
f1=plot([1,2,3],[mean(x1),mean(x2),mean(x3)],'r*');
f1.LineWidth=4;
hold on
f2=errorbar(1,[mean(x1)],[CI951],'b');
f2.LineWidth=1.5;
f2.CapSize=40;
hold on
f2=errorbar(2,[mean(x2)],[CI952],'b');
f2.LineWidth=1.5;
f2.CapSize=40;
hold on
f2=errorbar(3,[mean(x3)],[CI953],'b');
f2.LineWidth=1.5;
f2.CapSize=40;
hold on
legend([f1,f2,f3(1)],'mean','95%CI','sample','Location','northwest')
xlim([0.5,3.5])
xticks([1,2,3])
xticklabels({'k','b','\alpha'})
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=16;
% ylim([0,2])
% for i =1:length(x1)
%     if x1(i)<mean(x1)-CI951 || x1(i)>mean(x1)+CI951
%         plot(1,x1(i),'ko')
%         hold on
%     end
% end
% for i =1:length(x2)
%     if x2(i)<mean(x2)-CI952 || x2(i)>mean(x2)+CI952
%         plot(2,x2(i),'ko')
%         hold on
%     end
% end
% for i =1:length(x3)
%     if x3(i)<mean(x3)-CI953 || x3(i)>mean(x3)+CI953
%         plot(3,x3(i),'ko')
%         hold on
%     end
% end

figure
subplot(3,1,1)
histogram(x1)
subplot(3,1,2)
histogram(x2)
subplot(3,1,3)
histogram(x3)