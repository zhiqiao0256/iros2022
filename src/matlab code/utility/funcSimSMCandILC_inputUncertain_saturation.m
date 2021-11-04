function par_set=funcSimSMCandILC_inputUncertain_saturation(par_set)
close all
%% Input Output data
testData=par_set.trial1;
Ts=0.01;
timeArray=[0:Ts:60]';%sec
x1=zeros(length(timeArray),1);
x2=zeros(length(timeArray),1);
dx1=zeros(length(timeArray),1);
dx2=zeros(length(timeArray),1);
e=zeros(length(timeArray),1);
de=zeros(length(timeArray),1);
s=zeros(length(timeArray),1);
uUnbound=zeros(length(timeArray),1);
uBound=zeros(length(timeArray),1);
m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=par_set.L;      % segment length
% Input signal
%%% Sin Wave
Amp=deg2rad(10);
Boff=deg2rad(-30);
freq=0.1;%Hz
xd=-Amp*sin(2*pi*freq*timeArray)+Boff;
ddxd=Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
%%%% Step input
% %           seed = rng;
%           numOfSetps = deg2rad(-10+(-50-(-10))*rand(1,6));
%           stepInterval=int64(linspace(1,length(timeArray),length(numOfSetps)));
% xd=ones(length(timeArray),1);
% ddxd=zeros(length(timeArray),1);
% for step_i=1:length(numOfSetps)-1
%     xd(stepInterval(step_i):stepInterval(step_i+1)-1,1)=numOfSetps(step_i)*xd(stepInterval(step_i):stepInterval(step_i+1)-1,1);
% end
% xd(end,1)=xd(end-1,1);
%%
%%%% Initial values
x1(2)=testData.theta_rad(2);x2(1)=0;
e(1)=xd(1)-x1(1);
de(1)=0;
uBound(1)=0;
r0=mean(testData.beta);
phi=testData.phi_rad;
pm1=testData.pm_MPa(:,2);
pm2=testData.pm_MPa(:,3);
pm3=testData.pm_MPa(:,4);
alpha=par_set.meanAlpha;
k=par_set.meanK;
b=par_set.meanB;
%%% smc tunning parameters
lambda=10;
eta=1;
epsi=10;
saturationBound=deg2rad(3);
Km=par_set.maxK-k;
Dm= par_set.maxB-b;
Alpham= (par_set.maxAlpha-alpha)/alpha;
u_smc=zeros(length(timeArray),1);
%%% ILC parameters
max_iteration=100;
kp=0.03;
e_old=zeros(length(timeArray),1);
u_old=e_old;
u_current=u_old;
RMSE_k=zeros(max_iteration,1);
x_k=zeros(length(timeArray),max_iteration);
%%% Randomize the Delta K and D
deltaD=0;
deltaK=0;
seed1=rng;
Kmax = Km;
Kmin= -Km;
deltaK = (Kmax-Kmin).*rand(1,1) + Kmin;
%
seed2=rng;
Dmax =Dm;
Dmin= -Dm;
deltaD = (Dmax-Dmin).*rand(1,1) + Dmin;
deltaD=Dmax;
deltaK=Kmax;

seed3=rng;
Alphamax =Alpham;
Alphamin= -Alpham;
deltaAlpha = (Alphamax-Alphamin).*rand(1,1) + Alphamin;
%%% Max uncertainty
% deltaD=Dmax;
% deltaK=Kmax;
% deltaAlpha=Alphamax;

% Tm=max(abs((par_set.maxAlpha-alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
%         -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2))));
uMax= max(abs((alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
    -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2))));
uMin=0.0;
% p1_max_psi=50;
% % p1_min_psi=1;
% p1_max_MPa=p1_max_psi/145.038;
% uMax= max(abs((alpha)*(sin(phi).*(0.5*p1_max_MPa+0.5*pm2-pm3)...
%     -sqrt(3)*cos(phi).*(0.5*p1_max_MPa-0.5*pm2))));
% uMin=0.0;
%%ILC
for iteration=1:max_iteration
    %% %For each iteration
    for i=2:length(timeArray)-1
        %% Sliding Mode controller
        e(i,1)=xd(i,1)-x1(i,1);
        de(i,1)=(e(i)-e(i-1))/Ts;
        s(i,1)=lambda*e(i,1)+de(i,1);
        %%% Variable caculation
        theta=x1(i,1);
        dtheta=x2(i,1);
        Izz=m0*r0^2;
        M=Izz/4 + m0*((cos(theta/2)*(r0 - L/theta))/2 +...
            (L*sin(theta/2))/theta^2)^2 + (m0*sin(theta/2)^2*(r0 - L/theta)^2)/4;
        C_simp=-(L*dtheta*m0*(2*sin(theta/2) - theta*cos(theta/2))*(2*L*sin(theta/2)...
            - L*theta*cos(theta/2) + r0*theta^2*cos(theta/2)))/(2*theta^5);
        G_simp=-(g*m0*(L*sin(theta) + r0*theta^2*cos(theta) - L*theta*cos(theta)))/(2*theta^2);
        
        f=M\(-k*x1(i,1) -(b+C_simp)*x2(i,1)- G_simp);
        %%%% Update input u with input uncetainty sliding surface
        %%%% saturation and ilc term
        if abs(s(i,1)/saturationBound)<=1
            sat=s(i,1)/saturationBound;
        else
            sat=sign(s(i,1));
        end
        B_star=Alphamax*abs(1/M);
        eta=1/(1+B_star)*(B_star*abs(lambda*de(i,1)+ddxd(i,1) - f)+Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)+epsi);
        u_smc(i,1)=M/alpha*(lambda*de(i,1)+ddxd(i,1) - f+ eta*sat);
        u_current(i,1)=u_old(i,1)+kp*e(i,1);
        u_alpha(i,1)=u_smc(i,1)+u_current(i,1);
        %     %%% Input Signal bound
        if u_alpha(i,1)>=uMax
            uBound(i,1)=uMax*sign(u_alpha(i,1));
        elseif u_alpha(i,1)<=uMin
            uBound(i,1)=0.0;
        else
            uBound(i,1)=u_alpha(i,1);
        end
        %% State feedback controller
        dx1(i,1)=x2(i,1);
        %     dx2(i,1)=f+1/M*u_alpha(i,1)+(deltaK*x1(i,1)+deltaD*x2(i,1))/M;
        dx2(i,1)=f+1/M*uBound(i,1)+(deltaK*x1(i,1)+deltaD*x2(i,1))/M;
        %     dx2(i,1)=f+alpha/M*uUnbound(i,1);
        %     dx2(i,1)=f+alpha/M*uUnbound(i,1)-(deltaK*x1(i,1)+deltaD*x2(i,1))/M;
        %     dx2(i,1)=f+alpha/M*uBound(i,1);
        %     dx2(i,1)=f+alpha/M*u(i,1);
        %% Update State Variable
        x1(i+1,1)=x1(i,1)+dx1(i,1)*Ts;
        x2(i+1,1)=x2(i,1)+dx2(i,1)*Ts;
    end
    x_k(:,iteration)=x1;
    u_old=u_current;
    RMSE_k(iteration,1)=sqrt(mean(xd-x1).^2);
end
%% Result compare
fp=figure('Name','smcAndilc','Position',[100,100,600,400]);
subplot(2,1,1)
plot(timeArray(2:end),xd(2:end),'r')
hold on
plot(timeArray(2:end),x_k(2:end,1),'b')
hold on
plot(timeArray(2:end),x_k(2:end,end),'k')
legend('ref','x1_{k=1}','x1_{k=n}','Orientation','horizontal')
ylim([-1,1])
title(['\lambda =',num2str(lambda),' \epsilon=',num2str(epsi),...
    ' \Delta k =',num2str(deltaK),' \Delta d =',num2str(deltaD),' \Delta \alpha =',num2str(deltaAlpha)])
ylabel('Angle(rad)')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
hold on
subplot(2,1,2)
plot([1:1:max_iteration],RMSE_k,'r')
title(['RMSE'])
ylabel('RMSE(rad)')
xlabel('Iterations')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;

fp=figure('Name','smcAndilc','Position',[500,100,600,400]);
subplot(2,1,1)
plot(timeArray(2:end),rad2deg(xd(2:end)),'r')
hold on
plot(timeArray(2:end),rad2deg(x_k(2:end,1)),'b')
hold on
plot(timeArray(2:end),rad2deg(x_k(2:end,end)),'k')
legend('ref','x1_{k=1}','x1_{k=n}','Orientation','horizontal')
ylim([-90,5])
title(['\lambda =',num2str(lambda),' \epsilon=',num2str(epsi),' k_p=',num2str(kp),' n=',num2str(max_iteration)...
    ' \Delta k =',num2str(deltaK),' \Delta d =',num2str(deltaD),' \Delta \alpha =',num2str(deltaAlpha)])
ylabel('Angle(deg)')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
hold on
subplot(2,1,2)
plot([1:1:max_iteration],rad2deg(RMSE_k),'r')
title(['RMSE'])
ylabel('RMSE(deg)')
xlabel('Iterations')
fp.CurrentAxes.FontWeight='Bold';
fp.CurrentAxes.FontSize=10;
% figure(2)
% subplot(2,1,1)
% plot(timeArray(2:end),xd(2:end),'r')
% hold on
% plot(timeArray(2:end),x1(2:end),'b')
% legend('ref','x1')
% ylim([-5,5])
% title(['Bonded Input with \lambda =',num2str(lambda),', \eta =',num2str(eta),...
%     '\Delta k =',num2str(deltaK),'\Delta d =',num2str(deltaD)])
% ylabel('Angle(rad)')
% hold on
% subplot(2,1,2)
% plot(timeArray(2:end),uBound(2:end),'r')
% title(['Input Signal u = \alphaR(\psi)p_{3x1}, u_{max}=',num2str(uMax),'Ts=',num2str(Ts)])
% ylim([-0.5,0.5])
% ylabel('Torque(N\cdotm)')
% xlabel('time')
end