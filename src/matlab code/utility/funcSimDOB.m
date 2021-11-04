function par_set=funcSimDOB(par_set)
close all
%% Input Output data
testData=par_set.trial1;
%%%%%%
%%%%%% State Ini.
Ts=0.01; % sampling period
timeArray=[0:Ts:180]';%sec
x1=zeros(length(timeArray),1); %state variable theta
x2=zeros(length(timeArray),1); %state variable dtheta
x2_filt=x2;

dx1=zeros(length(timeArray),1);
dx2=zeros(length(timeArray),1);
e=zeros(length(timeArray),1);
de=zeros(length(timeArray),1);
s=zeros(length(timeArray),1);



m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=par_set.L;      % segment length


% Input signal
Amp=deg2rad(20);
Boff=deg2rad(-50);
freq=0.1;%Hz
xd=-Amp*sin(2*pi*freq*timeArray)+Boff;
dxd=-Amp*(2*pi*freq)*cos(2*pi*freq*timeArray);
ddxd=Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
pd1_MPa=zeros(length(timeArray),1);
%%%% Initial values
x1(1)=testData.theta_rad(end-1);
x1(1)=xd(1);
r0=mean(testData.beta);
phi=mean(testData.phi_rad);
phi=mean(testData.phi_rad);
pd1_upperlimit_MPa=40*0.00689476;
pd1_lowerlimit_MPa=1*0.00689476;
pm2_MPa=1*0.00689476;
pm3_MPa=1*0.00689476;
u_raw=zeros(length(timeArray),1); %unbounded input
u_bound=zeros(length(timeArray),1);   %bounded input

alpha=par_set.meanAlpha;
k=par_set.meanK;
b=par_set.meanB;

%%% smc tunning parameters
smc_lambda=100;
smc_epsilon=1;
%%%% system uncertainty
Km=par_set.maxK-k;
Dm= par_set.maxB-b;
Alpham= (par_set.maxAlpha-alpha)/alpha;
%%% Randomize the Delta K and D
seed1=rng;
Kmax = Km;
Kmin= -Km;
deltaK = (Kmax-Kmin).*rand(1,1) + Kmin;
%
seed2=rng;
Dmax =Dm;
Dmin= -Dm;
deltaD = (Dmax-Dmin).*rand(1,1) + Dmin;
deltaDmax=Dmax;
deltaK=Kmax;

seed3=rng;
Alphamax =Alpham;
Alphamin= -Alpham;
deltaAlpha = (Alphamax-Alphamin).*rand(1,1) + Alphamin;
%%% Max uncertainty
% deltaD=Dmax;
% deltaK=Kmax*0.01;
% deltaAlpha=Alphamax;

deltaD=0;
deltaK=0;
deltaAlpha=0;
% deltaD=Dmax;
deltaK=Kmax*0.0;
% deltaAlpha=Alphamax;
%%%% disturbance
tau_d=zeros(length(timeArray),1);
tau_d(10/Ts:end,1)=0.01;
disturb=zeros(length(timeArray),1);
disturb_est=zeros(length(timeArray),1);
%%%%%% ndob
max_disturbance_est_err=20;
ndob_g2=[0;1];
ndob_lx=[0,1];
smc_eta=(1+ndob_lx*ndob_g2)*max_disturbance_est_err;
x2_filter=zeros(3,1);
ndob_p=zeros(length(timeArray),1);
ndob_dp=zeros(length(timeArray),1);
ndob_d_est=zeros(length(timeArray),1);
for i=1:length(timeArray)-1
    e(i,1)=xd(i,1)-x1(i,1);
    %%velocity filter
    if i ==1
        x2_filter(end)=x2(i,1);
    else
        x2_filter=circshift(x2_filter,-1);
        x2_filter(end)=x2(i,1);
        
    end
    x2_filt(i,1)=mean(x2_filter);
    de(i,1)=dxd(i,1)-x2_filt(i,1);
    %%% Variable caculation
    theta=x1(i,1);
    dtheta=x2_filt(i,1);
    Izz=m0*r0^2;
    M=Izz/4 + m0*((cos(theta/2)*(r0 - L/theta))/2 +...
        (L*sin(theta/2))/theta^2)^2 + (m0*sin(theta/2)^2*(r0 - L/theta)^2)/4;
    C_simp=-(L*dtheta*m0*(2*sin(theta/2) - theta*cos(theta/2))*(2*L*sin(theta/2)...
        - L*theta*cos(theta/2) + r0*theta^2*cos(theta/2)))/(2*theta^5);
    G_simp=-(g*m0*(L*sin(theta) + r0*theta^2*cos(theta) - L*theta*cos(theta)))/(2*theta^2);
    f1=-M\(k*x1(i,1) +(b+C_simp)*x2_filt(i,1)+ G_simp);
    b_x=alpha/M;
    ndob_d_est(i,1)=ndob_p(i,1)+ndob_lx*[x1(i,1);x2_filt(i,1)];
    disturb_est(i,1)=ndob_d_est(i,1)*M;
    
    smc_s=de(i,1)+smc_lambda*e(i,1)+ndob_d_est(i,1);
%     max_disturbance_est_err=Km*(x1(i,1)/M);
    smc_eta=(1+ndob_lx*ndob_g2)*max_disturbance_est_err;
    if smc_s > smc_epsilon
        smc_s_sat=smc_s/abs(smc_s);
    else
        smc_s_sat=smc_s/smc_epsilon;
    end
    u(i,1)=1/b_x*(ddxd(i,1)-f1+smc_lambda*de(i,1)+smc_eta*smc_s_sat+ndob_d_est(i,1));
    disturb(i,1)=(deltaK*x1(i,1)+deltaD*x2_filt(i,1)+deltaAlpha*alpha*u(i,1)+tau_d(i,1));
    pd1_MPa(i,1)=(u(i,1)-(0.5*sin(phi)+0.5*sqrt(3)*cos(phi))*pm2_MPa+sin(phi)*pm3_MPa)/(0.5*sin(phi)-0.5*sqrt(3)*cos(phi));
    ndob_f=[x2_filt(i,1);f1];
    ndob_g1=[0;b_x];
    %%% Update px lx
    %%% Update estimated torque
    dx1(i,1)=x2(i,1);
    dx2(i,1)=f1+b_x*u(i,1)+(deltaK*x1(i,1)+deltaD*x2_filt(i,1)+deltaAlpha*alpha*u(i,1)+tau_d(i,1))/M;
    ndob_dp(i,1)=-ndob_lx*ndob_g2*ndob_p(i,1)-ndob_lx*(ndob_g2*ndob_lx*[x1(i,1);x2_filt(i,1)]+ndob_f+ndob_g1*u(i,1));
    x1(i+1,1)=x1(i,1)+dx1(i,1)*Ts;
    x2(i+1,1)=x2(i,1)+dx2(i,1)*Ts;
    ndob_p(i+1,1)=ndob_p(i,1)+ndob_dp(i,1)*Ts;

end
% disturb=(Kmax*x1+Dmax*x2_filt+Alphamax*alpha*u+tau_d);
%% Result compare
fp=figure('Name','smcAndilc','Position',[100,100,600,400]);
subplot(4,1,1)
plot(timeArray(1:end),xd(1:end),'r','LineWidth',2)
hold on
plot(timeArray(1:end),x1(1:end),'b')
legend('ref','x1')
% ylim([-5,5])
title(['Unbonded Control Signal with'...
    ' \Delta k =',num2str(deltaK),' \Delta d =',num2str(deltaD),' \Delta \alpha=',num2str(deltaAlpha)])
ylabel('Angle(rad)')
hold on
subplot(4,1,2)
plot(timeArray(1:end),x2_filt(1:end),'k','LineWidth',2)
hold on
plot(timeArray(1:end),x2(1:end),'b')
legend('x2_{filt}','x2')
% ylim([-5,5])
title(['x2'])
ylabel('Anguler Vel.(rad/s)')
hold on
subplot(4,1,3)
plot(timeArray(1:end),pd1_MPa(1:end)*145.038,'r')
hold on
title(['Control Signal u'])
ylabel('Pd(Psi)')
xlabel('time')
subplot(4,1,4)
plot(timeArray(1:end),disturb_est(1:end),'r','LineWidth',2)
hold on
plot(timeArray(1:end),disturb(1:end),'b')
legend('\Psi','\Psi_{est}')
title(['Lumped Dist. Estimation'])
ylabel('Torque(N\cdotm)')
xlabel('time')
end