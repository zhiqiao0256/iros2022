function par_set=funcSimSMC_inputUncertain_saturation(par_set)
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
Amp=deg2rad(10);
Boff=deg2rad(-40);
freq=0.1;%Hz
xd=-Amp*sin(2*pi*freq*timeArray)+Boff;
ddxd=Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
%%
% xd=deg2rad(-40)*ones(length(timeArray),1);
% ddxd=0*sin(timeArray);
%%%% Initial values
x1(2)=testData.theta_rad(2);x2(1)=0;
e(1)=xd(1)-x1(1);
de(1)=0;
uUnbound(1)=0;
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
lambda=1;
eta=10;
epsi=1;
saturationBound=deg2rad(1);
Km=par_set.maxK-k;
Dm= par_set.maxB-b;
Alpham= (par_set.maxAlpha-alpha)/alpha;
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
deltaD=Dmax;
deltaK=Kmax;
deltaAlpha=Alphamax;
% Tm=max(abs((par_set.maxAlpha-alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
%         -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2))));
uMax= max(abs((alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
    -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2))));

p1_max_psi=40;
% p1_min_psi=1;
p1_max_MPa=p1_max_psi/145.038;
uMax= max(abs((alpha)*(sin(phi).*(0.5*p1_max_MPa+0.5*pm2-pm3)...
    -sqrt(3)*cos(phi).*(0.5*p1_max_MPa-0.5*pm2))));


uMin=0.0;
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
    %%%% Update input u
    u_alpha(i,1)=M*(lambda*de(i,1)+ddxd(i,1) - f+ eta*sign(s(i,1))+sign(s(i,1))*(Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)));
    %%%% Update input u with saturation bound
%     sat=0;
%     if abs(s(i,1)/saturationBound)<=1
%         sat=s(i,1)/saturationBound;
%     else
%         sat=sign(s(i,1));
%     end
%      u_alpha(i,1)=M*(lambda*de(i,1)+ddxd(i,1) - f+ eta*sat+sat*(Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)));
     
%     %%%% Update input u with input uncetainty
%     B_star=Alphamax*abs(1/M);
%     eta=1/(1+B_star)*(B_star*abs(lambda*de(i,1)+ddxd(i,1) - f)+Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)+epsi);
%     u_alpha(i,1)=M/alpha*(lambda*de(i,1)+ddxd(i,1) - f+ eta*sign(s(i,1)));
    %%%% Update input u with input uncetainty
    if abs(s(i,1)/saturationBound)<=1
        sat=s(i,1)/saturationBound;
    else
        sat=sign(s(i,1));
    end
    B_star=Alphamax*abs(1/M);
    eta=1/(1+B_star)*(B_star*abs(lambda*de(i,1)+ddxd(i,1) - f)+Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)+epsi);
    u_alpha(i,1)=M/alpha*(lambda*de(i,1)+ddxd(i,1) - f+ eta*sat);
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
%% Result compare
figure(1)
subplot(2,1,1)
plot(timeArray(2:end),xd(2:end),'r')
hold on
plot(timeArray(2:end),x1(2:end),'b')
legend('ref','x1')
ylim([-5,5])
title(['Unbonded Input with \lambda =',num2str(lambda),'\epsilon=',num2str(epsi),...
    '\Delta k =',num2str(deltaK),'\Delta d =',num2str(deltaD),'\Delta \alpha =',num2str(deltaAlpha)])
ylabel('Angle(rad)')
hold on
subplot(2,1,2)
plot(timeArray(2:end),uBound(2:end),'r')
title(['Input Signal u = (\psi)p_{3x1}, u_{max}=',num2str(uMax)])
ylabel('Torque(N\cdotm)')
xlabel('time')

figure(2)
subplot(2,1,1)
plot(timeArray(2:end),xd(2:end),'r')
hold on
plot(timeArray(2:end),x1(2:end),'b')
legend('ref','x1')
ylim([-5,5])
title(['Bonded Input with \lambda =',num2str(lambda),', \eta =',num2str(eta),...
    '\Delta k =',num2str(deltaK),'\Delta d =',num2str(deltaD)])
ylabel('Angle(rad)')
hold on
subplot(2,1,2)
plot(timeArray(2:end),uBound(2:end),'r')
title(['Input Signal u = \alphaR(\psi)p_{3x1}, u_{max}=',num2str(uMax),'Ts=',num2str(Ts)])
ylim([-0.5,0.5])
ylabel('Torque(N\cdotm)')
xlabel('time')
end