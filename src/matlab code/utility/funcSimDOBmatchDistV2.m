function par_set=funcSimDOBmatchDistV2(par_set)
close all
%% Input Output data
testData=par_set.trial1;
%%%%%%

%%%%%% State Ini.
Ts=0.01; % sampling period
timeArray=[0:Ts:120]';%sec
trail_duriation=15.0;

x1=zeros(length(timeArray),1); %state variable theta
x2=zeros(length(timeArray),1); %state variable dtheta
x2_filt=x2;% moving average filter
x2_filter=zeros(3,1);

dx1=zeros(length(timeArray),1); % dot theta
dx2=zeros(length(timeArray),1); % double dot theta
e=zeros(length(timeArray),1); % e = x1-xd
de=zeros(length(timeArray),1);% dot e 
% smc_s=zeros(length(timeArray),1); % s = dot_e + lambda*e



m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=par_set.L;      % segment length


%% Input signal
%%% Sine
% Amp=deg2rad(10);
% Boff=deg2rad(-40);
% freq=0.01;%Hz
% xd=-Amp*sin(2*pi*freq*timeArray)+Boff;
% dxd=-Amp*(2*pi*freq)*cos(2*pi*freq*timeArray);
% ddxd=Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
% pd1_MPa=zeros(length(timeArray),1);
% %%% Step
% xd=0.0*sin(2*pi*freq*timeArray)+Boff;
% dxd=0.0*(2*pi*freq)*cos(2*pi*freq*timeArray);
% ddxd=0.0*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
% pd1_MPa=zeros(length(timeArray),1);
%%% Ramp from x1
rampAmpAbs=deg2rad(40);
rampRateAbs=deg2rad(2);
t_flatTime=5.0;
t_ramp=[0:Ts:rampAmpAbs/rampRateAbs*2+t_flatTime];
xd = zeros(length(t_ramp),1);
for i =1:length(t_ramp)
    t_i=t_ramp(i);
    if t_i <= rampAmpAbs/rampRateAbs
        xd(i,1)= - rampRateAbs * t_i;
        dxd(i,1)= - rampRateAbs;
        ddxd(i,1)=0;
    elseif  rampAmpAbs/rampRateAbs<= t_i && t_i <= rampAmpAbs/rampRateAbs*+t_flatTime
        xd(i,1)= - rampAmpAbs;
        dxd(i,1)= 0;
        ddxd(i,1)=0;
    else
        xd(i,1)= - rampAmpAbs + rampRateAbs * (t_i-(rampAmpAbs/rampRateAbs*+t_flatTime));
        dxd(i,1)= rampRateAbs;
        ddxd(i,1)=0;
    end
end
%%%% Initial values
x1(1)=testData.theta_rad(end-1);
r0=mean(testData.beta);
phi=mean(testData.phi_rad);
phi=mean(testData.phi_rad);
pd1_upperlimit_MPa=40*0.00689476;
pd1_lowerlimit_MPa=1*0.00689476;
pm2_MPa=1*0.00689476;
pm3_MPa=1*0.00689476;
u_raw=zeros(length(timeArray),1); %unbounded input
u_bound=zeros(length(timeArray),1);   %bounded input

%%%% Mean value of alpha k b
alpha=par_set.meanAlpha;
k=par_set.meanK;
b=par_set.meanB;

%%% smc tunning parameters
smc_lambda=10.0;
smc_epsilon=1;
smc_k=30.0;
smc_eta=40;
ndob_k_p=10;
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
deltaD=Dmax*0.2;
deltaK=Kmax*0.;
deltaAlpha=Alphamax*0.;

% deltaD=0;
% deltaK=0;
% deltaAlpha=0;

%%%% disturbance
tau_d=zeros(length(timeArray),1);
tau_d(10/Ts:end,1)=-.0;
lumped_disturb=zeros(length(timeArray),1);
lumped_disturb_est=zeros(length(timeArray),1);
lumped_disturb_torque=zeros(length(timeArray),1);
lumped_disturb_est_torque=zeros(length(timeArray),1);
%%%%%% Matched Disturbance estimation
%%%% d_est= z_est + p(s)
%%%% dot_z_est = -dp/dt*()
ndob_z_est=zeros(length(timeArray),1);
ndob_dz_est=zeros(length(timeArray),1);
ndob_d_est=zeros(length(timeArray),1);
for i=1:length(timeArray)-1
    e(i,1)=x1(i,1)-xd(i,1);
    %%velocity filter
    if i ==1
        x2_filter(end)=x2(i,1);
    else
        x2_filter=circshift(x2_filter,-1);
        x2_filter(end)=x2(i,1);
        
    end
    x2_filt(i,1)=mean(x2_filter);
    de(i,1)=x2_filt(i,1)-dxd(i,1);
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
    %%% Update SMC
    smc_s=de(i,1)+smc_lambda*e(i,1);
%     smc_eta=0.5*abs(smc_s)+0.1;
    if smc_s > smc_epsilon
        smc_s_sat=smc_s/abs(smc_s);
    else
        smc_s_sat=smc_s/smc_epsilon;
    end
    u_eq(i,1)=-1/b_x*(f1+smc_lambda*de(i,1)-ddxd(i,1)+smc_k*smc_s);
    u_n(i,1)=-1/b_x*ndob_d_est(i,1);
    u_s(i,1)=-1/b_x*smc_eta*smc_s_sat;
    u(i,1)=u_eq(i,1)+u_n(i,1)+u_s(i,1);
    lumped_disturb(i,1)=M\(deltaK*x1(i,1)+deltaD*x2_filt(i,1)+alpha*deltaAlpha*u(i,1)+tau_d(i,1));
    lumped_disturb_torque(i,1)=(deltaK*x1(i,1)+deltaD*x2_filt(i,1)+alpha*deltaAlpha*u(i,1)+tau_d(i,1));
    %%% Update estimated torque
    dx1(i,1)=x2(i,1);
    dx2(i,1)=f1+b_x*u(i,1)+lumped_disturb(i,1);
    x1(i+1,1)=x1(i,1)+dx1(i,1)*Ts;
    x2(i+1,1)=x2(i,1)+dx2(i,1)*Ts;
    %%%% Update Observer
    ndob_p_of_s=ndob_k_p*smc_s;
    ndob_dpds=ndob_k_p;
    ndob_dz_est(i,1)=-ndob_dpds*(b_x*(u_n(i,1)+u_s(i,1))+ndob_d_est(i,1)); 
    ndob_z_est(i+1,1)=ndob_dz_est(i,1)*Ts+ndob_z_est(i,1);
    ndob_d_est=ndob_z_est+ndob_p_of_s;
    ndob_d_est_torque(i,1)=ndob_z_est(i,1)*M;
end
% disturb=(Kmax*x1+Dmax*x2_filt+Alphamax*alpha*u+tau_d);
%% Result compare
fp=figure('Name','smcMathcedDist','Position',[500,100,600,600]);
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
plot(timeArray(1:end-1),u(1:end),'r')
hold on
title(['Control Signal u'])
ylabel('torque(N\cdot m)')
xlabel('time')
subplot(4,1,4)
plot(timeArray(1:end-1),ndob_d_est_torque(1:end),'b','LineWidth',2)
hold on
plot(timeArray(1:end-1),lumped_disturb_torque(1:end-1),'r')
legend('\Psi_{est}','\Psi_{ref}')
title(['Lumped Dist. Estimation'])
ylabel('torque(N\cdot m)')
xlabel('time')
end