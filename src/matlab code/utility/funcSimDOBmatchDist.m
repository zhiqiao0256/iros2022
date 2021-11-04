function par_set=funcSimDOBmatchDist(par_set)
close all
%% Input Output data
testData=par_set.trial1;
%%%%%%

%%%%%% State Ini.
Ts=0.001; % sampling period
timeArray=[0:Ts:60]';%sec
x1=zeros(length(timeArray),1); %state variable theta
x2=zeros(length(timeArray),1); %state variable dtheta
x2_filt=x2;% moving average filter
x2_filter=zeros(3,1);
<<<<<<< HEAD
=======

>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
dx1=zeros(length(timeArray),1); % dot theta
dx2=zeros(length(timeArray),1); % double dot theta
e=zeros(length(timeArray),1); % e = x1-xd
de=zeros(length(timeArray),1);% dot e 
% smc_s=zeros(length(timeArray),1); % s = dot_e + lambda*e



m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=par_set.L;      % segment length


<<<<<<< HEAD
% Input signal
=======
%% Input signal
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
%%% Sine
Amp=deg2rad(10);
Boff=deg2rad(-30);
freq=0.01;%Hz
xd=-Amp*sin(2*pi*freq*timeArray)+Boff;
dxd=-Amp*(2*pi*freq)*cos(2*pi*freq*timeArray);
ddxd=Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
pd1_MPa=zeros(length(timeArray),1);
%%% Step
<<<<<<< HEAD
xd=timeArray*0+Boff;
dxd=timeArray*0;
ddxd=timeArray*0;
=======
% xd=0.0*sin(2*pi*freq*timeArray)+Boff;
% dxd=0.0*(2*pi*freq)*cos(2*pi*freq*timeArray);
% ddxd=0.0*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
% pd1_MPa=zeros(length(timeArray),1);

>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
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
<<<<<<< HEAD
smc_lambda=5; % converge rate to sliding surface
smc_epsilon=1; % bandwith for s
smc_epsilon_2=1; % small positive number for eta

=======
smc_lambda=10;
smc_epsilon=1;
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
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
deltaAlpha = ((Alphamax-Alphamin).*rand(1,1)+Alphamin);
%%% Max uncertainty
deltaD=Dmax*0.05;
deltaK=Kmax*0.0;
deltaAlpha=Alphamax*0;

% deltaD=0;
% deltaK=0;
% deltaAlpha=0;

<<<<<<< HEAD
deltaD=0;
deltaK=0;
deltaAlpha=0;

%%%% disturbance
tau_d=zeros(length(timeArray),1);
tau_d(10/Ts:end,1)=0.0;
disturb=zeros(length(timeArray),1);
disturb_est=zeros(length(timeArray),1);
=======
%%%% disturbance
tau_d=zeros(length(timeArray),1);
tau_d(10/Ts:end,1)=-.0;
lumped_disturb=zeros(length(timeArray),1);
lumped_disturb_est=zeros(length(timeArray),1);
lumped_disturb_torque=zeros(length(timeArray),1);
lumped_disturb_est_torque=zeros(length(timeArray),1);
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
%%%%%% Matched Disturbance estimation
%%%% d_est= z_est + p(s)
%%%% dot_z_est = -dp/dt*()
ndob_z_est=zeros(length(timeArray),1);
ndob_dz_est=zeros(length(timeArray),1);
ndob_d_est=zeros(length(timeArray),1);
%%% Loop Starts
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
<<<<<<< HEAD
    %%% sliding surface
    smc_s=de(i,1)+smc_lambda*e(i,1);
    %%% Sat function
=======
    %%% Update SMC
    smc_s=de(i,1)+smc_lambda*e(i,1);
    smc_eta=0.5*abs(smc_s)+0.1;
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
    if smc_s > smc_epsilon
        smc_s_sat=smc_s/abs(smc_s);
    else
        smc_s_sat=smc_s/smc_epsilon;
    end
<<<<<<< HEAD
    %%% update contorl input u= u_eq + u_n + u_s
    u_eq(i,1)=-1/b_x*(f1+smc_lambda*de(i,1)-ddxd(i,1));
    u_n(i,1)= - 1/b_x*ndob_d_est(i,1);
    smc_eta=0.5*abs(smc_s)+smc_epsilon_2;
    u_s(i,1)=-1/b_x*smc_eta*smc_s_sat;
    u(i,1)= u_eq(i,1) + u_n(i,1) + u_s(i,1);
    %%%
    lumped_dist(i,1)=1/M*(deltaK*x1(i,1)+deltaD*x2_filt(i,1)+deltaAlpha*alpha*u(i,1)+tau_d(i,1));
    lumped_torque(i,1)=lumped_dist(i,1)*M;
    ndob_d_torque(i,1)=ndob_d_est(i,1)*M;
    %%% Update estimated torque
    dx1(i,1)=x2_filt(i,1);
    dx2(i,1)=f1+b_x*u(i,1)+lumped_dist(i,1);
    x1(i+1,1)=x1(i,1)+dx1(i,1)*Ts;
    x2(i+1,1)=x2(i,1)+dx2(i,1)*Ts;
    %%% Update DOB
    ndob_p_of_s=smc_s;
    ndob_dp_ds=1;
    ndob_dz_est(i,1)=-ndob_dp_ds*(b_x*(u_n(i,1)+u_s(i,1))+ndob_d_est(i,1));
    ndob_z_est(i+1,1)= ndob_z_est(i,1) + ndob_dz_est(i,1)*Ts;

    ndob_d_est(i+1,1)=ndob_z_est(i,1)+ ndob_p_of_s;

=======
    u_eq(i,1)=-1/b_x*(f1+smc_lambda*de(i,1)-ddxd(i,1));
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
    ndob_p_of_s=35*smc_s;
    ndob_dpds=35;
    ndob_dz_est(i,1)=-ndob_dpds*(b_x*(u_n(i,1)+u_s(i,1))+ndob_d_est(i,1)); 
    ndob_z_est(i+1,1)=ndob_dz_est(i,1)*Ts+ndob_z_est(i,1);
    ndob_d_est=ndob_z_est+ndob_p_of_s;
    ndob_d_est_torque(i,1)=ndob_z_est(i,1)*M;
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
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
<<<<<<< HEAD
plot(timeArray(2:end),u(1:end)*145.038,'r')
hold on
title(['Control Signal u'])
ylabel('torque(N\cdotm)')
xlabel('time')
subplot(4,1,4)
plot(timeArray(2:end),ndob_d_torque(1:end),'b','LineWidth',2)
hold on
plot(timeArray(2:end),lumped_torque(1:end),'r')
legend('\Psi_{est}','\Psi_{ref}')
title(['Lumped Dist. Estimation'])
ylabel('torque (N\cdot m)')
=======
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
>>>>>>> 0d0a90107b4326b608daf6a6952f957ad306a178
xlabel('time')
end