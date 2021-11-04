function par_set=funcSimSMC(par_set)
%% Input Output data
testData=par_set.trial1;
timeArray=testData.pm_psi(:,1);%sec
x1=zeros(length(timeArray),1);
x2=zeros(length(timeArray),1);
dx1=zeros(length(timeArray),1);
dx2=zeros(length(timeArray),1);
uUnbound=zeros(length(timeArray),1);
uBound=zeros(length(timeArray),1);
m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=par_set.L;      % segment length
%%% Input signal
% Amp=deg2rad(10);
% Boff=deg2rad(-40);
% freq=0.05;%Hz
% xd=Amp*sin(2*pi*freq*timeArray)+Boff;
% ddxd=-Amp*(2*pi*freq)^2*sin(2*pi*freq*timeArray);
%%
xd=deg2rad(-40)*ones(length(timeArray),1);
ddxd=0*sin(timeArray);
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
lambda=10;
eta=1.0;
Km=par_set.maxK;
Dm=par_set.maxB;
Tm=max(abs((par_set.maxAlpha-alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
        -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2))));
uMax= max((alpha)*(sin(phi).*(0.5*pm1+0.5*pm2-pm3)...
        -sqrt(3)*cos(phi).*(0.5*pm1-0.5*pm2)));
for i=2:length(timeArray)-1
    %% Sliding Mode controller
    e(i,1)=xd(i,1)-x1(i,1);
    de(i,1)=e(i)-e(i-1)/timeArray(i,1)-timeArray(i-1,1);
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
    %%%% Update input u
    f=M\(-k*x1(i,1) -(b+C_simp)*x2(i,1)- G_simp);
    uUnbound(i,1)=lambda*e(i,1)+ddxd(i,1)-f+eta*sign(s(i,1))+...
        sign(s(i,1))*(Km*abs(x1(i,1)/M)+Dm*abs(x2(i,1)/M)+Tm*abs(1/M));
    %%% Input Signal bound
    if uUnbound(i,1)>=uMax
        uBound(i,1)=uMax;
    else
        uBound(i,1)=uUnbound(i,1);
    end
    %% State feedback controller
    dx1(i,1)=x2(i,1);
    dx2(i,1)=f+uUnbound(i,1);
    %% Update State Variable
    x1(i+1,1)=x1(i,1)+dx1(i,1)*par_set.Ts;
    x2(i+1,1)=x2(i,1)+dx2(i,1)*par_set.Ts;
end


%% Result compare
end