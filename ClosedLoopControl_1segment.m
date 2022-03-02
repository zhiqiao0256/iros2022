%Sample code for configuration (bending) code for the silicone robotic arm
%Azadeh 02_01_2021
T=60; %Time of run [s]
dt=0.0250; %time steps [s]
STEPS = T/dt;
number_of_segments = 1;
%% vectors directions in 3D
e1 = [ 1 ; 0 ; 0 ];
e2 = [ 0 ; 1 ; 0 ];
e3 = [ 0 ; 0 ; 1 ];
%% specificaiton
L0 = 0.185*number_of_segments; % length of the segment [m]
r0 = 0.075*sqrt(2)/2;  % radius at the cross-section [m]
da = 0.0343; %distance of each actuator tube to the backbone [m] 0.0275 measured
A0 = pi*r0^2; % cicular cross-sectional area [m^2]
rho = 792; % density (kg/m^3)
E = 0.28*10^6; % Young modulus [Pa]
G = E/(2*(1 + 0.4));   % Shear modulus [Pa]
Ixx = (pi/4)*r0^4;  % second moment of inertai for circular cross-section  %rectangular: (height*width^3)/12;
Iyy = (pi/4)*r0^4;  % second moment of inertai for circular cross-section circular   %rectangular: (width*height^3)/12;
Izz=Ixx+Iyy;
J = diag([Ixx  Iyy  Izz]); % Second mass moment of inertia tensor
Kb = diag([E*J(1,1), E*J(2,2), G*J(3,3)]); % Stiffness matrix for bending(d1,d2) and torsion (d3) [N.m^2]
force_tip = 0*-e1; %zeros(1,3); %Force from interaction with environment (FSR)-local frame [N]
g = -9.81*e3 ;  %gravity accelaration [m/s^2]
Force_gravity = rho*A0*L0*g; %force per unit length at cross-section [N/m]
% u =[ux;uy;uz]; The curvature vector describing bending along x-y axes [1/m]
% u_star = [0;0;0]; %undeformed state of u [1/m]
% v_star = [0;0;1]; %undeformed state of v [-]
a = 15;  %amplitude of bending limited by the design and worksapce range=(0,6) [1/m]
freq = 0.05; %frequency of bending limited by air pressure speed [Hz] range=
w = 2*pi*freq;  
%% The current code is written for one segment, based on the backbone passsing through each cross section
for i=1:STEPS

    t(i) = (i-1)*dt;

    u_desired = [0; a*sin(w*t(i)); 0]; %desired curvature
    ut_desired = [0; a*w*cos(w*t(i)); 0]; %first derivative of the desired curvature
    utt_desired = [0; -a*w^2*sin(w*t(i)); 0]; %second derivative of the desired curvature

% position_0 = [x0,y0,z0] % cartesian coordinates of the fixed-end [m] at all times
% position(i) = [x(i);y(i);z(i)]= reading from camera; %cartesion
% coordinates of tip of the segment [m] w.r.t. the base of the segment
% x in direction of bending
% y orthogonal to x
% z in direction of the arm towards up

    R0 = eye(3);
    % R = %Rotation of the segment with respect to the base frame 
    %curvature variable in PCC derivation
    kapa = 2*(x(i)-x0)/((x(i)-x0)^2+(z(i)-z0)^2);  %bending curvature along x-axis
    theta = asin(u_y*(z(i)-z0));

    %Common curvetuare variable between PCC and Cosserat model
    kapa_0 = 2*x0/(x0^2+z0^2);
    u_star = [0; kapa_0; 0];  %bending curvature vector 
    u(i) = [0; kapa; 0];  %bending curvature vector
    
    %center of mass location over time
    x0_c = x0;
    y0_c = y0;
    z0_c = z0+L0/2;
    x_c = (1-cos(theta/2))/kapa;
    y_c = 0;
    z_c = sin(theta/2)/kapa;

    Moment_gravity = cross([x_c;y_c;z_c]-[x0_c;y0_c;z0_c],Force_gravity);  %moment per unit length at center [N/m]
    Moment_tip = cross([x(i);y(i);z(i)]-[x0;y0;z0],R*force_tip); %Moment from interaction with the environment (FSR) [Nm]

    K=Kb;  %Proportional coefficient matrix

    applied_torque_by_actuator= R*K*(u_desired-u)+Moment_tip+Moment_gravity; %[Nm], 3x1 vector
%     applied_torque_by_actuator= R*(torq_l_desired-torq_l+M*utt_desired)-Moment_tip-Moment_gravity; %[Nm], 3x1 vector
    applied_force_by_actuator_1= abs(applied_torque_by_actuator(2))/(2*da*cos(pi/4));  %[N], scalar
%     applied_force_by_actuator_2= abs(applied_torque_by_actuator(2))/(2*da*cos(pi/4)); %[N], scalar
    applied_pressure_by_actuator_1= applied_force_by_actuator_1/A0; %[N/m^2]=[Pa]
%     applied_pressure_by_actuator_2= applied_force_by_actuator_2/A0; %[N/m^2]=[Pa]
end