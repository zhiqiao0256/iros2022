%Sample code for configuration (bending) code for the silicone robotic arm
%Azadeh 01_26_2021
T=5; %Time of run [s]
dt=0.01; %time steps [s]
STEPS = T/dt;
number_of_segments = 1;
%% vectors directions in 3D
e1 = [ 1 ; 0 ; 0 ];
e2 = [ 0 ; 1 ; 0 ];
e3 = [ 0 ; 0 ; 1 ];
%% specificaiton
L0 = 0.185*number_of_segments; % length of the segment [m]
r0 = 0.075*sqrt(2)/2;  % radius at the cross-section [m]
da = 0.0343; %distance of each actuator tube to the backbone [m]
r1 = da*(cos(pi/4))*(e1+e2);  %actuator 1 distance vector to backbone [m]
r2 = da*(sqrt(2)/2)*(e1-e2); %actuator 2 distance vector to backbone [m]
r3 = da*(sqrt(2)/2)*(-e1-e2); %actuator 3 distance vector to backbone [m]
r4 = da*(sqrt(2)/2)*(-e1+e2); %actuator 4 distance vector to backbone [m]
db = L0*e3; %distance of the tip force (FSR sensor) to the center of mass vector [m]
A0 = pi*r0^2; % cicular cross-sectional area [m^2]
rho = 792; % density (kg/m^3)
M = rho*A0*J; %material cross-section stiffness per unit length [Nm]
E = 0.27*10^6; % Young modulus [Pa]
G = E/(2*(1 + 0.4));   % Shear modulus [Pa]
Ixx = (pi/4)*r0^4;  % second moment of inertai for circular cross-section  %rectangular: (height*width^3)/12;
Iyy = (pi/4)*r0^4;  % second moment of inertai for circular cross-section circular   %rectangular: (width*height^3)/12;
Izz=Ixx+Iyy;
J = diag([Ixx  Iyy  Izz]); % Second mass moment of inertia tensor
Kb = diag([E*J(1,1), E*J(2,2), G*J(3,3)]); % Stiffness matrix for bending(d1,d2) and torsion (d3) [N.m^2]
% Ke = diag([G*A0, G*A0, E*A0]); % Stiffness matrix for shear (d1,d2) and extension(d3) [N]
tau = 0; %time period of free-end damping in x-y-z axes [s]
Bb = Kb* diag([tau, tau, tau]); % Damping matrix corresponding to bending and extension [N.m^2.s]
Force_tip = -e1; %zeros(1,3); %Force from interaction with environment (FSR) [N]
Moment_tip = cross(db,Force_tip); %Moment from interaction with the environment (FSR) [Nm]
g = -9.81*e3 ;  %gravity accelaration [m/s^2]
Force_gravity = rho*A0*g; %force per unit length at each cross-section [N/m]
% u =[ux;uy;uz]; The curvature vector describing bending along x-y axes [1/m]
u_star = [0;0;0]; %undeformed state of u [1/m]
% v_star = [0;0;1]; %undeformed state of v [-]
a = 1;  %amplitude of bending limited by the design and worksapce range=(0,6) [1/m]
freq = 1; %frequency of bending limited by air pressure speed [Hz] range=
w = 2*pi*freq;  
%% The current code is written for one segment, based on the backbone passsing through each cross section
for i=1:STEPS

    t(i) = (i-1)*dt;

    u_desired = [0; a*sin(w*t(i)); 0]; %desired curvature
    ut_desired = [0; a*w*cos(w*t(i)); 0]; %first derivative of the desired curvature
    utt_desired = [0; -a*w^2*sin(w*t(i)); 0]; %second derivative of the desired curvature

% position_0 = [x0,y0,z0]= [0;0;0]; % cartesian coordinates of the fixed end (proximal point) [m] 
% position(i) = [x(i);y(i);z(i)]= reading from camera; %cartesion coordinates of tip of the segment [m]
% x in direction of bending
% y orthogonal to x
% z in direction of the arm towards up

    R0 = eye(3);
    % R = %Rotation of the segment with respect to the global frame

    u_y = 2*(x(i)-x0)/((x(i)-x0)^2+(z(i)-z0)^2);  %bending curvature along x-axis
    u = [0; u_y; 0];  %bending curvature vector
    theta = asin(u_y*(z(i)-z0));
    x_c = (1-cos(theta/2)/u_y);
    Moment_gravity = cross((x_c-x0)*e1,Force_gravity);  %moment per unit length at each cross-section [N/m]
%     xt(i) = (x-x_previous_step)/dt;
%     yt(i) = (y-y_previous_step)/dt;
%     zt(i) = (z-z_previous_step)/dt;
%     ut_y=(2*xt*((z(i)-z0)^2-(x(i)-x0)^2)-4*(x(i)-x0)*(z(i)-z0)*zt)/((x(i)-x0)^2+(z(i)-z0)^2)^2; %first derivative of u_y
    
    torq_l = Kb*(u-u_star); %+Bb*ut; %torque along x-axis on the backbone
    K=Kb;  %Proportional coefficient matrix
%     D= Bb; %Derivative coefficient matrix
    torq_l_desired = K*(u_desired-u_star); %+D*ut_desired; %desired torque along x-axis on the backbone

    applied_torque_by_actuator= R*(torq_l_desired-torq_l+M*utt_desired-Moment_tip-Moment_gravity); %[Nm], 3x1 vector
    applied_force_by_actuator_1= abs(applied_torque_by_actuator(2))/(2*da*cos(pi/4));  %[N], scalar
    applied_force_by_actuator_2= abs(applied_torque_by_actuator(2))/(2*da*cos(pi/4)); %[N]
    applied_pressure_by_actuator_1= applied_force_by_actuator_1/A0; %[N/m^2]=[Pa]
    applied_pressure_by_actuator_2= applied_force_by_actuator_2/A0; %[N/m^2]=[Pa]
end