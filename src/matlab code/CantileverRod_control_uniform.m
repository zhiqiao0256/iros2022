%% Inverse Dynamic Control of Cantilever Rod using Cosserat-Rod model (Actuation Forces + Gravity Force + External Forces)
%Azadeh version created 03/04/22
%Bending for grasping-Silicone
%I want the exp in-out being validated by simulation
%I need the position and rotation values be fed in to the code and the
%required pressure for each segment to be sent out to the experiment
%low-level controller code
%visialization: Silicone based soft robotic arm

%original Cosserat Code from Paper John Till, Caleb Rucker 2018
% Real-Time Dynamics of Soft and Continuum Robots based on Cosserat-Rod Models

function output= CantileverRod_control_uniform
%% Parameters - Section 2.1
global L dL fseg lseg rhoAg ds C q t sim_result exp_result ud vd segment N kapa_out L_out pressure_of_tubes Pressure_from_actuator_L_1 lenv
% N L STEPS 
%% Video recording ()
% myVideo = VideoWriter('grasp'); %open video file
% myVideo.FrameRate = 1;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
%% Time descretization
del_t = 1; % Time step= 0.01 is good ds=5ms
% del_t = 0.0250;
STEPS = 50;
t= zeros(STEPS,1);
%% Space discretezition
scale = 1;
physical_segment = 1;
decretized_sections = 1;
N = physical_segment*decretized_sections;      %40 ; Number of discretization for rod = 4 * number of nodes per segment
L0 = physical_segment*0.185*scale; % Length of rod %L>>r cross-section diameter or sqrt(A)
% L = L0;
ds = L0/N;    % Discretization of curve parameter and step size in RK4
dL = zeros(1,N+1);
%% Main input and outputs
result = zeros(STEPS,25,N+1);
% state_out = zeros(STEPS,6,N+1);
kapa_out = zeros(STEPS,N+1);
L_out = zeros(STEPS,N+1);
% seg_num = zeros(physical_segment,1);
circle = zeros(301,3);  %visualization cylindrical view
p_c = zeros(301,3,N+1);  %the points of the arm on the cylindar serface
%% vectors directions in 3D
e1 = [ 1 ; 0 ; 0 ];
e2 = [ 0 ; 1 ; 0 ];
e3 = [ 0 ; 0 ; 1 ];
%% desired motions-initial
ud = zeros(STEPS,3,N+1);
vd = zeros(STEPS,3,N+1);

u_d = zeros(3,N+1);
ut_d = zeros(3,N+1);
utt_d = zeros(3,N+1); 

v_d = zeros(3,N+1); 
vt_d = zeros(3,N+1); 
vtt_d = zeros(3,N+1); 

kapa = zeros(1,N+1);
L = L0*ones(1,N+1);
R = eye(3);
%% External forces and torques to tip (free-end)- global frame
F_tip = zeros(STEPS,3); %0.001*9.81*[0 ; 0 ; 0 ]; % Force on tip based on gram
M_tip = zeros(STEPS,3); %[ 0 ; 0 ; 0 ]; % Momentum on tip
g = 1*[0 ; 0 ; -9.81] ; % Gravitational force
%% Boundary & Initial Conditions - Section 2.4   global frame
p0 = [ 0 ; 0 ; 0 ]; % Global position in cartesian coordinates
h0 = [ 1 ; 0 ; 0 ; 0 ]; % Quaternions for material orientation
q0 = [ 0 ; 0 ; 0 ]; % Velocity (linear) in the local frame (w.r.t time)
w0 = [ 0 ; 0 ; 0 ]; % Angular velocity in the local frame (w.r.t time)
n0 = [ 0 ; 0 ; 0 ]; % initial global internal force
m0 = [ 0 ; 0 ; 0 ]; % initial global internal moment
vstar = [ 0 ; 0 ; 1 ];    % initial position velocity w.r.t arc length (local frame)
ustar = [ 0 ; 0 ; 0 ];   % initial curvature for a straight rod (rotation derivative w.r.t arc length) (local frame)
%% Dependent Parameter Calculations
rho = 800;   %507kg/m3 % 2330; % 1.3468e3;    % Hydrogel density=1.1 - Beam density=M(3e-3)/V(2.2275e-6)=1.3468e3
C=zeros(3,3);
r0 = 0.075*sqrt(2)/2;  % uniform arm radius (surface to the backbone)
% r1 = 0.03*(cos(pi/4))*(e1+e2);  %actuator 1 distance vector to backbone [m]
% r2 = 0.03*(cos(pi/4))*(e1-e2);  %actuator 2 distance vector to backbone [m]
% r3 = 0.03*(cos(pi/4))*(e1+e2);  %actuator 1 distance vector to backbone [m]
% r4 = 0.03*(cos(pi/4))*(e1-e2);  %actuator 2 distance vector to backbone [m]
r = -0.03*cos(pi/4)*2*e1;    %can be replaced instead of sum of r1+r2 since P1A1=P2A2
%% Cross-section area
A0 = pi*r0^2;  %circular  %rectangular: beam_height*beam_width; % Cross-sectional area
radius_chamber= 0.01; %the pressure chamber radius [m]
A_chamber = pi*(radius_chamber)^2; %[m2]
Pressure_from_actuator_L_1=zeros(STEPS,1);
%% Young modulus of Hydrogel beam, was E=(L=0.045/A=11*4.5e-6)*(2.2061)=2.005e3
Em = 0.27*10^6; %188*10^9;   %0.3*1000*scale^2*(0.0542*scale)/A0;  %E=6000 when L is not considered for rigidity
Gm = Em/(2*(1 + 0.4));   %Shear modulus

Ixx = (pi/4)*r0^4;  %circular   %rectangular: (beam_height*beam_width^3)/12;
Iyy = (pi/4)*r0^4;  %circular   %rectangular: (beam_width*beam_height^3)/12;
Izz=Ixx+Iyy;
J = diag([Ixx  Iyy  Izz]); % Second mass moment of inertia tensor

Kse = diag([Gm*A0, Gm*A0, Em*A0]); % Stiffness matrix for shear(d1,d2) rigidity and extension(d3) or axial rigidity
Kbt = diag([Em*J(1,1), Em*J(2,2), Gm*J(3,3)]); % Stiffness matrix for bending(d1,d2)rigidity and twisting(d3) rigidity

% Bse = zeros(3); % viscous damping matrix for shear & extension
% Bbt = zeros(3); % viscous damping matrix for bending & torsion
tau=0; %2*1.5; %
Bse = Kse* diag([tau, tau, tau]);
Bbt = Kbt* diag([tau, tau, tau]);
%% BDF (backward diffrentiation formula) Coefficients 
%%(an implicit method of time derivative approximation)
%%Discretization in time for PDE (Section 2.2 Eq.(5)-b)
alpha=0;    %BDF2
% alpha=-0.5; %Trapezoidal
% alpha=-0.2; %found to be accurate and stable in Till 2019 paper
%d1 = alpha/(1+alpha); %alpha >> d1
c0 = (1.5+alpha)/(del_t*(1+alpha)); %c0 = 1.5/del_t; 
c1 = -2/del_t;
c2 = (0.5+alpha)/(del_t*(1+alpha)); %c2 = 0.5/del_t; 
%% Expressions extracted from simulation loop
Kse_plus_c0_Bse_inv = (Kse+c0*Bse)^-1; % Eq.(7)-b
Kbt_plus_c0_Bbt_inv = (Kbt+c0*Bbt)^-1; % Eq.(7)-b
Kse_vstar = Kse*vstar; % Eq.(7)-b
Kbt_ustar = Kbt*ustar; % Eq.(7)-b
rhoA = rho*A0; % Eq.(7)-a
rhoJ = rho*J; % % Eq.(1)
rhoAg = rho*A0*g; % Eq.(7)-a
%% Initialize to straight configuration
% y and z are general variables as in Eq.(12) >> Generalized PDE
% y : = [ p ; h ; n ; m ; q ; w ] and z : = [ v ; u ]
y = [zeros(2,N+1); linspace(0,L0,N+1); zeros(16,N+1)] ;
z = [zeros(2,N+1); ones(1,N+1); zeros(3,N+1)];
y_prev = y;
z_prev = z;
%% Main Simulation Loop - Section 2.4
G = zeros(6,1) ; % Shooting method initial guess
% y_exp = zeros(STEPS,3,N+1);
% load('y_exp.mat');
% t_tcp = tcpclient("10.220.3.5",80);
recv = tcpserver("10.203.49.203",55555)
sen = tcpclient("10.203.49.209",6666)
datasize = 14;
x_bias = 0.0015;
R_initial = [ 1.0000   -0.0015    0.0000
              0.0015    1.0000    0.0029
             -0.0000   -0.0029    1.0000];
%%
for i = 1:STEPS
    i
    t(i)= (i-1) * del_t;      
    %% main original code solving the coesserat rod forward dynamics
        mocap_data = read(recv,datasize,"double");
     if size(mocap_data)>0
        position_global = (mocap_data(8:10)-mocap_data(1:3))';
        position(1) = position_global(1,1)-x_bias;
        position(2) = position_global(2,1); 
        position(3) = position_global(3,1);  
        h = mocap_data(11:14)';
        h1 = h(1); h2 = h(2); h3 = h(3); h4 = h(4);
        R = eye(3)+ 2/( h'*h )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2];
        position_local = (R_initial'*R)'*position';
        if abs(position_local(1))<=0.00000001
            kapa(2)=0;
            dL(2)=0;
            L(2) = position_local(3);
        else          
            kapa(2) = -2*position_local(1)/(position_local(1)^2+position_local(3)^2);  %curvature_about_x_axis
%             L = asin(kapa(node+1)*p_l(3))/kapa(node+1); 
            L(2) = (2/kapa(2))*atan2(-position_local(1),position_local(3));
            dL(2) = L(2)-L0;
        end
        y(1:3,2) = [position(1);position(2);position(3)];
        y(4:7,2) = mocap_data(11:14)';
        z(3,2) = 1+dL(2)/L(2);
        z(5,2) = kapa(2);
        
        % Set history terms - Eq.(5)-b
        % y_prev ~ _()^(i-2)y(t)
        yh = c1*y + c2*y_prev;
        zh = c1*z + c2*z_prev;
        %store from previous step for the next step
        y_prev = y;
        z_prev = z;
        
        % Midpoints are linearly interpolated for RK4
        yh_int = 0.5*(yh(:, 1:end-1) + yh(:, 2:end));
        zh_int = 0.5*(zh(:, 1:end-1) + zh(:, 2:end));

        exp_result(i,:,:) = [y;z];    %saving results
        kapa_out(i,:) = kapa;
        L_out(i,:) = L;
        t_c = 0:pi/150:2*pi;

        for counter = 1:length(t_c)
            x_c = r0*cos(t_c(counter));
            y_c = r0*sin(t_c(counter));
            z_c = 0;
            circle(counter,:) = [x_c;y_c;z_c]; 
            p_c(counter,:,2) = (y(1:3,2)+R*circle(counter,:)');
        end
        plotRobot();
       
        %Standard Simple Shooting Method (SSM) using fsolve          
        G = fsolve(@getResidual,G); 
        sim_result(i,:,:) = [y;z]; 
%         plotRobot();
        pressure_of_tubes(i) = 24*0.000145038*Pressure_from_actuator_L_1(i);
        pd_data = pressure_of_tubes(i);
        if pd_data>=20
            pd_data=20;
        elseif pd_data<=0
            pd_data=0;
%         elseif pd_data<0.7 && pd_data>0
%             p0 = [ 0 ; 0 ; 0 ]; % Global position in cartesian coordinates
%             h0 = [ 1 ; 0 ; 0 ; 0 ]; % Quaternions for material orientation
%             q0 = [ 0 ; 0 ; 0 ]; % Velocity (linear) in the local frame (w.r.t time)
%             w0 = [ 0 ; 0 ; 0 ]; % Angular velocity in the local frame (w.r.t time)
%             n0 = [ 0 ; 0 ; 0 ]; % initial global internal force
%             m0 = [ 0 ; 0 ; 0 ]; % initial global internal moment
%         elseif pd_data>=0.7
         
        end
        write(sen,pd_data)
        
     end
end
%% getResidual Function
    function E = getResidual(G)   %takes initial guess and calculate E, give it to next step as a new guess
        %Reaction force and moment are guessed values
        n0 = G(1:3); m0 = G(4:6);
        initial_guess(i,:) = G;
        %Cantilever boundary condition-initial point
        y(:,1) = [p0; h0; n0; m0; q0; w0] ; 
        %Fourth-Order Runge-Kutta Integration 
        %(implicit integrating scheme) semi-discretized model: space not time
        for j = 1:N
            yj = y(:,j) ; %yhj_int = yh_int(:,j); %ythj_int = yth_int(:,j);
            [k1, z(:,j+1), ~] = ODE(yj, yh(:,j), zh(:,j),j);
            [k2, ~, ~] = ODE(yj + k1*ds/2, yh_int(:,j), zh_int(:, j), j);
            [k3, ~, ~] = ODE(yj + k2*ds/2, yh_int(:,j), zh_int(:, j), j);
            [k4, ~, R] = ODE(yj + k3*ds , yh(:,j+1), zh(:,j+1), j);
            %where y is calculated using RK4 spatial integration
            y(:, j+1) = yj + ds*(k1 + 2*(k2+k3) + k4)/6 ;
            %y(:, j+1) = yj + ds*k1; % Euler's Method (explicit time integrating method)
            
            t_c = 0:pi/150:2*pi;

            for counter = 1:length(t_c)
                x_c = r0*cos(t_c(counter));
                y_c = r0*sin(t_c(counter));
                z_c = 0;
                circle(counter,:) = [x_c;y_c;z_c]; 
                p_c(counter,:,j+1) = (y(1:3,j+1)+R*circle(counter,:)');
            end

        end

        nL = y(8:10, N+1); mL = y(11:13 ,N+1);
        F_tip(i,:) = [0;0;0]'+ fseg(i,:,end);   % rhoAg'- (R*C*q.*abs(q))'
        M_tip(i,:) = [0;0;0]'+ lseg(i,:,end);  
        E = [F_tip(i,:)'-nL; M_tip(i,:)'-mL]; 
        free_end(i,:) = E;
    end

%% ODE Function Eq.(12)-b - solving PDE/ODE 
    function [ys, z, R] = ODE(y, yh, zh, node)
        %% discritization and segment relationship based on 4 real segments
        if mod(node,N/physical_segment)==0
            segment = node/(N/physical_segment);
            seg_num(segment,1) = node;
        else
            segment = 1+(node-mod(node,N/physical_segment))/(N/physical_segment);
        end
        %% defining initial input data to the segment for inverse loop coming from previous forward loop
        %at time step i for seg j
        p = y(1:3);
        h = y(4:7); 
        n = y(8:10); 
        m = y(11:13);
        q = y(14:16); 
        w = y(17:19);
        vh = zh(1:3); 
        uh = zh(4:6);
        %Quaternion to Rotation - Eq.(10)- Kinematics - Task space variables
        h1 = h(1); h2 = h(2); h3 = h(3); h4 = h(4);
        R = eye(3)+ 2/( h'*h )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2];
        %% this is actually the main forward dynamic calculation (force/moment to motion)
        %Solved - Eq.(6)- Constitutive Law - Configuration space variables-local frame
        v = Kse_plus_c0_Bse_inv*(R'*n + Kse_vstar - Bse*vh);
        u = Kbt_plus_c0_Bbt_inv*(R'*m + Kbt_ustar - Bbt*uh);
%         u(2) = kapa(node+1);
%         v(3) = 1+dL(node+1)/L0;
        %% the rest...
        z = [v; u];
        %Time Derivatives - Eq.(5)-- BDF-alpha- an implicit method
        %using backward diffrentiation method 
        yt = c0*y + yh; % derivative wrt time
        zt = c0*z + zh; % derivative wrt time
        vt = zt(1:3); % derivative wrt time
        ut = zt(4:6); % derivative wrt time
        qt = yt(14:16); % derivative wrt time
        wt = yt(17:19); % derivative wrt time
        %% The final input to the robotic arm 
        %%these data are the arc length spatial derivatives in order to be used in RK4 integrator
        %Rod State Derivatives - Eq.(7) - Dynamics
        %diff. kinematics calculations for final Cosserat PDE equations
        ps = R*v;
        Rs = R*[0, -u(3), u(2);
                u(3), 0, -u(1);
                -u(2), u(1), 0];  %cross(R,u) or R*uhat;
            
        %% Weight and Square-Law-Drag - Eq.(3)
         fenv = rhoAg - R*C*q.*abs(q); %without water drag effect  
         lenv(i,:) = cross(R*ds*e3,fenv);
        %% when external loads being applied (like from the object being detected by FSR)
%         fext = [0;0;0];   %external force to the segment (constant and global)
%         lext = cross(fext,[0;0;0]);   %external moment to the segment (constant and global)
       %% PD control-shear -part of solving inverse dynamics- global frame- where we close the loop
        grasp();
        ud(i,:,node+1) = u_d;
        vd(i,:,node+1) = v_d;
        fseg(i,:,segment) = [0;0;0];%-fenv; %10*R*(Kse*(v_d-v) + Bse*(vt_d-vt))- fenv;   % %summation of the forces that is required to be applied to the backbbone from the actutors
        direction_of_segment_moment = 2*cross(p,R*e3)+cross(R*r,R*e3); 
        pressure_initial = (0.7/0.000145038);
        lseg(i,:,segment) = 30*R*(Kbt*(u_d-u) + Bbt*(ut_d-ut))-lenv(i,:)'+(pressure_initial*A_chamber*direction_of_segment_moment);   %summation of the moments that is required to be applied to the backbbone from the actutors
        direction_of_segment_force = R*e3;   %*PiAi=fp = fseg    %summation of internal force vectors being applied to the backbone from actuators
        Pressure_from_actuator_F_1(i) = direction_of_segment_force'*fseg(i,:,segment)'/(A_chamber);  
%         direction_of_segment_force = Rs*e3  %*PiAi=fp = fseg
%         Pressure_from_actuator_F_2(i) = direction_of_segment_force'*fseg(i,:,segment)'/(A_chamber);  
%         direction_of_segment_moment = 2*cross(p,R*e3)+cross(R*r,R*e3); %summation of internal moment vectors being applied to the backbone from actuators
        Pressure_from_actuator_L_1(i) = direction_of_segment_moment'*lseg(i,:,segment)'/(A_chamber);
%         direction_of_segment_moment = R*(cross(2*v+cross(u,r),e3)+cross(r,cross(u,e3)))  %*PiAi=lp =lseg
%         Pressure_from_actuator_L_2(i) = direction_of_segment_moment'*lseg(i,:,segment)'/(A_chamber);
        %% this is actually the main inverse dynamic calculation (motion to force/moment)
        % n=R*(Kse*(v-vstar)+Bse*vt) %global frame
        % m=R*(Kbt*(u-ustar)+Bbt*ut) %global frame
%% calculating the input to the segment for the next loop considering all dynamics 
        ns = R*rhoA*(cross(w,q)+qt)  - fseg(i,:,segment)' - fenv; 
        ms = R*(cross(w,rhoJ*w) + rhoJ*wt) - cross(ps,n) - lseg(i,:,segment)'-lenv(i,:)';        
        %compatibility
        qs = vt - cross(u , q) + cross(w, v) ;
        ws = ut - cross(u ,w) ;
        %Quaternion Derivative - Eq.(9)
        hs = [0, -u(1), -u(2), -u(3);
              u(1), 0, u(3), -u(2);
              u(2), -u(3), 0 , u(1);
              u(3), u(2), -u(1), 0]*h/2;
        ys = [ps; hs; ns; ms; qs; ws];   %the segments' tip data at this time step
    end
%% Grasp related input definition, switching conditions based on sensor feedback 
    function grasp()

        T_period = 100;
        frequency = 2*pi/T_period; 

             if segment==1

                 alpha= 4; %6/physical_segment;

                 u_xd =  0; %alpha*(sin(2*pi*frequency*t)); %+pi/2 %angle of changes
                 ut_xd =  0; %alpha*(2*pi*frequency)*cos(2*pi*frequency*t);
                 utt_xd = 0; %-alpha*(2*pi*frequency)^2*sin(2*pi*frequency*t);
        %          
                 u_yd =  alpha*(sin(frequency*t(i))); %exp(alpha*t(i)); %
                 ut_yd = alpha*(frequency)*cos(frequency*t(i)); %alpha*exp(alpha*t(i));
                 utt_yd = -alpha*(frequency)^2*sin(frequency*t(i));%alpha*alpha*exp(alpha*t(i));

                 u_zd =  0; %alpha*sin((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)));
                 ut_zd =  0; %alpha*(2*pi*frequency)*cos((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)))+sin((2*pi)*(frequency*t(i)))*(-2*pi*frequency)*sin((2*pi)*(frequency*t(i)-k*ds/(lambda)));
                 utt_zd = 0; %-alpha*(2*pi*frequency)^2*sin((2*pi)*(frequency*t(i)));
                 
                 v_zd = 1; %1.26+0.26*sin(frequency*t(i)); %/STEPS; 
                 vt_zd = 0; %0.26*frequency*cos(frequency*t(i)); %/STEPS;
                 vtt_zd = 0; %-0.26*(frequency)^2*sin(frequency*t(i)); 

             elseif segment==2

                 alpha= 0.6; %6/physical_segment;

                 u_xd =  0; %alpha*(sin(2*pi*frequency*t)); %+pi/2 %angle of changes
                 ut_xd =  0; %alpha*(2*pi*frequency)*cos(2*pi*frequency*t);
                 utt_xd = 0; %-alpha*(2*pi*frequency)^2*sin(2*pi*frequency*t);
        %          
                 u_yd =  0; %alpha*(sin(2*pi*frequency*t(i))); %exp(alpha*t(i)); %
                 ut_yd = 0; %alpha*(2*pi*frequency)*cos(2*pi*frequency*t(i)); %alpha*exp(alpha*t(i));
                 utt_yd = 0; %-alpha*(2*pi*frequency)^2*sin(2*pi*frequency*t(i));%alpha*alpha*exp(alpha*t(i));

                 u_zd =  0; %alpha*sin((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)));
                 ut_zd =  0; %alpha*(2*pi*frequency)*cos((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)))+sin((2*pi)*(frequency*t(i)))*(-2*pi*frequency)*sin((2*pi)*(frequency*t(i)-k*ds/(lambda)));
                 utt_zd = 0; %-alpha*(2*pi*frequency)^2*sin((2*pi)*(frequency*t(i)));

                 v_zd = 1; %1+0.2*t(i)/STEPS;
                 vt_zd = 0; %0.2/STEPS;
                 vtt_zd = 0; %0;

             elseif segment==3

                 alpha= 0.6;%6/physical_segment;

                 u_xd=  0; %alpha*(sin(2*pi*frequency*t)); %+pi/2 %angle of changes
                 ut_xd =  0; %alpha*(2*pi*frequency)*cos(2*pi*frequency*t);
                 utt_xd = 0; %-alpha*(2*pi*frequency)^2*sin(2*pi*frequency*t);
        %          
                 u_yd =  0; %alpha*(sin(2*pi*frequency*t(i))); %exp(alpha*t(i)); %
                 ut_yd = 0; %alpha*(2*pi*frequency)*cos(2*pi*frequency*t(i)); %alpha*exp(alpha*t(i));
                 utt_yd = 0; %-alpha*(2*pi*frequency)^2*sin(2*pi*frequency*t(i));%alpha*alpha*exp(alpha*t(i));

                 u_zd =  0; %alpha*sin((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)));
                 ut_zd=  0; %alpha*(2*pi*frequency)*cos((2*pi)*(frequency*t(i)))*cos((2*pi)*(frequency*t(i) - k*ds/(lambda)))+sin((2*pi)*(frequency*t(i)))*(-2*pi*frequency)*sin((2*pi)*(frequency*t(i)-k*ds/(lambda)));
                 utt_zd = 0; %-alpha*(2*pi*frequency)^2*sin((2*pi)*(frequency*t(i)));

                 v_zd = 1; %1+0.2*t(i)/STEPS;
                 vt_zd = 0; %0.2/STEPS;
                 vtt_zd =0; %0;
             end
    %% input command for bending (u_x,u_y) and torsion (u_z)-one at an experiment
        u_d = [u_xd;u_yd;u_zd];
        ut_d = [ut_xd;ut_yd;ut_zd];
        utt_d = [utt_xd;utt_yd;utt_zd];
        
        v_d = [0;0;v_zd];  %1.1*t(i)/STEPS
        vt_d = [0;0;vt_zd];
        vtt_d = [0;0;vtt_zd];
    end
%% Visualization of the robotic arm been drawn in real-time 
    function plotRobot()
        
        p_c(:,:,1) = circle(:,:);   %first segment's first point
    %     p_c(:,:,N+1) = p_c(:,:,N);    %last segment's last point
        for j=1:N+1
            p_c1(j,:) = p_c(:,1,j)';
            p_c2(j,:) = p_c(:,2,j)';
            p_c3(j,:) = p_c(:,3,j)';
%         end
%         for j=1:N+1
        
            if j<= N/physical_segment
                
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[1 0 0],'LineWidth',3);
    
            elseif j<= 2*N/physical_segment
                
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[0 0 1],'LineWidth',3);
                 
            elseif j<= 3*N/physical_segment
    
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[1 0 0],'LineWidth',3);
                 
            else
    
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[0 0 1],'LineWidth',3);
                 
            end          
            hold on;
            if mod(j-1,N/physical_segment)==0
                    plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Marker','o','Markersize',4,'MarkerEdgeColor','k'); %'Color',[1 0 0],'LineWidth',1,
            end
        end
    
    %             hold on;
    %             hold off;
                grid on;          
                ax = gca;
                ax.XDir = 'reverse';
                ax.YDir = 'reverse';
                ax.ZDir = 'normal';
                axis([-50 50 -50 50 0  50]) ;
                xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
    %           daspect ([ 2 1 1 ]); 
                view(0,0);
    %             view(15,10);
    %             view(-150,20);
                drawnow;
    %             if i==1
    %                 savefig('shearg1.fig')
    %             elseif i==3
    %                 savefig('shearg2.fig')
    %             elseif i==8
    %                 savefig('shearg3.fig')
    %             elseif i==10
    %                 savefig('shearg4.fig')
    %             end
                hold off;
        %get frame
%         frame = getframe(gcf);
%         writeVideo(myVideo, frame);
    end
%   close(myVideo)
  output= exp_result;
end