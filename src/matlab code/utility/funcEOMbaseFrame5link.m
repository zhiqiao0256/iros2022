function par_set =funcEOMbaseFrame5link(par_set)
fprintf( 'EOM... \n' )
% par_set=[];
%% Transformations
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
pi =sym('pi');
b0=sym('b0');
xi = sym('xi', [5 1]);
%q1=phi_i, q2=theta_i/2 - zeta_theta_i, q3 = zeta_theta_i,q4=bi
dxi = sym('dxi', [5 1]);

    rigid_a=zeros(1,5);
    rigid_alpha=[-pi/2 pi/2 0      -pi/2    pi/2];
    rigid_d=    [0     0    xi(3)   xi(4)    0 ];
    rigid_theta=[xi(1)   xi(2)+pi/2   0        0   xi(5)+pi/2];
 
syms m0 g A
m=[0 0 m0 0 0];
% n=length(q);% DOF
% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic transform matrix
Ti = cell(5+1,1);% z0 z_end_effector
% Ti(1) = {[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1]};
Ti{1}=eye(4);
%     Ti(1)= {[0 0 1 0;
%              1 0 0 0; 
%              0 1 0 0; 
%              0 0 0 1]};
    p_i{1}=Ti{1}(1:3,4);
    z_i{1}=Ti{1}(1:3,3);
for i = 2:5+1     
        % ROTATION FOR X
        A(1,1) = cos((rigid_theta(i-1)));
        A(2,1) = sin((rigid_theta(i-1)));
        A(3,1) = 0;
        A(4,1)=0;
        %ROTATION FOR Y
        A(1,2) = -(sin(rigid_theta(i-1)))*(cosd(rigid_alpha(i-1)))*(1);
        A(2,2) = (cos(rigid_theta(i-1)))*(cosd(rigid_alpha(i-1)));
        A(3,2) = (sind(rigid_alpha(i-1)));
        A(4,2)=0;
        % ROTATION FOR Z
        A(1,3) = (sin(rigid_theta(i-1)))*(sind(rigid_alpha(i-1)));
        A(2,3) = -(cos(rigid_theta(i-1)))*(sind(rigid_alpha(i-1)))*(1);
        A(3,3) = (cosd(rigid_alpha(i-1)));
        A(4,3)=0;
        % TRANSLATION VECTOR
        A(1,4) = (rigid_a(i-1))*(cos(rigid_theta(i-1)));
        A(2,4) = (rigid_a(i-1))*(sin(rigid_theta(i-1)));
        A(3,4) = rigid_d(i-1);
        A(4,4)=1;
        Ti{i} =  Ti{i-1} * A;
    p_i{i}=Ti{i}(1:3,4);
    z_i{i}=Ti{i}(1:3,3);
end
par_set.Ti=Ti;
return
%% Protential energy
E_p=0;
for link_i=1:5
    E_p=E_p+m(link_i)*[0;g;0].'*p_i{link_i+1};
end
%% Linear Velocity
J_v=cell(5,1);
for link_i = 1:5
    j_v=sym(zeros(3,5));
    for j_counter =1:link_i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_v(:,j_counter)=z_i{j_counter};
        else %% Rotational Joint
            j_v(:,j_counter)=cross(z_i{j_counter},(p_i{link_i+1}-p_i{j_counter}));
        end
    end
    J_v{link_i}=j_v;
end
% par_set.Jv=J_v;
%% Angular Velocity
J_w=cell(5,1);
for link_i= 1:5
    j_w=sym(zeros(3,5));
    for j_counter =1:link_i
        if theta(j_counter) == 0 %% Prismatic Joint
            j_w(:,j_counter)=zeros(3,1);
        else %% Rotational Joint
            j_w(:,j_counter)=z_i{j_counter};
        end
    end
    J_w{link_i}=j_w;
end
% par_set.Jw=J_w;
%% Unite Jacobian
par_set.J_xyz=cell(5,1);
for i =1:length(J_w)
    par_set.J_xyz{i}=[J_v{i};J_w{i}];
    
end
par_set.sym_J_xyz2xi=par_set.J_xyz{end};    
%% Inerial and Kinetic energy
syms Ixx Iyy Izz Ixy Ixz Iyz
I=cell(5,1);
% I = [0 0 0 0 I_u 0 0 0 0 0];
for link_i =1:5
    t_Ti=Ti{link_i+1}.';
    if link_i == 3 %%%%%%5/2
%         xyz_i=Ti{link_i+1}(1:3,4).^2;
%         Ixx=m(link_i)*(xyz_i(2)+xyz_i(3));
%         Iyy=m(link_i)*(xyz_i(1)+xyz_i(3));
%         Izz=m(link_i)*(xyz_i(1)+xyz_i(2));
%         Ixy=-m(link_i)*(xyz_i(1)*xyz_i(2));
%         Ixz=-m(link_i)*(xyz_i(1)*xyz_i(3));
%         Iyz=-m(link_i)*(xyz_i(2)*xyz_i(3));
%         I{link_i}=Ti{link_i+1}(1:3,1:3)*diag([I_x,I_y,I_z])*t_Ti(1:3,1:3);
        I{link_i}=[Ixx Ixy Ixz;
                   Ixy Iyy Iyz;
                   Ixz Iyz Izz];
    else
        I{link_i}=zeros(3,3);
    end
end

D = (m(1)*J_v{1}.'*J_v{1} + J_w{1}.'*I{1}*J_w{1});

for d_counter = 2:5
    D = D + (m(d_counter)*J_v{d_counter}.'*J_v{d_counter} + J_w{d_counter}.'*I{d_counter}*J_w{d_counter});
end
D=simplify(D);
% v_dq=[vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1) vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1)];
% E_k=simplify(1/2*dxi.'*D*dxi);
%% Coriolis
Cs = sym(zeros(5,5,5));
for i1 = 1:5
    for j1 = 1:5
        for k1 = 1:5
              diff1 = 1/2*(diff(D(k1,j1),xi(i1)));
            diff2 = 1/2*(diff(D(k1,i1),xi(j1)));
            diff3 = 1/2*(diff(D(i1,j1),xi(k1)));
            Cs(i1,j1,k1) = (diff1+diff2-diff3)*dxi(i1);
        end
    end
end
cor = sym(zeros(5,5));

for k1 = 1:5
    for j1 = 1:5 
        for i1 = 1:5
            cor(k1,j1)=cor(k1,j1)+Cs(i1, k1 , j1);
        end
    end
end
Phi = sym(zeros(5,1));

for i1 = 1:5
    Phi(i1) = diff(E_p,xi(i1));
%     Phi = Phi;2345
end
%% EOM rigid
% syms f_p1 f_p2 f_p3
% ddxi = sym('ddxi', [5 1], 'rational'); % "q double dot" - the second derivative of the q's in time (joint accelerations)
% eom_lhs = D*ddxi+cor*dxi+Phi;

par_set.B_rigid=D;
par_set.C_rigid=cor;
par_set.G_rigid=Phi;
% for i =1:3
%     T_p{i}=Ti{end}*[eye(3),par_set.r_p{i};0 0 0 1];
%     r_p_base{i}=T_p{i}(1:3,4);
% end
% % par_set.sym_wrench=[f_p1*T_p{1}(1:3,3);cross(r_p_base{1},f_p1*T_p{1}(1:3,3))]+...
% %     [f_p2*T_p{2}(1:3,3);cross(r_p_base{2},f_p2*T_p{2}(1:3,3))]+...
% %     [f_p3*T_p{3}(1:3,3);cross(r_p_base{3},f_p3*T_p{3}(1:3,3))];
% % 
% % eom_rhs=par_set.J_xyz{end}.'*par.sym_wrench;
% % par_set.f_xi=eom_rhs;
% % % par_set.sym_wrench=[f_x f_y f_z tau_x tau_y tau_z].';
%% mapping
temp_var=[];
syms phi theta L dphi dtheta ddphi ddtheta phi_t(t) theta_t(t) b_phi l_rect_sq_3

%%%%% Case List
% % % % % % % % % % 
% % % theta>0 and phi in [0,2pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);

% % % theta>0 and phi = 2pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);

% % % theta>0 and phi in [2pi/3,4pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);

% % % theta>0 and phi = 4pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(4*pi/3-pi/3);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);

% % % theta>0 and phi in [4pi/3,2pi]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-5*pi/3);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);

% % % theta>0 and phi = 2pi
b_phi=l_rect_sq_3*cos(pi/3)/cos(2*pi-pi/3);
b_theta_phi= (L/(theta)-b_phi)*sin(theta/2);
% % % % % % % % % % % 

% % % % % % % % % % % 
% % % theta=0 and phi in [0,2pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= L/2;

% % % theta=0 and phi = 2pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= L/2;

% % % theta=0 and phi in [2pi/3,4pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi);
b_theta_phi= L/2;

% % % theta=0 and phi = 4pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(4*pi/3-pi/3);
b_theta_phi= L/2;

% % % theta=0 and phi in [4pi/3,2pi]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-5*pi/3);
b_theta_phi= L/2;

% % % theta=0 and phi = 2pi
b_phi=l_rect_sq_3*cos(pi/3)/cos(2*pi-pi/3);
b_theta_phi= L/2;
% % % % % % % % % % % % 

% % % % % % % % % % 
% % % theta>0 and phi in [0,2pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);

% % % theta>0 and phi = 2pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi/3);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);

% % % theta>0 and phi in [2pi/3,4pi/3]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-pi);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);

% % % theta>0 and phi = 4pi/3
b_phi=l_rect_sq_3*cos(pi/3)/cos(4*pi/3-pi/3);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);

% % % theta>0 and phi in [4pi/3,2pi]
b_phi=l_rect_sq_3*cos(pi/3)/cos(phi-5*pi/3);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);

% % % theta>0 and phi = 2pi
b_phi=l_rect_sq_3*cos(pi/3)/cos(2*pi-pi/3);
b_theta_phi= (L/(theta)+b_phi)*sin(theta/2);
% % % % % % % % % % % 

m_q=[phi theta/2 b_theta_phi b_theta_phi theta/2 ].';
J_f=diff(m_q,theta);
temp.dJ_f=diff(subs(J_f,[theta],[theta_t(t)]),t);
dJ_fdt=subs(temp.dJ_f,[theta_t(t),diff(theta_t(t), t)],[theta,dtheta]);
par_set.J_xi2q=J_f;
temp.xi=m_q;
temp.dxi=J_f*[dtheta].';
temp.ddxi=dJ_fdt*[dtheta].'+J_f*[ddtheta].';
% %%
B_xi_q=subs(D,xi,m_q);
par_set.sym_J_xi2q=subs(par_set.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;
%%%
% M_xi_q=subs(M,xi,m_q);
% par_set.B_q_simplify=J_f.'*M_xi_q*J_f;
% %%
% temp_1=subs(cor,xi,f);
% temp_2=subs(temp_1,dxi,df);
% %%
% C_q=J_f.'*subs(D,xi,f)*J_ff+J_f.'*temp_2*J_f;
C_q=J_f.'*subs(D,xi,m_q)*dJ_fdt+J_f.'*subs(cor,[xi;dxi],[temp.xi;temp.dxi])*J_f;
% %%
G_q=J_f.'*subs(Phi,xi,m_q);
% %%
% par_set.J_xyz2q=subs(par_set.J_xyz2xi*par_set.J_xi2q,[xi],[m_q]);

% f_q=J_f.'*subs(par_set.f_xi,xi,f);
% 
% %%
% % par_set={};
par_set.B_q=B_q;
par_set.C_q=C_q;
par_set.G_q=G_q;

% par_set.C_q_simplify=simplify(C_q);
% par_set.G_q_simplify=simplify(G_q);
%% Actuation mapping
syms tau_x tau_y pm1 pm2 pm3 Al
temp.tau=[-sind(30) -sind(30) sind(90);
                 cosd(30) -cosd(30) 0;
                 0 0 0]*[pm1 pm2 pm3].';
 temp.Rphi=[cos(phi-pi/2) -sin(phi-pi/2)  0;
            sin(phi-pi/2) cos(phi-pi/2)   0;
            0                   0                   1];
  temp.Rtheta=[cos(theta) -sin(theta)  0;
            sin(theta) cos(theta)   0;
            0                   0                   1];
        temp.Ry=[cos(-theta) 0 sin(-theta) ;
                0 1 0
                    -sin(-theta) cos(-theta)   0;];
 temp.Rx=[1 0 0
            0 0 -1
            0 1 0]; 
 temp.tauBaseFrame=temp.Rphi.'*temp.tau;
 par_set.tauBaseFrame= simplify(temp.tauBaseFrame);
fprintf('EOM Done\n')
end