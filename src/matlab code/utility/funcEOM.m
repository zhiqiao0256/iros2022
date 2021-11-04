function par =funcEOM(par)
fprintf( 'EOM... \n' )
%% Protential energy
syms g
E_p=0;
Ti=par.Ti;
m=par.rigid_m;
for i =1:length(Ti)
    p_i{i}=Ti{i}(1:3,4);
    z_i{i}=Ti{i}(1:3,3);
end
for link_i=1:par.n
    E_p=E_p+m(link_i)*[0;0;g].'*p_i{link_i+1};
end 
% return
%% Inerial and Kinetic energy
syms Ixx Iyy Izz Ixy Ixz Iyz
I=cell(par.n,1);
% I = [0 0 0 0 I_u 0 0 0 0 0];
for link_i =1:par.n
    t_Ti=Ti{link_i+1}.';
    if link_i == 3 %%%%%%par.n/2
        xyz_i=Ti{link_i+1}(1:3,4).^2;
        Ixx=m(link_i)*(xyz_i(2)+xyz_i(3));
        Iyy=m(link_i)*(xyz_i(1)+xyz_i(3));
        Izz=m(link_i)*(xyz_i(1)+xyz_i(2));
        Ixy=-m(link_i)*(xyz_i(1)*xyz_i(2));
        Ixz=-m(link_i)*(xyz_i(1)*xyz_i(3));
        Iyz=-m(link_i)*(xyz_i(2)*xyz_i(3));
%         I{link_i}=Ti{link_i+1}(1:3,1:3)*diag([I_x,I_y,I_z])*t_Ti(1:3,1:3);
        I{link_i}=[Ixx Ixy Ixz;
                   Ixy Iyy Iyz;
                   Ixz Iyz Izz];
    else
        I{link_i}=zeros(3,3);
    end
end
J_v=par.J_v;
J_w=par.J_w;
D = (m(1)*J_v{1}.'*J_v{1} + J_w{1}.'*I{1}*J_w{1});

for d_counter = 2:par.n
    D = D + (m(d_counter)*J_v{d_counter}.'*J_v{d_counter} + J_w{d_counter}.'*I{d_counter}*J_w{d_counter});
end
D=simplify(D);
% v_dq=[vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1) vec_q(1) vec_q(2) 0 vec_q(3) -vec_q(1)];
% E_k=simplify(1/2*dxi.'*D*dxi);
% return
%% Coriolis
Cs = sym(zeros(par.n,par.n,par.n));
xi=par.rigid_xiVic;
dxi=par.rigid_dxiVic;
for i1 = 1:par.n
    for j1 = 1:par.n
        for k1 = 1:par.n
              diff1 = 1/2*(diff(D(k1,j1),xi(i1)));
            diff2 = 1/2*(diff(D(k1,i1),xi(j1)));
            diff3 = 1/2*(diff(D(i1,j1),xi(k1)));
            Cs(i1,j1,k1) = (diff1+diff2-diff3)*dxi(i1);
        end
    end
end
Cor = sym(zeros(par.n,par.n));

for k1 = 1:par.n
    for j1 = 1:par.n 
        for i1 = 1:par.n
            Cor(k1,j1)=Cor(k1,j1)+Cs(i1, k1 , j1);
        end
    end
end
Phi = sym(zeros(par.n,1));

for i1 = 1:par.n
    Phi(i1) = diff(E_p,xi(i1));
%     Phi = Phi;2345
end
%% EOM rigid
% syms f_p1 f_p2 f_p3
% ddxi = sym('ddxi', [par.n 1], 'rational'); % "q double dot" - the second derivative of the q's in time (joint accelerations)
% eom_lhs = D*ddxi+cor*dxi+Phi;

par.B_rigid=D;
par.C_rigid=Cor;
par.G_rigid=Phi;
% for i =1:3
%     T_p{i}=Ti{end}*[eye(3),par.r_p{i};0 0 0 1];
%     r_p_base{i}=T_p{i}(1:3,4);
% end
% % par.sym_wrench=[f_p1*T_p{1}(1:3,3);cross(r_p_base{1},f_p1*T_p{1}(1:3,3))]+...
% %     [f_p2*T_p{2}(1:3,3);cross(r_p_base{2},f_p2*T_p{2}(1:3,3))]+...
% %     [f_p3*T_p{3}(1:3,3);cross(r_p_base{3},f_p3*T_p{3}(1:3,3))];
% % 
% % eom_rhs=par.J_xyz{end}.'*par.sym_wrench;
% % par.f_xi=eom_rhs;
% % % par.sym_wrench=[f_x f_y f_z tau_x tau_y tau_z].';
% return
%% mapping
syms phi theta l_tri Rp1 L dphi dtheta ddphi ddtheta
% mq=[phi theta/2-pi/2 b_phi_theta b_phi_theta theta/2-pi/2]' (5x1)
mq=[];
mod_phi=mod((phi + Rp1),2*pi/3);
r_phi=l_tri/sqrt(3)*cos(pi/3)/cos(mod_phi- pi/3);
% assuming theta >0 (z >0)
b_phi_theta= sin(theta/2).*(L/theta-r_phi);
mq=[phi theta/2 b_phi_theta b_phi_theta theta/2].';
% Jf=[dmq_dphi, dmq_dtheta] 5x2
dr_phi_dphi= l_tri/sqrt(3)*cos(pi/3)/(cos(mod_phi- pi/3)).^2*sin(mod_phi- pi/3);
db_phi_theta_dphi=(-1)*sin(theta/2)*dr_phi_dphi;
dmq_dphi=[1 0 db_phi_theta_dphi db_phi_theta_dphi 0].';
db_phi_theta_dtheta= (-1)*L/theta.^2*sin(theta/2)+(L/theta-r_phi)*cos(theta/2);
dmq_dtheta=[0 0.5 db_phi_theta_dtheta db_phi_theta_dtheta 0.5].';
Jf=[dmq_dphi, dmq_dtheta];
% dJf_dt(5x2)
ddr_phi_dphidt=l_tri/sqrt(3)*cos(pi/3)*...
                ((2)/(cos(mod_phi- pi/3)).^3*sin(mod_phi-pi/3).^2+1/cos(mod_phi- pi/3));
ddb_phi_theta_dphidt=(-0.5)*cos(theta/2)*dr_phi_dphi+(-1)*sin(theta/2)*ddr_phi_dphidt;
ddb_phi_theta_dthetadt=(2)*L/theta.^3*sin(theta/2)+(-1)*L/theta.^2*cos(theta/2)...
                        + (-1)*L/theta.^2*cos(theta/2) + (L/theta-r_phi)* sin(theta/2);
ddmq_dphidt=[0 0 ddb_phi_theta_dphidt ddb_phi_theta_dphidt 0].';
ddmq_dthetadt=[0 0 ddb_phi_theta_dthetadt ddb_phi_theta_dthetadt 0].';
dJf_dt=[ddmq_dphidt,ddmq_dthetadt];
% ddJf_dt2(5x2)
dddr_phi_dphidt2=l_tri/sqrt(3)*cos(pi/3)*...
                 ((-6)/(cos(mod_phi- pi/3)).^4*sin(mod_phi-pi/3).^3 + (4)/(cos(mod_phi- pi/3)).^2*sin(mod_phi-pi/3)...
                 +(-1)/(cos(mod_phi- pi/3)).^2*sin(mod_phi-pi/3));
dddb_phi_theta_dphidt2=(0.5)*sin(theta/2)*dr_phi_dphi + (-0.5)*cos(theta/2)* ddr_phi_dphidt...
                        +(-1)*cos(theta/2)*ddr_phi_dphidt + (-1)*sin(theta/2)*dddr_phi_dphidt2;
                    
dddb_phi_theta_dthetadt2=(-6)*L/theta.^4*sin(theta/2) + (2)*L/theta.^3*cos(theta/2)...
                +(-2)*L/theta.^3*cos(theta/2) + (1)*L/theta.^2*sin(theta/2)...
                +(2)*L/theta.^3*cos(theta/2) + (1)*L/theta.^2*sin(theta/2)...
                +(-1)*L/theta.^2* sin(theta/2)+(L/theta-r_phi)* cos(theta/2);
            dddmq_dphidt2=[0 0 dddb_phi_theta_dphidt2 dddb_phi_theta_dphidt2 0].';
            dddmq_dthetadt2=[0 0 dddb_phi_theta_dthetadt2 dddb_phi_theta_dthetadt2 0].';
ddJf_dt2=[dddmq_dphidt2,dddmq_dthetadt2];            
%% calcualte B_q,C_q,G_q
tempVar=[];
tempVar.xi=mq;
tempVar.dxi=Jf*[dphi dtheta].';
tempVar.ddxi=dJf_dt*[dphi dtheta].'+Jf*[ddphi ddtheta].';
B_xi_q=subs(D,xi,mq.');
B_q=Jf.'*B_xi_q*Jf;

C_q=Jf.'*subs(D,xi,mq.')*dJf_dt +Jf.'*subs(Cor,[xi;dxi],[tempVar.xi.';tempVar.dxi.'])*Jf;

G_q=Jf.'*subs(Phi,xi,mq.');

par.B_q=B_q;
par.C_q=C_q;
par.G_q=G_q;
% return
% temp_var=[];
% syms phi theta L dphi dtheta ddphi ddtheta phi_t(t) theta_t(t) 
% b_theta= (L/(theta)-b0)*sin(theta/2);
% 
% m_q=[theta/2 b_theta b_theta theta/2 ].';
% J_f=diff(m_q,theta);
% 
% temp.dJ_f=diff(subs(J_f,[theta],[theta_t(t)]),t);
% dJ_fdt=subs(temp.dJ_f,[theta_t(t),diff(theta_t(t), t)],[theta,dtheta]);
% par.J_xi2q=J_f;
% temp.xi=m_q;
% temp.dxi=J_f*[dtheta].';
% temp.ddxi=dJ_fdt*[dtheta].'+J_f*[ddtheta].';
% % %%
% B_xi_q=subs(D,xi,m_q);
% par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
% B_q=J_f.'*B_xi_q*J_f;
% %%%
% % M_xi_q=subs(M,xi,m_q);
% % par.B_q_simplify=J_f.'*M_xi_q*J_f;
% % %%
% % temp_1=subs(cor,xi,f);
% % temp_2=subs(temp_1,dxi,df);
% % %%
% % C_q=J_f.'*subs(D,xi,f)*J_ff+J_f.'*temp_2*J_f;
% C_q=J_f.'*subs(D,xi,m_q)*dJ_fdt+J_f.'*subs(Cor,[xi;dxi],[temp.xi;temp.dxi])*J_f;
% % %%
% G_q=J_f.'*subs(Phi,xi,m_q);
% % %%
% % par.J_xyz2q=subs(par.J_xyz2xi*par.J_xi2q,[xi],[m_q]);
% 
% % f_q=J_f.'*subs(par.f_xi,xi,f);
% % 
% % %%
% % % par={};
% par.B_q=B_q;
% par.C_q=C_q;
% par.G_q=G_q;

% par.C_q_simplify=simplify(C_q);
% par.G_q_simplify=simplify(G_q);
%% Actuation mapping
% syms tau_x tau_y pm1 pm2 pm3 Al
% temp.tau=[-sind(30) -sind(30) sind(90);
%                  cosd(30) -cosd(30) 0;
%                  0 0 0]*[pm1 pm2 pm3].';
%  temp.Rphi=[cos(phi-pi/2) -sin(phi-pi/2)  0;
%             sin(phi-pi/2) cos(phi-pi/2)   0;
%             0                   0                   1];
%   temp.Rtheta=[cos(theta) -sin(theta)  0;
%             sin(theta) cos(theta)   0;
%             0                   0                   1];
%         temp.Ry=[cos(-theta) 0 sin(-theta) ;
%                 0 1 0
%                     -sin(-theta) cos(-theta)   0;];
%  temp.Rx=[1 0 0
%             0 0 -1
%             0 1 0]; 
%  temp.tauBaseFrame=temp.Rphi.'*temp.tau;
%  par.tauBaseFrame= simplify(temp.tauBaseFrame);
fprintf('EOM Done\n')
end