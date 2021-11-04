function par_set=funcFwdKinematic5linkSyms(par_set)
% temp_b=(par_set.L./trainSet.theta_rad-sign(trainSet.theta_rad).*trainSet.Ri).*sin(trainSet.theta_rad/2);
syms q1 q2 q3 q4 q5 L m0 A
xi_vector=[q1 q2 q3 q3 q2 -q1];par_set.n=5;
for j=1:1
    xi=xi_vector(1,:);
%     rigid_b0=trainSet.Ri(j);thetad=trainSet.theta_rad(j);L=par_set.L;a0=par_set.a0;
%      p1=trainSet.pm_MPa(j,2);p2=trainSet.pm_MPa(j,3);p3=trainSet.pm_MPa(j,4);
% % Maps to Table 1. DH parameters  
    rigid_a=zeros(1,par_set.n);
    rigid_alpha=[-90    90     0      -90      90       ];
    rigid_d=    [0      0      xi(3)   xi(4)    0       ];
    rigid_theta=[xi(1)  xi(2)  0       0        xi(5)   ];
    m=[0 0 m0 0 0 0];
% %
% Get HTM T{i}, p{i}, z{i}, for Jacobian calculation    
    Ti = cell(par_set.n+1,1);
    Ti(1) = {eye(4)};
%     R_base2Cam=[-1 0 0;
%                 0 0 -1;
%                 0 1 0];
%     Ti{1}=[R_base2Cam',zeros(3,1);
%         0 0 0 1];
    p_i{1}=Ti{1}(1:3,4);
    z_i{1}=Ti{1}(1:3,3);
for i = 2:par_set.n+1

% ROTATION FOR X
A(1,1) = cos((rigid_theta(i-1)));
A(2,1) = sin((rigid_theta(i-1)));
A(3,1) = 0;
% ROTATION FOR Y 
A(1,2) = -(sin(rigid_theta(i-1)))*(cosd(rigid_alpha(i-1)))*(1);
A(2,2) = (cos(rigid_theta(i-1)))*(cosd(rigid_alpha(i-1)));
A(3,2) = (sin(rigid_theta(i-1)));

% ROTATION FOR Z
A(1,3) = (sin(rigid_theta(i-1)))*(sind(rigid_alpha(i-1)));
A(2,3) = -(cos(rigid_theta(i-1)))*(sind(rigid_alpha(i-1)))*(1);
A(3,3) = (cosd(rigid_alpha(i-1)));

% TRANSLATION VECTOR
A(1,4) = (rigid_a(i-1))*(cos(rigid_theta(i-1)));
A(2,4) = (rigid_a(i-1))*(sin(rigid_theta(i-1)));
A(3,4) = rigid_d(i-1);
A(4,1:4)=[0 0 0 1];
Ti{i} =  Ti{i-1} * A;   
% Ti{i} =  Ti{i-1} * ([cos(rigid_theta(i-1)) -sin(rigid_theta(i-1)) 0 0; sin(rigid_theta(i-1)) cos(rigid_theta(i-1)) 0 0; 0 0 1 0; 0 0 0 1] *[1 0 0 0; 0 1 0 0; 0 0 1 rigid_d(i-1); 0 0 0 1]*[1 0 0 rigid_a(i-1); 0 1 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 0; 0 cos(rigid_alpha(i-1)) -sin(rigid_alpha(i-1)) 0 ; 0 sin(rigid_alpha(i-1)) cos(rigid_alpha(i-1)) 0; 0 0 0 1]);
    p_i{i}=Ti{i}(1:3,4);
    z_i{i}=Ti{i}(1:3,3);
end
% Ti{4}(1:3,4)
Ti{end}
return
%% Linear Velocity J_v
J_v=cell(par_set.n,1);
i=par_set.n;
    j_v=(zeros(3,par_set.n));
    for j_counter =1:i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_v(:,j_counter)=z_i{j_counter};
        else %% Rotational Joint
            j_v(:,j_counter)=cross(z_i{j_counter},(p_i{i+1}-p_i{j_counter}));
        end
    end
    J_v{i}=j_v;
%% Angular Velocity J_w
J_w=cell(par_set.n,1);
i = par_set.n;
    j_w=(zeros(3,par_set.n));
    for j_counter =1:i
        if rigid_theta(j_counter) == 0 %% Prismatic Joint
            j_w(:,j_counter)=zeros(3,1);
        else %% Rotational Joint
            j_w(:,j_counter)=z_i{j_counter};
        end
    end
    J_w{i}=j_w;

%% Jacobian maps to Eq.(8)
J_xi2q{j}=[0.5, - (cos(thetad/2)*(rigid_b0 - L/thetad))/2 - (L*sin(thetad/2))/thetad^2,- (cos(thetad/2)*(rigid_b0 - L/thetad))/2 - (L*sin(thetad/2))/thetad^2, 0.5]';

J_x2xi{j}=[J_v{end};J_w{end}];
xyz_estimation(j,1:3)=(Ti{end}(1:3,4))';
% %% Wrech force w/ base frame
% f_p1=588.31*p1;%mpa
% f_p2=588.31*p2;%mpa
% f_p3=588.31*p3;%mpa
% for i =1:3
%     T_p{i}=Ti{end}*[eye(3),trainSet.r_p{i};0 0 0 1];
%     r_p_base{i}=T_p{i}(1:3,4);
% end
% 
% trainSet.top_p1_xyz(j,1:3)=r_p_base{1};
% trainSet.top_p2_xyz(j,1:3)=r_p_base{2};
% trainSet.top_p3_xyz(j,1:3)=r_p_base{3};
% 
% wrench_base_pm{j}=[f_p1*T_p{1}(1:3,3);cross(r_p_base{1},f_p1*T_p{1}(1:3,3))]+...
%     [f_p2*T_p{2}(1:3,3);cross(r_p_base{2},f_p2*T_p{2}(1:3,3))]+...
%     [f_p3*T_p{3}(1:3,3);cross(r_p_base{3},f_p3*T_p{3}(1:3,3))];
end
trainSet.xyz_estimation=xyz_estimation;
if par_set.plot_fwdKinematic ==1
    funcCompareKinematicXYZ(trainSet,trainSet.xyz_estimation,par_set);
end
end