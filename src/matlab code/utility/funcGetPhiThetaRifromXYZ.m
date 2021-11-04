function trainSet = funcGetPhiThetaRifromXYZ(trainSet,par_set)
%% Calculate Phi in camera frame then maps to robot base frame 0,2pi
%%%% Y-up to Z-up rotate -90deg w/ x axis
if par_set.flag_exp_z_up == 0
    temp.Yup=trainSet.tip_exp;
    temp.Zup=trainSet.tip_exp;
    Rx=[1 0 0;
        0 cosd(-90) -sind(-90)
        0 sind(-90) cosd(-90)];
    for i =1:length(trainSet.pd_psi)
        temp.Zup(i,2:4)=((Rx)'*(temp.Yup(i,2:4)'))';
    end
elseif par_set.flag_exp_z_up == 1
     temp.Zup=trainSet.tip_exp;
end    




trainSet.tip_exp=temp.Zup;

% for i =1:length(trainSet.pd_psi)
%     temp.ZupR90(i,2:4)=((Rz)'*(temp.Zup(i,2:4)'))';
% end
% trainSet.tip_exp=temp.ZupR90;
temp.phi_vector=trainSet.tip_exp(:,2:4); %xyz in Camera frame
temp.angle_phi=[];
%%% Calculate phi anlge ranging [-pi,pi] atan(y_top/x_top)
temp.tip_exp_baseFrame(:,1)=trainSet.tip_exp(:,1);
for i =1:length(trainSet.pd_psi)
    temp.angle_phi(i,1)=rad2deg(atan2(temp.phi_vector(i,2),temp.phi_vector(i,1)));
    temp.angle_theta(i,1)=2*rad2deg(acos(temp.phi_vector(i,3)/norm(temp.phi_vector(i,:))));
    temp.Rz=[cosd(temp.angle_phi(i,1)) -sind(temp.angle_phi(i,1))  0;
        sind(temp.angle_phi(i,1)) cosd(temp.angle_phi(i,1))   0;
        0                   0                   1];
%     temp.Rz2=[1 0 0
%               0 0 -1
%               0 1 0];
      temp.Rz2=eye(3);
    temp.R_cam2Base=[cosd(-temp.angle_phi(i,1)) 0 sind(-temp.angle_phi(i,1));
        sind(-temp.angle_phi(i,1)) 0 -cosd(-temp.angle_phi(i,1));
        0                   1                   0];
    %         temp.Rz2=eye(3);
    temp.tip_exp_baseFrame(i,2:4)=((temp.Rz*temp.Rz2)'*(trainSet.tip_exp(i,2:4)'))';
end
trainSet.phi_deg=temp.angle_phi;
trainSet.phi_rad=deg2rad(temp.angle_phi);
trainSet.tip_exp_baseFrame=temp.tip_exp_baseFrame;
%%%%%%%
%% Calculate Ri in Arc Frame using phi only
trainSet.Ri=[];
Ri_array=zeros(length(trainSet.phi_deg),3);
p1_offset=deg2rad(par_set.p1_angle);
Ri_array=par_set.trianlge_length/sqrt(3)*cos(pi/3)./cos(mod((trainSet.phi_rad)+p1_offset,2*pi/3)-pi/3);
for i=1:length(Ri_array)
    Ri_k=Ri_array(i,:);
    theta_rad_k=deg2rad(temp.angle_theta(i));
    phi_rad_k=trainSet.phi_rad(i);
    if theta_rad_k==0
        b_theta_phi_i=par_set.L/2;
    else
    b_theta_phi_i=(par_set.L/theta_rad_k - sign(theta_rad_k)*Ri_k)*sin(theta_rad_k/2);
    end
    trainSet.Ri(i,1)=Ri_k;
    trainSet.b_theta_phi(i,1)=b_theta_phi_i;
    trainSet.x_y_edge(i,:)=[Ri_k*cos(phi_rad_k-pi),Ri_k*sin(phi_rad_k-pi),0];
end
if par_set.flag_plot_movingCC==1
    funcCamFramePlotMovingCC(trainSet,par_set);
end
% return
%% Calculate theta in Cam frame
% trainSet.theta_rad= sign(trainSet.tip_exp(:,2)).*...
%     (pi/2-asin(sqrt(trainSet.tip_exp(:,4).^2)./sqrt(trainSet.tip_exp(:,2).^2+trainSet.tip_exp(:,3).^2++trainSet.tip_exp(:,4).^2)));
%% Calculate theta in base frame ranging -pi/2,pi/2
% trainSet.theta_rad=2*-sign(trainSet.tip_exp_baseFrame(:,2)).*asin(sqrt(trainSet.tip_exp_baseFrame(:,2).^2)./sqrt(trainSet.tip_exp_baseFrame(:,2).^2+trainSet.tip_exp_baseFrame(:,3).^2));
% trainSet.theta_deg=rad2deg(trainSet.theta_rad);
trainSet.theta_deg=temp.angle_theta;
trainSet.theta_rad=deg2rad(trainSet.theta_deg);
% func_fwdKinematic(trainSet,par_set);
end