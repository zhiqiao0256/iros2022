function trainSet= func3ptFilter(trainSet)
windowSize = 3; 
filter_b = (1/windowSize)*ones(1,windowSize);
filter_a = 1;
trainSet.pos_phi_rad=zeros(length(trainSet.phi_rad),1);
trainSet.pos_phi_rad(1:end)=filter(filter_b,filter_a,trainSet.phi_rad(1:end));
% trainSet.velocity_phi_rad=zeros(length(trainSet.phi_rad),1);
% trainSet.velocity_phi_rad(2:end)=filter(filter_b,filter_a,(trainSet.phi_rad(2:end)-trainSet.phi_rad(1:end-1))/(1/20));
% trainSet.acc_phi_rad=zeros(length(trainSet.phi_rad),1);
% trainSet.acc_phi_rad(2:end)=filter(filter_b,filter_a,(trainSet.velocity_phi_rad(2:end)-trainSet.velocity_phi_rad(1:end-1))/(1/20));
trainSet.velocity_phi_rad=zeros(length(trainSet.phi_rad),1);
trainSet.velocity_phi_rad(2:end)=filter(filter_b,filter_a,(trainSet.pos_phi_rad(2:end)-trainSet.pos_phi_rad(1:end-1))/(1/20));
trainSet.acc_phi_rad=zeros(length(trainSet.phi_rad),1);
trainSet.acc_phi_rad(2:end)=filter(filter_b,filter_a,(trainSet.velocity_phi_rad(2:end)-trainSet.velocity_phi_rad(1:end-1))/(1/20));

trainSet.pos_theta_rad=zeros(length(trainSet.phi_rad),1);
trainSet.pos_theta_rad(1:end)=filter(filter_b,filter_a,trainSet.theta_rad(1:end));
trainSet.velocity_theta_rad=zeros(length(trainSet.theta_rad),1);
trainSet.velocity_theta_rad(2:end)=smooth((trainSet.pos_theta_rad(2:end)-trainSet.pos_theta_rad(1:end-1))/(1/20));
trainSet.acc_theta_rad=zeros(length(trainSet.theta_rad),1);
trainSet.acc_theta_rad(2:end)=smooth((trainSet.velocity_theta_rad(2:end)-trainSet.velocity_theta_rad(1:end-1))/(1/20));
end