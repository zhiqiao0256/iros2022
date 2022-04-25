% clc;
% close all;
% clear all;
% output = CantileverRod_control_uniform;
% global elapsedTime t N exp_result sim_result segment vd ud kapa_out L_out lenv fenv lseg fseg Pressure_from_actuator pressure_of_tubes
%%
figure()
plot(100*exp_result(3:end,1,N))
hold on;
plot(100*sim_result(:,1,N))
xlabel('T(s)');ylabel('X(cm)')
title('position in x-axis')
legend('Experiment','Simulation')
figure()
plot(100*exp_result(3:end,1,N+1))
hold on;
plot(100*sim_result(:,1,N+1))
xlabel('T(s)');ylabel('X(cm)')
title('position in x-axis')
legend('Experiment','Simulation')
%%
figure()
plot(100*exp_result(2:end,2,N))
hold on;
plot(100*sim_result(:,2,N))
xlabel('T(s)');ylabel('Y(cm)')
title('position in y-axis')
legend('Experiment','Simulation')
figure()
plot(100*exp_result(2:end,2,N+1))
hold on;
plot(100*sim_result(:,2,N+1))
xlabel('T(s)');ylabel('Y(cm)')
title('position in y-axis')
legend('Experiment','Simulation')
%%
figure()
plot(100*exp_result(2:end,3,N))
hold on;
plot(100*sim_result(:,3,N))
xlabel('T(s)');ylabel('Z(cm)')
title('Segment 1 position in z-axis')
legend('Experiment','Simulation')

figure()
plot(100*exp_result(2:end,3,N+1))
hold on;
plot(100*sim_result(:,3,N+1))
xlabel('T(s)');ylabel('Z(cm)')
title('Segment 2 position in z-axis')
legend('Experiment','Simulation')
%% hold on;
figure()
plot(100*L_out(:,N))
hold on;
plot(100*L_out(:,N+1))
xlabel('T(s)');ylabel('L(cm)')
title('length of the arm')
legend('Length of segment 1','Length of segment 2')
%%
% figure()
% plot(t, 100*result(:,1,N/2))
% xlabel('T(s)');ylabel('X(cm)')
% title('position in x-axis')
% % figure()
% % plot(t, 100*result(:,2,N/2))
% % xlabel('T(s)');ylabel('Y(cm)')
% figure()
% plot(t, 100*result(:,3,N/2))
% xlabel('T(s)');ylabel('Z(cm)')
% 
%%
% figure()
% plot(fseg(:,3,1))
% xlabel('T(s)');ylabel('Force [N.m]')
% title('Cross-section control input Force')
% hold on;
% plot(fenv(:,3))
% xlabel('T(s)');ylabel('Force [N]')
% title('Cross-section control input Force')
%%
figure()
plot(lseg(:,2,1))
xlabel('T(s)');ylabel('Torque [N.m]')
title('Cross-section control input Torque')
hold on;
plot(lenv(:,2))
xlabel('T(s)');ylabel('Torque [N]')
title('Cross-section control input Torque')
%%
figure()
plot(1+0.000145038*Pressure_from_actuator)
hold on;
plot(pressure_of_tubes(2:end,:))
xlabel('T(s)');ylabel('Pressure [psi]')
title('Actuator Pressure')
%%
figure()
plot(ud(:,1,N))
hold on;
plot(exp_result(2:end,23,N))
hold on;
% plot(kapa_out(:,N))
hold on;
plot(sim_result(:,23,N),'.-')
title('Curvature')
legend('Desired','Experiment','Kapa','Simulation')

figure()
plot(ud(:,2,N+1))
hold on;
plot(exp_result(2:end,24,N+1))
hold on;
plot(kapa_out(:,N+1))
hold on;
plot(sim_result(:,24,N+1),'.-')
% plot(exp_result(:,24,N+1)-ud(:,2,N+1))
title('Curvature')
legend('Desired','Experiment','Kapa','Simulation')
%%
% figure()
% plot(vd(:,3,N))
% hold on;
% plot(exp_result(2:end,22,N))
% hold on;
% plot(sim_result(:,22,N),'.-')
% hold on;
% plot(vd(:,3,N+1))
% hold on;
% plot(exp_result(2:end,22,N+1))
% hold on;
% plot(sim_result(:,22,N+1),'.-')
% % plot(exp_result(:,22,N+1)-vd(:,3,N+1))
% title('Extension')
% legend('Desired','Experiment','Simulation')