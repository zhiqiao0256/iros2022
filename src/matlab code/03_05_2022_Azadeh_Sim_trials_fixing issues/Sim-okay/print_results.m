clc;
close all;
clear all;
output = CantileverRod_control_uniform;
global elapsedTime t N exp_result sim_result segment vd ud kapa_out L_out lenv fenv lseg fseg Pressure_from_actuator_L_1 pressure_of_tubes
%%
figure()
plot(100*exp_result(2:end,1,N+1))
hold on;
plot(100*sim_result(:,1,N+1))
xlabel('T(s)');ylabel('X(cm)')
title('position in x-axis')
%%
figure()
plot(100*exp_result(:,2,N+1))
hold on;
plot(100*sim_result(:,2,N+1))
xlabel('T(s)');ylabel('Y(cm)')
title('position in y-axis')
%%
figure()
plot(100*exp_result(:,3,N+1))
hold on;
plot(100*sim_result(:,3,N+1))
xlabel('T(s)');ylabel('Z(cm)')
title('position in z-axis')
hold on;
% figure()
plot(100*L_out(:,N+1))
xlabel('T(s)');ylabel('L(cm)')
title('length of the arm')
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
figure()
plot(fseg(:,3,1))
xlabel('T(s)');ylabel('Force [N.m]')
title('Cross-section control input Force')
hold on;
plot(fenv(:,3))
xlabel('T(s)');ylabel('Force [N]')
title('Cross-section control input Force')
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
% plot(t,10*0.000145038*Pressure_from_actuator_F_1(:))
% xlabel('T(s)');ylabel('Pressure [psi]')
% title('Actuator pressure calculated from control input f')
% hold on;
plot(t,Pressure_from_actuator_L_1(:))
xlabel('T(s)');ylabel('Pressure [psi]')
title('Actuator pressure calculated from control input l')
plot(pressure_of_tubes(:))
xlabel('T(s)');ylabel('Pressure [psi]')
title('Actuator pressure calculated from control input')
%%
figure()
plot(ud(:,2,N+1))
hold on;
plot(sim_result(:,24,N+1))
hold on;
plot(exp_result(:,24,N+1))
hold on;
plot(kapa_out(:,N+1))
hold on;
% plot(exp_result(:,24,N+1)-ud(:,2,N+1))
title('curvature vs. desired value vs. error in between')
%%
figure()
plot(vd(:,3,N+1))
hold on;
plot(sim_result(:,22,N+1))
hold on;
plot(exp_result(:,22,N+1))
hold on;
% plot(exp_result(:,22,N+1)-vd(:,3,N+1))
title('extension vs. desired value vs. error in between')
