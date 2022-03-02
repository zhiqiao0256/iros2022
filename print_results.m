clc;
close all;
clear all;
output = CantileverRod_control_uniform;
global t N result segment vd ud kapa_out L_out lenv lseg fseg pressure_of_tubes 
figure()
plot(t, 100*result(:,1,N+1))
xlabel('T(s)');ylabel('X(cm)')
title('position in x-axis')

figure()
plot(t, 100*result(:,2,N+1))
xlabel('T(s)');ylabel('Y(cm)')
title('position in y-axis')

figure()
plot(t, 100*result(:,3,N+1))
xlabel('T(s)');ylabel('Z(cm)')
title('position in z-axis')
hold on;
% figure()
plot(t,100*L_out(:,N+1))
xlabel('T(s)');ylabel('L(cm)')
title('length of the arm')

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

figure()
plot(t,lseg(:,2,1))
xlabel('T(s)');ylabel('Torque [N.m]')
title('Cross-section control input Torque')
hold on;
plot(t,lenv(:,2))
xlabel('T(s)');ylabel('Torque [N]')
title('Cross-section control input Force')

figure()
% plot(t,10*0.000145038*Pressure_from_actuator_F_1(:))
% xlabel('T(s)');ylabel('Pressure [psi]')
% title('Actuator pressure calculated from control input f')
% hold on;
% plot(t,30*0.000145038*Pressure_from_actuator_L_1(:))
% xlabel('T(s)');ylabel('Pressure [psi]')
% title('Actuator pressure calculated from control input l')
plot(t,pressure_of_tubes(:))
xlabel('T(s)');ylabel('Pressure [psi]')
title('Actuator pressure calculated from control input')

figure()
plot(t,ud(:,2,N+1))
hold on;
plot(t,result(:,24,N+1))
hold on;
plot(t,kapa_out(:,N+1))
hold on;
plot(t,ud(:,2,N+1)-result(:,24,N+1))
title('curvature vs. desired value vs. error in between')

figure()
plot(t,vd(:,3,N+1))
hold on;
plot(t,result(:,22,N+1))
hold on;
plot(t,vd(:,3,N+1)-result(:,22,N+1))
title('extension vs. desired value vs. error in between')
