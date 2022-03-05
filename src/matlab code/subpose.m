clear all;close all;clc;
recv= tcpserver("10.203.49.203",55555)
datasize=14;
for i =1:1000
    i
    mocap_data = read(recv,datasize,"double")
    if size(mocap_data,1)>0
       position_global = mocap_data(8:10)-mocap_data(1:3);
       x_bias_initial= position_global(1)
       position_global(1) = position_global(1)-x_bias_initial;
       %calculating the rotation matrix using quaternion to euler
       base_angles = quat2eul(mocap_data(4:7),'zyx');
       phi=base_angles(1);
       theta=base_angles(2);
       psi=base_angles(3);
       R_ZYX_initial=[cos(phi)*cos(theta)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
              sin(phi)*cos(theta)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
             -sin(theta)        cos(theta)*sin(psi)                    cos(theta)*cos(psi)            ]
       %Calculating the Rzyx from quaternion directly
       h = mocap_data(11:14)';
       h1 = h(1); h2 = h(2); h3 = h(3); h4 = h(4);
       R_initial = eye(3)+ 2/( h'*h )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2]
    end
end