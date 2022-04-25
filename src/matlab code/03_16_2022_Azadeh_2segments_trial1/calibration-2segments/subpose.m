clear all;close all;clc;
recv= tcpserver("10.203.49.203",6666)
datasize=21;
for i =1:100
    i
    mocap_data = read(recv,datasize,"double")
    if size(mocap_data,1)>0

       position_global_base = mocap_data(1:3);
       x_bias_base= position_global_base(1)
       y_bias_base= position_global_base(2)
       z_bias_base= position_global_base(3)

       position_global_segment_1 = mocap_data(8:10)-mocap_data(1:3);
       x_bias_segment_1= position_global_segment_1(1)
       y_bias_segment_1= position_global_segment_1(2)
       z_bias_segment_1= position_global_segment_1(3)

       position_global_segment_2 = mocap_data(15:17)-mocap_data(1:3);
       x_bias_segment_2= position_global_segment_2(1)
       y_bias_segment_2= position_global_segment_2(2)
       z_bias_segment_2= position_global_segment_2(3)

       %Calculating the Rzyx from quaternion directly
       h_0 = mocap_data(4:7)';
       h1 = h_0(1); h2 = h_0(2); h3 = h_0(3); h4 = h_0(4);
       R_base = eye(3)+ 2/( h_0'*h_0 )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2]

       h_1 = mocap_data(11:14)';
       h1 = h_1(1); h2 = h_1(2); h3 = h_1(3); h4 = h_1(4);
       R_seg1 = eye(3)+ 2/( h_1'*h_1 )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2]

       h_2 = mocap_data(18:21)';
       h1 = h_2(1); h2 = h_2(2); h3 = h_2(3); h4 = h_2(4);
       R_seg2 = eye(3)+ 2/( h_2'*h_2 )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2]
    end
end