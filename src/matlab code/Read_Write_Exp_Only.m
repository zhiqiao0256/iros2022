close all;clear all;clc;
%% Video recording ()
% myVideo = VideoWriter('grasp'); %open video file
% myVideo.FrameRate = 1;  %can adjust this, 5 - 10 works well for me
% open(myVideo)
%% Initialize to straight configuration
% y and z are general variables as in Eq.(12) >> Generalized PDE
% y : = [ p ; h ; n ; m ; q ; w ] and z : = [ v ; u ]
y = [zeros(2,N+1); linspace(0,L0,N+1); zeros(16,N+1)] ;
z = [zeros(2,N+1); ones(1,N+1); zeros(3,N+1)];
y_prev = y;
z_prev = z;
t= zeros(STEPS,1);
del_t = 1; 
STEPS = 5;
recv = tcpserver("10.203.49.203",55555);
sen = tcpclient("10.203.49.209",6666);
datasize = 14;
x_bias = 0.0015;
R_initial = [ 1.0000   -0.0015    0.0000
              0.0015    1.0000    0.0029
             -0.0000   -0.0029    1.0000];
%% Main timing loop
for i = 1:STEPS
%     i
    tic;
    t(i)= (i-1) * del_t; 
    if i==1
        pd_data = 0;
    else
        pd_data = 1;
    end
    if pd_data>=20
        pd_data=20;
    elseif pd_data<=0
        pd_data=0;
    end
    write(sen,pd_data)
    java.lang.Thread.sleep(100)
    %% main original code solving the coesserat rod forward dynamics
        mocap_data = read(recv,datasize,"double");
     if size(mocap_data)>0
        position_global = (mocap_data(8:10)-mocap_data(1:3))';
        position(1) = position_global(1,1)-x_bias;
        position(2) = position_global(2,1); 
        position(3) = position_global(3,1);  
        h = mocap_data(11:14)';
        h1 = h(1); h2 = h(2); h3 = h(3); h4 = h(4);
        R = eye(3)+ 2/( h'*h )* ...
            [-h3^2- h4^2 , h2*h3 - h4*h1 , h2*h4+h3*h1;
            h2*h3+h4*h1 , -h2^2 - h4^2 , h3*h4 - h2*h1;
            h2*h4 - h3*h1 , h3*h4 + h2*h1 , -h2^2 - h3^2];
        position_local = (R_initial'*R)'*position';
        if abs(position_local(1))<=0.00000001
            kapa(2)=0;
            dL(2)=0;
            L(2) = position_local(3);
        else          
            kapa(2) = -2*position_local(1)/(position_local(1)^2+position_local(3)^2);  %curvature_about_x_axis
%             L = asin(kapa(node+1)*p_l(3))/kapa(node+1); 
            L(2) = (2/kapa(2))*atan2(-position_local(1),position_local(3));
            dL(2) = L(2)-L0;
        end
        y(1:3,2) = [position(1);position(2);position(3)];
        y(4:7,2) = mocap_data(11:14)';
        z(3,2) = 1+dL(2)/L(2);
        z(5,2) = kapa(2);
        
        % Set history terms - Eq.(5)-b
        % y_prev ~ _()^(i-2)y(t)
        yh = c1*y + c2*y_prev;
        zh = c1*z + c2*z_prev;
        %store from previous step for the next step
        y_prev = y;
        z_prev = z;
        
        % Midpoints are linearly interpolated for RK4
        yh_int = 0.5*(yh(:, 1:end-1) + yh(:, 2:end));
        zh_int = 0.5*(zh(:, 1:end-1) + zh(:, 2:end));

        exp_result(i,:,:) = [y;z];    %saving results
        kapa_out(i,:) = kapa;
        L_out(i,:) = L;
        t_c = 0:pi/150:2*pi;

        for counter = 1:length(t_c)
                x_c = r0*cos(t_c(counter))+y(1,2);
                y_c = r0*sin(t_c(counter))+y(2,2);
                z_c = y(3,2);
                circle(counter,:) = [x_c;y_c;z_c]; 
                p_c(counter,:,1) = [r0*cos(t_c(counter));r0*sin(t_c(counter));0];
                p_c(counter,:,2) = R*circle(counter,:)';
        end
        plotRobot();
     end
end

function plotRobot()
        
%         p_c(:,:,1) = circle(:,:);   %first segment's first point
%         p_c(:,:,N+1) = p_c(:,:,N);    %last segment's last point
        for j=1:N+1
            p_c1(j,:) = p_c(:,1,j)';
            p_c2(j,:) = p_c(:,2,j)';
            p_c3(j,:) = p_c(:,3,j)';
%         end
%         for j=1:N+1
        
            if j<= N/physical_segment
                
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[1 0 0],'LineWidth',3);
    
            elseif j<= 2*N/physical_segment
                
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[0 0 1],'LineWidth',3);
                 
            elseif j<= 3*N/physical_segment
    
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[1 0 0],'LineWidth',3);
                 
            else
    
                 plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Color',[0 0 1],'LineWidth',3);
                 
            end          
            hold on;
            if mod(j-1,N/physical_segment)==0
                    plot3(100*p_c1(j,:), 100*p_c2(j,:), 100*p_c3(j,:),'Marker','o','Markersize',4,'MarkerEdgeColor','k'); %'Color',[1 0 0],'LineWidth',1,
            end
        end
    
    %             hold on;
    %             hold off;
                grid on;          
                ax = gca;
                ax.XDir = 'reverse';
                ax.YDir = 'reverse';
                ax.ZDir = 'normal';
                axis([-50 50 -50 50 0  50]) ;
                xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
    %           daspect ([ 2 1 1 ]); 
                view(0,0);
    %             view(15,10);
    %             view(-150,20);
                drawnow;
    %             if i==1
    %                 savefig('shearg1.fig')
    %             elseif i==3
    %                 savefig('shearg2.fig')
    %             elseif i==8
    %                 savefig('shearg3.fig')
    %             elseif i==10
    %                 savefig('shearg4.fig')
    %             end
                hold off;
        %get frame
%         frame = getframe(gcf);
%         writeVideo(myVideo, frame);
end
%   close(myVideo)