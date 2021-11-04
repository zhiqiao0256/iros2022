function [dx, y] = funcFullModelOde(t, x, u, alpha,k,b,c,d, varargin)
% Output equations.
  y = [x(1);    % Angular position.                        
      x(2)
      x(3)
      x(4)
      x(5)];    % Angular velocity.
% State equations.
   m0=0.35;     % segment weight kg
   g=9.8;       % gravity 
   L=0.19;      % segment length
   pd1=u(1);    % Chamber 1 measured pressure
   pd2=u(2);    % Chamber 2 measured pressure
   pd3=u(3);    % Chamber 3 measured pressure
   r0=u(4);     % Off-set parameter for CC assumption
   phi=u(5);    % Azumuth angle
   theta=x(1);
   dtheta=x(2);
   pm1=x(3);    % Chamber 1 measured pressure
   pm2=x(4);    % Chamber 2 measured pressure
   pm3=x(5);    % Chamber 3 measured pressure
   Izz=m0*r0^2;
   
   M=Izz/4 + m0*((cos(theta/2)*(r0 - L/theta))/2 +...
       (L*sin(theta/2))/theta^2)^2 + (m0*sin(theta/2)^2*(r0 - L/theta)^2)/4;
   C_simp=-(L*dtheta*m0*(2*sin(theta/2) - theta*cos(theta/2))*(2*L*sin(theta/2)...
       - L*theta*cos(theta/2) + r0*theta^2*cos(theta/2)))/(2*theta^5);
   G_simp=-(g*m0*(L*sin(theta) + r0*theta^2*cos(theta) - L*theta*cos(theta)))/(2*theta^2);
   dx = [x(2);                        
        M\(-k*x(1) -(b+C_simp)*x(2)- G_simp+ (sin(phi)*(0.5*pm1+0.5*pm2-pm3)...
        -sqrt(3)*cos(phi)*(0.5*pm1-0.5*pm2))*alpha);  
        c*pm1+d*pd1;
        c*pm2+d*pd2;
        c*pm3+d*pd3;
       ];
end 