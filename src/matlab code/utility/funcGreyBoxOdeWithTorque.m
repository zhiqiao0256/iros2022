function [dx, y] = funcGreyBoxOdeWithTorque(t, x, u, alpha,k,b, varargin)
% Output equations.
  y = [x(1);    % Angular position.                        
      x(2)];    % Angular velocity.
% State equations.
   m0=0.35;     % segment weight kg
   g=9.8;       % gravity 
   L=0.19;      % segment length
   tau=u(1);    % Chamber 1 measured pressure
   r0=u(2);     % Off-set parameter for CC assumption
   theta=x(1);
   dtheta=x(2);
   Izz=m0*r0^2;
   
   M=Izz/4 + m0*((cos(theta/2)*(r0 - L/theta))/2 +...
       (L*sin(theta/2))/theta^2)^2 + (m0*sin(theta/2)^2*(r0 - L/theta)^2)/4;
   C_simp=-(L*dtheta*m0*(2*sin(theta/2) - theta*cos(theta/2))*(2*L*sin(theta/2)...
       - L*theta*cos(theta/2) + r0*theta^2*cos(theta/2)))/(2*theta^5);
   G_simp=-(g*m0*(L*sin(theta) + r0*theta^2*cos(theta) - L*theta*cos(theta)))/(2*theta^2);
   dx = [x(2);                        
        M\(-k*x(1) -(b+C_simp)*x(2)- G_simp+ alpha*tau);   
       ];
end 