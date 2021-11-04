function [dx, y] = funcGreyBoxOde3D5link(t, x, u, a11,a22,k11,k22,b11,b22, varargin)
% Output equations.
y = [x(1);    % Angular position.
    x(2)
    X(3)
    X(4)];    % Angular velocity.
% State equations.
m0=0.35;     % segment weight kg
g=9.8;       % gravity
L=0.185;
l_tri=0.015;% segment length
Rp1=deg2rad(240);
pm1=u(1);    % Chamber 1 measured pressure
pm2=u(2);    % Chamber 2 measured pressure
pm3=u(3);    % Chamber 3 measured pressure
r0=u(4);     % Off-set parameter for CC assumption
phi=u(5);    % Azumuth angle
phi=x(1);
dphi=x(2);
theta=x(1);
dtheta=x(2);
%    Izz=m0*r0^2;
%    Iyz=0.
% Update Parameters
if theta == pi/2
    m11=0;
    m12=0;
    m21=0;
    m22=m0*(L/2)^2;
else
m11=  5*m0*sin(theta/2)^4*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 ...
    + (l_tri^2*m0*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2*sin(theta/2)^2)/(12*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^4);

m12= (m0*cos(theta/2)^2*sin(theta/2)^6*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4)/2 ...
    + (3^(1/2)*l_tri*m0*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2)*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)...
       /(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2);
m21= m12;
m22=  m0*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2)^2 ...
+ (m0*sin(theta/2)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2*(5*cos(theta/2)^2*cos(phi)^2 + ...
	5*cos(theta/2)^2*sin(phi)^2 + 5*sin(theta/2)^2*cos(phi)^4 + 8*sin(theta/2)^2*sin(phi)^2 - 3*sin(theta/2)^2*sin(phi)^4 + 
	2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2))/4
end
M=[m11 m12;m21 m22];
c11= 10*m0*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*...
    (dtheta*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - ...
    (L*sin(theta/2))/theta^2) + (3^(1/2)*dphi*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*...
    sin(theta/2))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2)) + ...
    5*dtheta*m0*cos(theta/2)*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/...
    (6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 - (3^(1/2)*l_tri*m0*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2)*((3^(1/2)*l_tri*sin(theta/2)*((2*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2)/cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^3 + 1/cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))/6 - (3^(1/2)*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*cos(theta/2))/(12*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2)))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2);
c12= (3^(1/2)*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2)*((dtheta*(m0*sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*(5*cos(theta/2)^2*cos(phi)^2 + 5*cos(theta/2)^2*sin(phi)^2 + 5*sin(theta/2)^2*cos(phi)^4 + 8*sin(theta/2)^2*sin(phi)^2 - 3*sin(theta/2)^2*sin(phi)^4 + 2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2) + 2*m0*sin(theta/2)^7*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3))/2 + 2*dphi*m0*cos(theta/2)^2*sin(theta/2)^5*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2) - 2*(5*dphi*m0*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) + dtheta*m0*cos(theta/2)^2*sin(theta/2)^5*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3)*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) - (dtheta*m0*sin(theta/2)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2*(12*sin(theta/2)^2*cos(phi)*sin(phi)^3 + 20*sin(theta/2)^2*cos(phi)^3*sin(phi) - 16*sin(theta/2)^2*cos(phi)*sin(phi) + 6*sin(theta/2)^6*cos(phi)^2*sin(phi)^4*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 - 6*sin(theta/2)^6*cos(phi)^4*sin(phi)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2))/8 - (5*dphi*m0*cos(theta/2)*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2)/2 - (dphi*(5*m0*cos(theta/2)*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 - m0*cos(theta/2)^2*sin(theta/2)^6*cos(phi)^2*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4 + m0*cos(theta/2)^2*sin(theta/2)^6*sin(phi)^2*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4 + m0*cos(theta/2)^2*sin(theta/2)^6*cos(phi)*sin(phi)*(cos(phi) + sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4))/2 + (3^(1/2)*l_tri*m0*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2)*(sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (2*L*cos(theta/2))/theta^2 + (2*L*sin(theta/2))/theta^3))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2) + 2*m0*cos(theta/2)^2*sin(theta/2)^5*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3*(dtheta*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) + (3^(1/2)*dphi*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2));
c21=  (dtheta*((m0*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2*(cos(theta)/2 - 1/2)*(20*cos(phi)^3*sin(phi)*(cos(theta)/2 - 1/2) - 4*sin(2*phi)*(cos(theta) - 1) + 6*cos(phi)*sin(phi)^3*(cos(theta) - 1) - 6*sin(theta/2)^6*cos(phi)^2*sin(phi)^4*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 + 6*sin(theta/2)^6*cos(phi)^4*sin(phi)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2))/2 - 2*2^(1/2)*m0*cos(theta/2)*sin(theta/2)^7*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4 + 2*2^(1/2)*m0*cos(theta/2)^3*sin(theta/2)^5*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4))/4 - (dtheta*(2^(1/2)*m0*cos(theta/2)*sin(theta/2)^7*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4 - 2^(1/2)*m0*cos(theta/2)^3*sin(theta/2)^5*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^4))/2 + 2*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2)*(5*dphi*m0*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) + 2^(1/2)*dtheta*m0*sin(theta/2)^5*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3*(cos(theta)/2 + 1/2)) + 5*dphi*m0*cos(theta/2)*sin(theta/2)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 - (3^(1/2)*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2)*((dtheta*(m0*sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*(5*cos(phi)^2*(cos(theta)/2 + 1/2) - 5*cos(phi)^4*(cos(theta)/2 - 1/2) + 5*sin(phi)^2*(cos(theta)/2 + 1/2) + 3*sin(phi)^4*(cos(theta)/2 - 1/2) - 4*sin(phi)^2*(cos(theta) - 1) + 2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2) + 2*m0*sin(theta/2)^7*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3))/4 + 2^(1/2)*dphi*m0*sin(theta/2)^5*cos(phi)*cos(phi + pi/4)*sin(phi)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3*(cos(theta)/2 + 1/2)))/(3*sin(pi/6 + mod(Rp1 + phi, (2*pi)/3))^2) - (3^(1/2)*l_tri*m0*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2)*(3*sin(theta/2) - (3*cos(theta/2 + pi/6 - 2*mod(Rp1 + phi, (2*pi)/3)))/4 + sin(theta/2 + pi/3 + 2*mod(Rp1 + phi, (2*pi)/3))/4))/(12*sin(pi/6 + mod(Rp1 + phi, (2*pi)/3))^3) + 2*2^(1/2)*m0*sin(theta/2)^5*cos(phi)*cos(phi + pi/4)*sin(phi)*(dtheta*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) + (3^(1/2)*dphi*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2))/(6*sin(pi/6 + mod(Rp1 + phi, (2*pi)/3))^2))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3*(cos(theta)/2 + 1/2);
c22 = ((m0*sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*(5*cos(theta/2)^2*cos(phi)^2 + 5*cos(theta/2)^2*sin(phi)^2 + 5*sin(theta/2)^2*cos(phi)^4 + 8*sin(theta/2)^2*sin(phi)^2 - 3*sin(theta/2)^2*sin(phi)^4 + 2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2) + 2*m0*sin(theta/2)^7*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3)*(dtheta*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) + (3^(1/2)*dphi*l_tri*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*sin(theta/2))/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2)))/2 + ((dtheta*(m0*sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*(5*cos(theta/2)^2*cos(phi)^2 + 5*cos(theta/2)^2*sin(phi)^2 + 5*sin(theta/2)^2*cos(phi)^4 + 8*sin(theta/2)^2*sin(phi)^2 - 3*sin(theta/2)^2*sin(phi)^4 + 2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2) + 2*m0*sin(theta/2)^7*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3))/2 + 2*dphi*m0*cos(theta/2)^2*sin(theta/2)^5*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3)*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) - 2*((dtheta*(m0*sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))*(5*cos(theta/2)^2*cos(phi)^2 + 5*cos(theta/2)^2*sin(phi)^2 + 5*sin(theta/2)^2*cos(phi)^4 + 8*sin(theta/2)^2*sin(phi)^2 - 3*sin(theta/2)^2*sin(phi)^4 + 2*sin(theta/2)^6*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2) + 2*m0*sin(theta/2)^7*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3))/4 + dphi*m0*cos(theta/2)^2*sin(theta/2)^5*cos(phi)*sin(phi)*(cos(phi) - sin(phi))*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^3)*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) + m0*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2)*(sin(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (2*L*cos(theta/2))/theta^2 + (2*L*sin(theta/2))/theta^3) + (dtheta*m0*sin(theta/2)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2*(10*cos(theta/2)*sin(theta/2)*cos(phi)^4 - 10*cos(theta/2)*sin(theta/2)*cos(phi)^2 + 6*cos(theta/2)*sin(theta/2)*sin(phi)^2 - 6*cos(theta/2)*sin(theta/2)*sin(phi)^4 + 8*cos(theta/2)*sin(theta/2)^5*cos(phi)^3*sin(phi)^3*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2))/8 - (dphi*m0*sin(theta/2)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2*(12*sin(theta/2)^2*cos(phi)*sin(phi)^3 + 20*sin(theta/2)^2*cos(phi)^3*sin(phi) - 16*sin(theta/2)^2*cos(phi)*sin(phi) + 6*sin(theta/2)^6*cos(phi)^2*sin(phi)^4*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2 - 6*sin(theta/2)^6*cos(phi)^4*sin(phi)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))))^2))/8;
 
C_simp=[c11 c12;c21 c22];
g11= (3^(1/2)*g*l_tri*m0*sin(pi/3 - mod(Rp1 + phi, (2*pi)/3))*cos(theta/2)*sin(theta/2))/(3*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))^2);
g21= 2*g*m0*cos(theta/2)*(cos(theta/2)*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3)))) - (L*sin(theta/2))/theta^2) - g*m0*sin(theta/2)^2*(L/theta - (3^(1/2)*l_tri)/(6*cos(pi/3 - mod(Rp1 + phi, (2*pi)/3))));
  
G_simp=[g11;g21];
tempVar=[];
tau_a=[];
dx = [x(2);
    M\(-k*x(1) -(b+C_simp)*x(2)- G_simp+ (sin(phi)*(0.5*pm1+0.5*pm2-pm3)...
    -sqrt(3)*cos(phi)*(0.5*pm1-0.5*pm2))*alpha);
    x(4)
    ];
end