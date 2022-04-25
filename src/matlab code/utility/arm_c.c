#include "math.h"
/* State equations */
void compute_dx(double *dx, double *x,double *u,double **p)
{	/*Parameter to identify*/
	double *k11,*k12,*k21,*k22;
	double *b11,*b12,*b21,*b22;
	double *a11,*a12,*a21,*a22;
	a11=p[0];
	a22=p[1];
	k11=p[2];
	k22=p[3];

	a12=0;
	a21=0;
	k12=0;
	k21=0;
	b11=0;
	b12=0;
	b21=0;
	b22=0;

	/*Model constant values*/
	double m0,g,L,l_tri,Rp1,theta,phi,dtheta,dphi,pi;
	double m11,m12,m21,m22,inv_m11,inv_m12,inv_m21,inv_m22;
	double c11,c12,c21,c22;
	double g11,g21;
	double pm1,pm2,pm3;
	double tau_x,tau_y,tau_phi,tau_theta;
    pi=M_PI
	m0=0.35;     // segment weight kg
	g=9.8;       // gravity
	L=0.185;     // acutaor length
	l_tri=0.015;// segment length
	Rp1=deg2rad(240); // p1 offset angle

	// x[0] phi x[1] v_phi x[2] theta x[3] v_theta
	phi=x[0];
	dphi=x[1];
	theta=x[2];
	dtheta=x[3];

	pm1=u[0];    // Chamber 1 measured pressure
	pm2=u[1];    // Chamber 2 measured pressure
	pm3=u[2];    // Chamber 3 measured pressure

	tau_x=pow(3.0,0.5)/2*(pm1-pm3);
	tau_y=0.5*(pm1+pm3)-pm2;
	tau_phi = a11*(-cos(phi)*sin(theta)*tau_x - sin(phi)*tau_y);
	tau_theta= a22*(-sin(phi)*sin(theta)*tau_x + cos(phi)*tau_y);

	m11 = (pow(l_tri,2)*m0*pow(sin(pi/3 - (Rp1+phi)%(2*pi/3)),2)*pow(sin(theta/2),2))/(12*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),4)) - (5*m0*pow(sin(theta/2),2)*(cos(theta) - 1)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2))/2;
    
    m12 = (pow(3.0,(1/2))*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)))/(6*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),2)) - (m0*pow(cos(theta/2),2)*pow(sin(theta/2),4)*cos(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4)*(pow(cos(theta/2),2) - 1)*(pow(cos(theta/2),2) + sin(phi)*cos(phi) - 1))/2;
    
    m21 = (pow(3.0,(1/2))*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)))/(6*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),2)) - (m0*pow(cos(theta/2),2)*pow(sin(theta/2),4)*cos(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4)*(pow(cos(theta/2),2) - 1)*(pow(cos(theta/2),2) + sin(phi)*cos(phi) - 1))/2;
    
    m22 = m0*pow((cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)),2) + m0*pow(sin(theta/2),4)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + m0*pow(cos(theta/2),2)*pow(sin(theta/2),2)*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + m0*pow(cos(theta/2),2)*pow(sin(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + (m0*pow(sin(theta/2),2)*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),2)*pow(cos(theta/2),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/4 + (m0*pow(sin(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),2)*pow(sin(phi),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/4;
	    
	c11 =  - (5*m0*sin(theta/2)*(cos(theta) - 1)*(dtheta*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) + (pow(3.0,(1/2))*dphi*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2))/(6*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)))*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))))/2 - (5*dtheta*m0*sin(theta)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2))/4 - (pow(l_tri,2)*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*(3*sin(theta/2) - (3*cos(theta/2 + pi/6 - 2*(Rp1+phi)%(2*pi/3)))/4 + sin(theta/2 + pi/3 + 2*(Rp1+phi)%(2*pi/3))/4))/(24*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),5));

	c12 = (pow(3.0,(1/2))*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*(sin(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (2*L*cos(theta/2))/pow(theta,2) + (2*L*sin(theta/2))/pow(theta,3)))/(6*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)) - (cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*((3*dphi*m0*sin(theta/2)*(cos(theta) - 1)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))))/2 - dtheta*m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2)) - (pow(3.0,(1/2))*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*((dtheta*m0*sin(theta/2)*(36*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*(cos(theta)/2 - 1/2) - 72*pow(L,3)*pow(sin(theta/2),6)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(phi),3)*pow(sin(phi),3) - 54*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(theta/2),2)*(cos(theta)/2 + 1/2) + 18*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(phi),4)*(cos(theta)/2 - 1/2) - 54*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(sin(phi),2)*(cos(theta)/2 + 1/2) + 18*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(sin(phi),4)*(cos(theta)/2 - 1/2) - 6*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*(cos(theta)/2 - 1/2) + pow(3.0,(1/2))*pow(l_tri,3)*pow(theta,3)*pow(sin(theta/2),6)*pow(cos(phi),3)*pow(sin(phi),3) + 9*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(theta/2),2)*(cos(theta)/2 + 1/2) - 3*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(phi),4)*(cos(theta)/2 - 1/2) + 9*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(sin(phi),2)*(cos(theta)/2 + 1/2) - 3*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(sin(phi),4)*(cos(theta)/2 - 1/2) - 18*L*pow(l_tri,2)*pow(theta,2)*pow(sin(theta/2),6)*sin(pi/6 + (Rp1+phi)%(2*pi/3))*pow(cos(phi),3)*pow(sin(phi),3) + 36*pow(3.0,(1/2))*pow(L,2)*l_tri*theta*pow(sin(theta/2),6)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(phi),3)*pow(sin(phi),3)))/(36*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)) - 2*dphi*m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2)))/(12*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)) - dphi*m0*sin(theta/2)*(cos(theta) - 1)*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (dphi*((5*m0*sin(theta)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2))/2 + m0*pow(sin(theta/2),4)*sin(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2) - pow(2.0,(1/2))*m0*cos(2*phi + pi/4)*pow(sin(theta/2),4)*cos(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2)))/2 - m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*(dtheta*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) + (pow(3.0,(1/2))*dphi*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2))/(6*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)))*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2) - (dtheta*m0*cos(2*phi)*pow(sin(theta/2),4)*cos(phi)*sin(phi)*(96*pow(L,2)*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),4) + 8*pow(l_tri,2)*pow(theta,4)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2) - pow(l_tri,4)*pow(theta,4)*pow(sin(theta/2),4)*cos(phi)*sin(phi) - 32*pow(3.0,(1/2))*L*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3) - 144*pow(L,4)*pow(sin(theta/2),4)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),4)*cos(phi)*sin(phi) - 72*pow(L,2)*pow(l_tri,2)*pow(theta,2)*pow(sin(theta/2),4)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*cos(phi)*sin(phi) + 8*pow(3.0,(1/2))*L*pow(l_tri,3)*pow(theta,3)*pow(sin(theta/2),4)*sin(pi/6 + (Rp1+phi)%(2*pi/3))*cos(phi)*sin(phi) + 96*pow(3.0,(1/2))*pow(L,3)*l_tri*theta*pow(sin(theta/2),4)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*cos(phi)*sin(phi)))/(192*pow(theta,4)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),4)) - (pow(3.0,(1/2))*dtheta*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*(cos(theta)/2 - 1/2)*(3*L*cos((Rp1+phi)%(2*pi/3)) - pow(3.0,(1/2))*l_tri*theta + 3*pow(3.0,(1/2))*L*sin((Rp1+phi)%(2*pi/3))))/(72*theta*cos(pi/3 - (Rp1+phi)%(2*pi/3))*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2));

	c21 =  (dtheta*((m0*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2)*(sin(2*phi)*(cos(theta)/2 - 1/2) - pow(sin(theta/2),6)*pow(sin(phi),4)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 3*pow(sin(theta/2),6)*pow(cos(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2 - (m0*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2)*(sin(2*phi)*(cos(theta)/2 - 1/2) - pow(sin(theta/2),6)*pow(cos(phi),4)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 3*pow(sin(theta/2),6)*pow(cos(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2 + 2*m0*pow(cos(theta/2),3)*pow(sin(theta/2),5)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4) - m0*cos(phi)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2)*(cos(theta)/2 - pow(cos(theta/2),2)*(cos(theta)/2 - 1/2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 1/2) + m0*cos(phi)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2)*(cos(theta)/2 - pow(sin(phi),2)*(cos(theta)/2 - 1/2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 1/2) + 2*m0*cos(theta/2)*pow(sin(theta/2),5)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),4)*(cos(theta)/2 - 1/2)))/4 + (cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*((3*dphi*m0*sin(theta/2)*(cos(theta) - 1)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))))/2 - dtheta*m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2)) + (5*dphi*m0*sin(theta)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(cos(theta)/2 - 1/2))/4 + (pow(3.0,(1/2))*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2)*((dtheta*m0*sin(theta/2)*(36*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*(cos(theta)/2 - 1/2) - 72*pow(L,3)*pow(sin(theta/2),6)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(phi),3)*pow(sin(phi),3) - 54*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(theta/2),2)*(cos(theta)/2 + 1/2) + 18*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(cos(phi),4)*(cos(theta)/2 - 1/2) - 54*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(sin(phi),2)*(cos(theta)/2 + 1/2) + 18*L*pow(theta,2)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)*pow(sin(phi),4)*(cos(theta)/2 - 1/2) - 6*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*(cos(theta)/2 - 1/2) + pow(3.0,(1/2))*pow(l_tri,3)*pow(theta,3)*pow(sin(theta/2),6)*pow(cos(phi),3)*pow(sin(phi),3) + 9*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(theta/2),2)*(cos(theta)/2 + 1/2) - 3*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(phi),4)*(cos(theta)/2 - 1/2) + 9*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(sin(phi),2)*(cos(theta)/2 + 1/2) - 3*pow(3.0,(1/2))*l_tri*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(sin(phi),4)*(cos(theta)/2 - 1/2) - 18*L*pow(l_tri,2)*pow(theta,2)*pow(sin(theta/2),6)*sin(pi/6 + (Rp1+phi)%(2*pi/3))*pow(cos(phi),3)*pow(sin(phi),3) + 36*pow(3.0,(1/2))*pow(L,2)*l_tri*theta*pow(sin(theta/2),6)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)*pow(cos(phi),3)*pow(sin(phi),3)))/(72*pow(theta,3)*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)) - dphi*m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2)))/(6*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)) + dphi*m0*sin(theta/2)*(cos(theta) - 1)*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (pow(3.0,(1/2))*l_tri*m0*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*(3*sin(theta/2) - (3*cos(theta/2 + pi/6 - 2*(Rp1+phi)%(2*pi/3)))/4 + sin(theta/2 + pi/3 + 2*(Rp1+phi)%(2*pi/3))/4))/(12*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),3)) - m0*pow(sin(theta/2),3)*cos(phi)*((pow(2.0,(1/2))*sin(2*phi + pi/4))/2 - 1/2)*(dtheta*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) + (pow(3.0,(1/2))*dphi*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2))/(6*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2)))*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(cos(theta)/2 - 1/2)*(cos(theta)/2 + 1/2) + (pow(3.0,(1/2))*dtheta*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*(cos(theta)/2 - 1/2)*(3*L*cos((Rp1+phi)%(2*pi/3)) - pow(3.0,(1/2))*l_tri*theta + 3*pow(3.0,(1/2))*L*sin((Rp1+phi)%(2*pi/3))))/(72*theta*cos(pi/3 - (Rp1+phi)%(2*pi/3))*pow(sin(pi/6 + (Rp1+phi)%(2*pi/3)),2));

	c22 =   ((dtheta*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) + (pow(3.0,(1/2))*dphi*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2))/(6*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),2)))*(2*m0*pow(sin(theta/2),3)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))))/4 + ((cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*((dtheta*(2*m0*pow(sin(theta/2),3)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + m0*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(sin(phi),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(sin(theta/2),7)*pow(cos(phi),3)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3) + m0*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(cos(theta/2),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2))))/2 + 2*dphi*m0*pow(cos(theta/2),2)*pow(sin(theta/2),3)*cos(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(pow(cos(theta/2),2) - 1)*(pow(cos(theta/2),2) + sin(phi)*cos(phi) - 1)))/2 - (cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*((dtheta*(2*m0*pow(sin(theta/2),3)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + m0*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(sin(phi),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(sin(theta/2),7)*pow(cos(phi),3)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3) + m0*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(cos(theta/2),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2))))/4 + dphi*m0*pow(cos(theta/2),2)*pow(sin(theta/2),3)*cos(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3)*(pow(cos(theta/2),2) - 1)*(pow(cos(theta/2),2) + sin(phi)*cos(phi) - 1)) + (dtheta*(4*m0*cos(theta/2)*pow(sin(theta/2),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + (m0*pow(sin(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(2*cos(theta/2)*sin(theta/2)*pow(sin(phi),2) - 2*cos(theta/2)*sin(theta/2) + 4*cos(theta/2)*pow(sin(theta/2),5)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2 - 4*m0*cos(theta/2)*pow(sin(theta/2),3)*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) - 4*m0*cos(theta/2)*pow(sin(theta/2),3)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + (m0*pow(sin(theta/2),2)*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(2*cos(theta/2)*sin(theta/2)*pow(cos(theta/2),2) - 2*cos(theta/2)*sin(theta/2) + 4*cos(theta/2)*pow(sin(theta/2),5)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2))/8 + (dphi*((m0*pow(sin(theta/2),2)*pow(cos(theta/2),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),6)*pow(sin(phi),4)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 2*pow(sin(theta/2),2)*cos(phi)*sin(phi) - 3*pow(sin(theta/2),6)*pow(cos(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2 - (m0*pow(sin(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),6)*pow(cos(phi),4)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2) + 2*pow(sin(theta/2),2)*cos(phi)*sin(phi) - 3*pow(sin(theta/2),6)*pow(cos(theta/2),2)*pow(sin(phi),2)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)))/2 + m0*pow(sin(theta/2),2)*cos(phi)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),2)*pow(cos(theta/2),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)) - m0*pow(sin(theta/2),2)*cos(phi)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)*(pow(sin(theta/2),2)*pow(sin(phi),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2))))/4 + ((dtheta*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) + (pow(3.0,(1/2))*dphi*l_tri*sin(pi/3 - (Rp1+phi)%(2*pi/3))*sin(theta/2))/(6*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),2)))*(2*m0*pow(sin(theta/2),3)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + m0*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(sin(phi),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*pow(cos(phi),3)*sin(phi)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2)) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(cos(theta/2),2)*sin(theta/2)*pow(sin(phi),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) + 2*m0*pow(sin(theta/2),7)*pow(cos(phi),3)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),3) + m0*sin(theta/2)*pow(cos(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3))))*(pow(sin(theta/2),2)*pow(cos(theta/2),2) + pow(cos(theta/2),2) + pow(sin(theta/2),6)*cos(phi)*pow(sin(phi),3)*pow((L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))),2))))/4 + m0*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2))*(sin(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (2*L*cos(theta/2))/pow(theta,2) + (2*L*sin(theta/2))/pow(theta,3));

	g11=(pow(3.0,(1/2))*g*l_tri*m0*sin(pi/3 - (Rp1+phi)%(2*pi/3))*cos(theta/2)*sin(theta/2))/(6*pow(cos(pi/3 - (Rp1+phi)%(2*pi/3)),2));

	g21=g*m0*cos(theta/2)*(cos(theta/2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))) - (L*sin(theta/2))/pow(theta,2)) - (g*m0*pow(sin(theta/2),2)*(L/theta - (pow(3.0,(1/2))*l_tri)/(6*cos(pi/3 - (Rp1+phi)%(2*pi/3)))))/2;


	inv_m11=1.0/(m11*m22-m12*m21)*m22;
	inv_m12=1.0/(m11*m22-m12*m21)*m21*(-1);
	inv_m21=1.0/(m11*m22-m12*m21)*m12*(-1);
	inv_m22=1.0/(m11*m22-m12*m21)*m11;

	dx[0]=x[1];
	dx[1]=inv_m11*(tau_phi-(c11*x[1]+c12*x[3])-(b11*x[1]+b12*x[3])-g11 - (k11*x[0]+k12*x[2])) + inv_m12*(tau_theta-(c21*x[1]+c22*x[3])-(b21*x[1]+b22*x[3]) - g21 - (k21*x[0]+k22*x[2]));
	dx[2]=x[3];
	dx[3]=inv_m21*(tau_phi-(c11*x[1]+c12*x[3])-(b11*x[1]+b12*x[3])-g11 - (k11*x[0]+k12*x[2])) + inv_m22*(tau_theta-(c21*x[1]+c22*x[3])-(b21*x[1]+b22*x[3]) - g21 - (k21*x[0]+k22*x[2]));
}

void compute_dy(double *y,double *x)
{
	y[0]=x[0];
	y[1]=x[2];
}