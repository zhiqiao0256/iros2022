function par =funcEOMbaseFrame3D_addon(par,m_q)
% m_q=[phi theta/2 b_theta_phi b_theta_phi theta/2 ].';
J_f=[diff(m_q,theta).';diff(m_q,phi).'].';
temp.dJ_f=diff(subs(J_f,[theta, phi],[theta_t(t),phi_t(t)]),t);
dJ_fdt=subs(temp.dJ_f,[theta_t(t),diff(theta_t(t), t),phi_t(t),diff(phi_t(t), t)],[theta,dtheta,phi,dphi]);
par.J_xi2q=J_f;
temp.xi=m_q;
temp.dxi=(J_f*[dtheta;dphi]).';
temp.ddxi=(dJ_fdt*[dtheta;dphi]).'+(J_f*[ddtheta;ddphi]).';
% %%
B_xi_q=subs(D,xi,m_q);
par.sym_J_xi2q=subs(par.J_xyz{end},xi,m_q);
B_q=J_f.'*B_xi_q*J_f;
%%%
% M_xi_q=subs(M,xi,m_q);
% par.B_q_simplify=J_f.'*M_xi_q*J_f;
% %%
% temp_1=subs(cor,xi,f);
% temp_2=subs(temp_1,dxi,df);
% %%
% C_q=J_f.'*subs(D,xi,f)*J_ff+J_f.'*temp_2*J_f;
C_q=J_f.'*subs(D,xi,m_q)*dJ_fdt+J_f.'*subs(cor,[xi;dxi],[temp.xi;temp.dxi])*J_f;
% %%
G_q=J_f.'*subs(Phi,xi,m_q);
% %%
% par.J_xyz2q=subs(par.J_xyz2xi*par.J_xi2q,[xi],[m_q]);

% f_q=J_f.'*subs(par.f_xi,xi,f);
% 
% %%
% % par={};
par.B_q=B_q;
par.C_q=C_q;
par.G_q=G_q;
end