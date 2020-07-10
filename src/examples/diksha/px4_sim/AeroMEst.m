function [ Moment_aero ] = AeroMEst( eul, x_dot,omega, Fa);
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
coder.inline('never');
BQ_x_ac = [0.0838;0;0];
BQ_x_cg = [0.157;0;0];
c_root=0.39;
c_tip=0.176;
t= c_tip/c_root;
BQ_c = c_root*(2/3)*((1+t+t^2)/(1+t));
BQ_b=2.29/2;
BQ_S=0.754/2;

BQ_Cm_0=-0.0156;
BQ_Cm_alpha=0.995;
BQ_Cm_q=-0.51;
BQ_Cl_beta=0;
BQ_Cl_p=-0.43;
BQ_Cl_r=0.29;
BQ_Cn_beta=0.0812;
BQ_Cn_p=-0.4044;
BQ_Cn_r=-0.05;

rho=1.225;
Moment_aero=[0;0;0];

R=eul2rotm(eul);
xb_dot=R'*x_dot;
V=norm(x_dot);

if (V==0) return;
end

Rq2w = [
    0 0 1;
    0 1 0;
    -1 0 0];
xw_dot = Rq2w*xb_dot;

if (xw_dot(1)~=0)
    alpha=atan2(-xw_dot(3),xw_dot(1));
    beta=atan2(xw_dot(2),xw_dot(1));
else
    alpha=0;
    beta=0;
end

omega_w = Rq2w*omega;
p_w = omega_w(1,1);
q_w = omega_w(2,1);
r_w = omega_w(3,1);

C_l = BQ_Cl_beta*beta + BQ_Cl_p*p_w*BQ_b/(2*V) + BQ_Cl_r*r_w*BQ_b/(2*V);
C_m = BQ_Cm_0 + BQ_Cm_alpha*alpha + BQ_Cm_q*q_w*BQ_c/(2*V);
C_n = BQ_Cn_beta*beta + BQ_Cn_p*p_w*BQ_b/(2*V) + BQ_Cn_r*r_w*BQ_b/(2*V);

Mx_w_ac = 0.5*rho*(V^2)*BQ_S*BQ_c*C_l ;
My_w_ac = 0.5*rho*(V^2)*BQ_S*BQ_c*C_m ;
Mz_w_ac = 0.5*rho*(V^2)*BQ_S*BQ_c*C_n ;

M_ac = [Mx_w_ac;My_w_ac;Mz_w_ac];% wing frame
% r = BQ.x_ac - BQ.x_cg;
% M_cg = M_ac + cross(r',(Rq2w*Fa)')';
r = BQ_x_ac(1) - BQ_x_cg(1);
% L = -Fa(1);
% M_cg = M_ac + [0;r*L;0]; % wing frame

% Moment_aero = BQ.wing_n*Rq2w'*M_cg; % body frame

Moment_aero = [0;2*My_w_ac+r*Fa(1);0];

end

