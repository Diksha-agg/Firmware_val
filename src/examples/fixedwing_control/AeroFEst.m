function [ Fa] = AeroFEst( eul,x_dot,omega )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
coder.inline('never');
c_root=0.39;
c_tip=0.176;
t= c_tip/c_root;
BQ_c = c_root*(2/3)*((1+t+t^2)/(1+t));
BQ_b=2.29/2;
BQ_S=0.754/2;
BQ_CD_0=0.009;
BQ_CL0=0.4918;
BQ_CL_alpha=4.695;
BQ_CLq=0;
BQ_CY_beta=-0.951;
BQ_CY_p=0;
BQ_CY_r=0.008;

BQ_k=1/(pi*6.9*0.8);
BQ_M=50;
BQ_alpha0=20*(pi/180);
Fa=[0;0;0];
Fa_w=[0;0;0];
alpha=0;
beta=0;

p=omega(1,1);
q=omega(2,1);
r=omega(3,1);
rho=1.225;
% Eul=quat2eul((quat/norm(quat))');
% psi=Eul(1,1);
% theta=Eul(1,2);
% phi=Eul(1,3);
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

% alpha=atan2(-xw_dot(3),xw_dot(1));
% beta=atan2(xw_dot(2),xw_dot(1));

if (xw_dot(1)~=0)
    alpha=atan2(-xw_dot(3),xw_dot(1));
    beta=atan2(xw_dot(2),xw_dot(1));
else
    alpha=0;
    beta=0;
end
% alpha=atan2(xb_dot(1),xb_dot(3));
% beta=atan2(xb_dot(2),xb_dot(3));

% alpha = abs(alpha);
% beta = abs(beta);

A=[ sin(alpha)*cos(beta)    -sin(alpha)*sin(beta)      cos(alpha) ;
    sin(beta)               cos(beta)                 0       ;
    -cos(alpha)*cos(beta)   cos(alpha)*sin(beta)     sin(alpha)   ];

sigma_a = (1 + exp(-BQ_M*(alpha-BQ_alpha0)) + exp(BQ_M*(alpha+BQ_alpha0)))/(( 1 + exp(-BQ_M*(alpha-BQ_alpha0)))*(1 + exp(BQ_M*(alpha+BQ_alpha0))));


CLofalpha = (1-sigma_a)*(BQ_CL0+BQ_CL_alpha*alpha) + sigma_a*(2*sign(alpha)*(sin(alpha)^2)*cos(alpha));
%     CLofalpha = (2*sign(alpha)*(sin(alpha)^2)*cos(alpha));


CL = CLofalpha + BQ_CLq*(q*BQ_c/2*V);
CD = BQ_CD_0 + BQ_k*(CL^2) + 1*((sin(alpha))^2);

omega_w = Rq2w*omega;
p_w = omega_w(1,1);
q_w = omega_w(2,1);
r_w = omega_w(3,1);

CY = BQ_CY_beta*beta + BQ_CY_p*(p_w*BQ_b/(2*V)) + BQ_CY_r*(r_w*BQ_b/(2*V));

L = 0.5*rho*(V^2)*BQ_S*CL ;
D = 0.5*rho*(V^2)*BQ_S*CD ;
Y = 0.5*rho*(V^2)*BQ_S*CY ;

Fa=2*A*[-D;Y;-L];
Fa(2,1)=-Fa(2,1);
Fa(3,1)=-Fa(3,1);
Fa_w = [L;D;Y];

end

