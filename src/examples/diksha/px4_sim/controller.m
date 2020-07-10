function [Thrust, M] = controller(posc, velc, rotc, omegac , posd, veld, rotd, omegad, controld)
coder.inline('never');
%desired inputs:[xd yd zd xd_dot yd_dot zd_dot phid thetad psid phidotd thetadotd psidotd ud(1)---Tfwd
%ud(2)---Mfwd
Thrust=0;
BQ_m=12;%kg
BQ_g=9.81;
BQ_J=[1.86 0 0;
    0 2.031 0;
    0 0 3.617];
ud=controld;

omega_curr=[1 0 -sin(rotc(2));0 cos(rotc(3)) cos(rotc(2))*sin(rotc(3));0 -sin(rotc(3)) cos(rotc(2))*cos(rotc(3))]*[omegac(3); omegac(2); omegac(1)];
omega_des=[1 0 -sin(rotd(2));0 cos(rotd(3)) cos(rotd(2))*sin(rotd(3));0 -sin(rotd(3)) cos(rotd(2))*cos(rotd(3))]*[omegad(3); omegad(2); omegad(1)];

kp=eye(3);
kp(1,1)=5;
kp(2,2)=5;
kp(3,3)=50;

kv=eye(3);
kv(1,1)=50;     
kv(2,2)=6;
kv(3,3)=5;

kr=eye(3);
kr(1,1)=3;
kr(2,2)=100;
kr(3,3)=5;

kw=eye(3);
kw(1,1)=5;
kw(2,2)=1500;
kw(3,3)=1;

Rb=eul2rotm(rotc);

R=eul2rotm(rotd);
Fa=AeroFEst([0 rotc(2) 0],[velc(1);0;velc(3)],omega_curr);
acc_net=((R*[0;0;ud(1)])/BQ_m)+kp*[0;0;(posd(3)-posc(3))]+kv*[(veld(1)-velc(1));-velc(2);(veld(3)-velc(3))]+[0; 0; BQ_g]-((Rb*Fa)/(BQ_m)); % acc_net

% calculation of current euler angles
b3=acc_net/norm(acc_net);
c2=[-sin(rotd(1)) cos(rotd(1)) 0]';
b1=cross(c2,b3)/norm(cross(c2,b3));
b2=cross(b3,b1);
Rd=[b1 b2 b3];
nm=norm(acc_net);
%thrust control input
%flag=sign([0 0 1]*Rb'*acc_net);
Thrust=(BQ_m*nm);           

R=eul2rotm(rotc);
erm=0.5*((Rd'*R)-(R'*Rd));
er=[erm(3,2);erm(1,3);erm(2,1)];
ew=(omega_curr-((R'*Rd)*omega_des));

tau_a=AeroMEst([0; rotc(2); 0],[velc(1);0;velc(3)],omega_curr, Fa);
M=[0;ud(2);0]-kr*er-kw*ew+cross(omega_curr,BQ_J*omega_curr)-BQ_J*cross(ew,(R'*Rd)*omega_des)-tau_a;    %moment input

end