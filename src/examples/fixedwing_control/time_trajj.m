function [posd, veld, rot_des, omegad, controld] = time_trajj(t)

coder.inline('never');

a=10;
x=0; y=0; xdot=0; ydot=0; psid=0; phid=0; psidot_des=0; phidot_des=0; Mfwd=0;
z=0.5*a*t^2;
zdot=a*t;
thetad = t;
thetadot_des = t;
Tfwd = t;

posd = [x; y; z];
veld = [xdot; ydot; zdot];
rot_des=[psid; thetad; phid];
omegad=[psidot_des; thetadot_des; phidot_des];
controld=[Tfwd; Mfwd];
end
