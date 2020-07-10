function [posc, velc, rotc, omegac] = local(t)

coder.inline('never');

a=10;
xc=0; yc=0; xdotc=0; ydotc=0; psidc=0; thetadc=0; phidc=0;psidotc=0; thetadotc=0; phidotc=0;
zc=0.5*a*t^2;
zdotc=a*t;

posc = [xc; yc; zc];
velc = [xdotc; ydotc; zdotc];
rotc=[psidc; thetadc; phidc];
omegac=[psidotc; thetadotc; phidotc];
end
