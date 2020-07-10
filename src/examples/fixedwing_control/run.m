t=0;
[posc, velc, rotc, omegac] = local(t);
    [M, Thrust] = controller(posc, velc, rotc, omegac, t);