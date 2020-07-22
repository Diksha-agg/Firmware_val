function [ R ] = eul2rotm( Euler )
    psi = Euler(1);		//yaw abtz
    theta = Euler(2);		//pitch abty
    phi = Euler(3);		//roll abtx
    
    R = [cos(theta)*cos(psi)    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)     sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)  ;
        cos(theta)*sin(psi)     cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi)      -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi) ;
        -sin(theta)             sin(phi)*cos(theta)                                 cos(phi)*cos(theta)                             ];
end

