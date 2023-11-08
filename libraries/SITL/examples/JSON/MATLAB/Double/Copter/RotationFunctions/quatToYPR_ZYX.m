function YPR = quatToYPR_ZYX(q)

e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);

YPR = threeaxisrot( 2*(e1*e2 + e0*e3), ...
                    e0^2 + e1^2 - e2^2 - e3^2, ... 
                    -2*(e1*e3 - e0*e2), ...
                    2*(e2*e3 + e0*e1), ...
                    e0^2 - e1^2 - e2^2 + e3^2);

% YPR = [Yaw, pitch, roll] = [psi, theta, phi]