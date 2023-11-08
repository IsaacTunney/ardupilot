function q = RotToQuat(R)

R11 = R(1, 1);
R12 = R(1, 2);
R13 = R(1, 3);
R21 = R(2, 1);
R22 = R(2, 2);
R23 = R(2, 3);
R31 = R(3, 1);
R32 = R(3, 2);
R33 = R(3, 3);

% From page 68 of MotionGenesis book
tr = R11 + R22 + R33;

if tr > R11 && tr > R22 && tr > R33
    e0 = 0.5 * sqrt(1 + tr);
    r = 0.25 / e0;
    e1 = (R32 - R23) * r;
    e2 = (R13 - R31) * r;
    e3 = (R21 - R12) * r;
elseif R11 > R22 && R11 > R33
    e1 = 0.5 * sqrt(1 - tr + 2*R11);
    r = 0.25 / e1;
    e0 = (R32 - R23) * r;
    e2 = (R12 + R21) * r;
    e3 = (R13 + R31) * r;
elseif R22 > R33
    e2 = 0.5 * sqrt(1 - tr + 2*R22);
    r = 0.25 / e2;
    e0 = (R13 - R31) * r;
    e1 = (R12 + R21) * r;
    e3 = (R23 + R32) * r;
else
    e3 = 0.5 * sqrt(1 - tr + 2*R33);
    r = 0.25 / e3;
    e0 = (R21 - R12) * r;
    e1 = (R13 + R31) * r;
    e2 = (R23 + R32) * r;
end

% e0,e1,e2,e3 = qw,qx,qy,qz
q = [e0,e1,e2,e3];
% q = q*sign(e0);

q = q/norm(q);