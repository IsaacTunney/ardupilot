function q = YPRToQuat(r1, r2, r3)

% For ZYX, Yaw-Pitch-Roll
% psi   = RPY(0psi thet) = r1
% theta = RPY(1) = r2
% phi   = RPY(2) = r3

cr1 = cos(0.5*r1);
cr2 = cos(0.5*r2);
cr3 = cos(0.5*r3);
sr1 = sin(0.5*r1);
sr2 = sin(0.5*r2);
sr3 = sin(0.5*r3);

e0 = cr1*cr2*cr3 + sr1*sr2*sr3;
e1 = cr1*cr2*sr3 - sr1*sr2*cr3;
e2 = cr1*sr2*cr3 + sr1*cr2*sr3;
e3 = sr1*cr2*cr3 - cr1*sr2*sr3;

% e0,e1,e2,e3 = qw,qx,qy,qz
q = [e0;e1;e2;e3];
% q = q*sign(e0)

q = q/norm(q);