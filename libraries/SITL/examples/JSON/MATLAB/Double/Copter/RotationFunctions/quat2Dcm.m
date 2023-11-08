function dcm = quat2Dcm(q)

e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);

dcm = zeros(3,3);

dcm(1,1) = e0^2 + e1^2 - e2^2 - e3^2;
dcm(1,2) = 2*(e1*e2 - e0*e3);
dcm(1,3) = 2*(e1*e3 + e0*e2);
dcm(2,1) = 2*(e1*e2 + e0*e3);
dcm(2,2) = e0^2 - e1^2 + e2^2 - e3^2;
dcm(2,3) = 2*(e2*e3 - e0*e1);
dcm(3,1) = 2*(e1*e3 - e0*e2);
dcm(3,2) = 2*(e2*e3 + e0*e1);
dcm(3,3) = e0^2 - e1^2 - e2^2 + e3^2;