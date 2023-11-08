function [xdot, ydot] = getXYdotFromUVflat(uFlat, vFlat, att)

e0 = att(1);
e1 = att(2);
e2 = att(3);
e3 = att(4);

a =  2*(e0*e3 - e1*e2);
b = (e0^2 - e1^2 + e2^2 - e3^2);
c = -2*(e0*e3 + e1*e2);
d = (e0^2 + e1^2 - e2^2 - e3^2);

ydot = (vFlat-c*uFlat/b)/(d-c*a/b);
xdot = (uFlat-a*ydot)/b;


