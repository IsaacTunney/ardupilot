function out = quatMultiply(q, p)

Q = [q(1), -q(2), -q(3), -q(4); ...
     q(2),  q(1), -q(4),  q(3); ...
     q(3),  q(4),  q(1), -q(2); ...
     q(4), -q(3),  q(2),  q(1)];

if size(p,1) < size(p,2)
    p = p';
end

out = Q*p; 
out = out';