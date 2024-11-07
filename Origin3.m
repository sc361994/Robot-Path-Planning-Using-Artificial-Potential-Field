
function [Org3] = Origin3(a1,a2,d1,d3,q1,q2)

Org3 = [a2*cos(q1 + q2) + a1*cos(q1);
        a2*sin(q1 + q2) + a1*sin(q1);
        d1 - d3];
end