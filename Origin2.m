
function [Org2] = Origin2(a1,d1,q1)
Org2 = [a1*cos(q1);
        a1*sin(q1);
        d1];
end