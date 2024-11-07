
function [Jac2] = jacob2(a1,q1)

Jac2 = [-a1*sin(q1)  0    0;
         a1*cos(q1)  0    0;
            0        0    0];
end
 