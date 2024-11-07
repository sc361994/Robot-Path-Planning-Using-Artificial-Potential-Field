
function [Jac3] = jacob3(a1,a2,q1,q2)

Jac3 = [-a2*sin(q1 + q2)-a1*sin(q1)    -a2*sin(q1 + q2)    0;
         a2*cos(q1 + q2)+a1*cos(q1)     a2*cos(q1 + q2)    0;
                        0                 0               -1];
end