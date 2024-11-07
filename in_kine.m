
function [q1,q2,d3] = in_kine(X,Y,Z,d1,a1,a2)

% Inverse kinematics of the SCARA (3RRP) Manipulator)

d3 = d1-Z;

C2 = (X^2+Y^2-a1^2-a2^2)/(2*a1*a2);
S2 = sqrt(1-C2^2);

q2 = atan2(S2,C2);

q1 = atan2(Y,X)-atan2(a2*S2,a1+a2*C2);