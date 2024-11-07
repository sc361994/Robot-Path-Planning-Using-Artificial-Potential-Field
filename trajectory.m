
function [q] = trajectory(qi,qf,t,T)
% 	cycloidal Trajectory
q = qi + ((qf-qi)/T)*(t-(T/(2*pi))*sin(2*pi*t/T));
end