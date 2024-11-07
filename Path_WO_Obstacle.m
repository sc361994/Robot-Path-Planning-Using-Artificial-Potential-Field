%%% Path planning of SCARA Manipulator Without Obstacle in the
close all; clear; clc

% Robot Parameter
a1 = 0.5;
a2 = 0.5;
d1 = 0.3;
d3 = 0;
n = 3;

% DH Parameter
alp=[0   0   pi];
a = [0  a1  a2];
d = [0  d1  d3];
th =[0   0   0];
% Joint type
J=[0 0 1];%0 for revolute and 1 for prismatic

qf =[pi/2;2*pi/3;0.3];

qi =[0*pi/180;5*pi/180;0.01]; % Initial 

elp = 10^(-4); % small value for the conversion
stp  = 0.0001;  % step size or increment of the iteration

% Motion from initial to final
 Sx5 = ([]);
 Sy5 = ([]);
 Sz5 = ([]);
q(:,1) = qi;
i = 2;
t = 1;
ind = 1;
figure('Name','Motion of SCARA W/O Obstacle','NumberTitle','off')
movegui('east')
while norm(qi-qf)>elp
    
    [fk,Or] = f_kine(n,J,alp,a,d,q(:,t),th);
    O3ds = Or(:,2)+fk((1:3),(1:3),2)*[a2;0;0];
    x1 = Or(1,1);    y1 = Or(2,1);   z1 = Or(3,1);
    x2 = 0;          y2 = 0;         z2 = d1;
    x3 = Or(1,2);    y3 = Or(2,2);   z3 = Or(3,2);
    x4 = O3ds(1);    y4 = O3ds(2);   z4 = O3ds(3);
    x5 = Or(1,3);    y5 = Or(2,3);   z5 = Or(3,3);

    Sx1 = [x1 x2];   Sy1 = [y1 y2];   Sz1 = [z1 z2];
    Sx2 = [x2 x3];   Sy2 = [y2 y3];   Sz2 = [z2 z3];
    Sx3 = [x3 x4];   Sy3 = [y3 y4];   Sz3 = [z3 z4];
    Sx4 = [x4 x5];   Sy4 = [y4 y5];   Sz4 = [z4 z5];
    
    Sx5(ind,1) = Or(1,3);  Sy5(ind,1)= Or(2,3); Sz5(ind,1)=Or(3,3);
    
    plot3(Sx1,Sy1,Sz1,Sx2,Sy2,Sz2,Sx3,Sy3,Sz3,Sx4,Sy4,Sz4,Sx5,Sy5,Sz5,'*r','linewidth',3,'MarkerSize',2) 
    axis([-0.63 1.01  -0.1  1  -0.1  0.33])
    grid on 
    xlabel('X Axis')
    ylabel('Y Axis')
    zlabel('Z Axis')
    drawnow;   
    
    q1i = qi(1);
    q2i = qi(2);
    d3i = qi(3);
    
    q1f = qf(1);
    q2f = qf(2);
    d3f = qf(3);
    
    % Intial and final goal position
    Org3f = Origin3(a1,a2,d1,d3f,q1f,q2f);
    Org3i = Origin3(a1,a2,d1,d3i,q1i,q2i);
    
    % Initial and final position of the second origin
    Org2i = Origin2(a1,d1,q1i);
    Org2f = Origin2(a1,d1,q1f);
    
    % influence of attraction for both the origins
    zta2 = 3;
    zta3 = 2.5;
    
    % Attractive Potential Field
    Fatt_3 = -(zta3*(Org3i-Org3f));
    Fatt_2 = -(zta2*(Org2i-Org2f));
    
    % Jacobian of the origins
    Jac3 = jacob3(a1,a2,q1i,q2i);
    Jac2 = jacob2(a1,q1i);
    
    % Joint forces or torques at every origin
    tau3 = Jac3'*Fatt_3;
    tau2 = Jac2'*Fatt_2;
    
    % Total joint torque acting on the manipulator
    tau = tau3+tau2;
    
    % Gradient Desecnt to reach from the initial to final goal
    qi = qi +stp*(tau/norm(tau)); 
    q(:,i) = qi;
    i = i+1;
    t = t+1;
    ind = ind+1;
    norm(qi-qf);
    
end