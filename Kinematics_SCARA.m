
close all
clear
clc
%% Forward Kinematics of SCARA Manipulator RRP
n = 3;  % Degree of freedom
a1 = 0.5;
a2 = 0.5;
d1 = 0.3;
d3 = 0.2;  % Initial value of the third joint angle i.e prismatic joint

% DH Parameter
alp=[0   0   pi];
a = [0  a1  a2];
d = [0  d1  d3];
th =[0   0   0]; % initial first two joint angles i.e. revolute joint

% Joint type
J=[0 0 1];%0 for revolute and 1 for prismatic

qi=[0*pi/180;5*pi/180;0.01];%Initial JV

qf=[pi/2;2*pi/3;0.3];%Final JV

Tf = 10;
time = 0:0.01:10;
figure('Name','Motion of SCARA Manipulator(3RRP)','NumberTitle','off')
movegui('east')
for i=1:length(time)
    t = time(i);
    q = trajectory(qi,qf,t,Tf);
    [fk,Or] = f_kine(n,J,alp,a,d,q,th);
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
    
    Sx5(i,1) = Or(1,3);  Sy5(i,1)= Or(2,3); Sz5(i,1)=Or(3,3);

    t=num2str(t);
    plot3(Sx1,Sy1,Sz1,Sx2,Sy2,Sz2,Sx3,Sy3,Sz3,Sx4,Sy4,Sz4,Sx5,Sy5,Sz5,'*r','linewidth',4,'MarkerSize',2)
    axis([-0.63 1.01  -0.1  1  -0.1  0.33])
    grid on 
    xlabel('X Axis')
    ylabel('Y Axis')
    zlabel('Z Axis')
    title(['Current time T = ',t],'fontweight','normal','fontsize',10)
    drawnow;   
end
hold on;
%% Inverse Kinemtics 
for i=1:length(Sx5)
    X = Sx5(i);
    Y = Sy5(i);
    Z = Sz5(i);
    [q1,q2,d3] = in_kine(X,Y,Z,d1,a1,a2);
    in_q = [q1,q2,d3];
    [fki,Ori] = f_kine(n,J,alp,a,d,in_q,th);
    plot3(Ori(1,3),Ori(2,3),Ori(3,3),'ob','MarkerSize',5)
end
