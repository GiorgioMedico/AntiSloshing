% Creata da Roberto Di Leva


function [robot,joint_max_velocity] = create_ur()

% d1 = 128; 
% a2 = 612.7;
% a3 = 571.6;
% d4 = 163.891;
% d5 = 115.7;
% d6 = 85.7;
d1 = 180.7; 
a2 = 612.6;
a3 = 571.55;
d4 = 174.15;
d5 = 119.85;
d6 = 110.050;

L(1) = Link([0,d1,0,pi/2,0]);
L(2) = Link([pi,0,a2,0,0]);
L(3) = Link([0,0,a3,0,0]);
L(4) = Link([pi/2,d4,0,pi/2,0]);
L(5) = Link([0,d5,0,-pi/2,0]);
L(6) = Link([pi,d6,0,0,0]);

robot = SerialLink(L, 'name', 'ur10');

robot.offset = robot.theta;
robot.qlim(1,1) = -180;
robot.qlim(1,2) = 180;
robot.qlim(2,1) = -180;
robot.qlim(2,2) = 180;
robot.qlim(3,1) = -180;
robot.qlim(3,2) = 180;
robot.qlim(4,1) = -180;
robot.qlim(4,2) = 180;
robot.qlim(5,1) = -180;
robot.qlim(5,2) = 180;
robot.qlim(6,1) = -180;
robot.qlim(6,2) = 180;
robot.qlim = robot.qlim*pi/180;


% max valocità dei giunti per UR10
% dati in gradi/sec
joint_max_velocity(1) = 120;
joint_max_velocity(2) = 120;
joint_max_velocity(3) = 180;
joint_max_velocity(4) = 180;
joint_max_velocity(5) = 180;
joint_max_velocity(6) = 180;

end