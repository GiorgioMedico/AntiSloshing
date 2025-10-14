% Creata da Claudio Mazzotti


function [robot,joint_max_velocity] = create_lbr()

L1 = 0.36;%0.34 per il 7 kg %0.36 per il 14 kg
L2 = 0.42;%0.4 per il 7 kg %0.42 per il 14 kg
L3 = 0.4;% 0.4 uguale per entrambi i robot
L4 = 0.126;% 0.126 uguale per entrambi i robot

L(1) = Link([0,L1,0,-pi/2,0]);
L(2) = Link([0,0,0,pi/2,0]);
L(3) = Link([0,L2,0,pi/2,0]);
L(4) = Link([0,0,0,-pi/2,0]);
L(5) = Link([0,L3,0,-pi/2,0]);
L(6) = Link([0,0,0,pi/2,0]);
L(7) = Link([0,L4,0,0,0]);

robot = SerialLink(L, 'name', 'lbr14');

robot.qlim(1,1) = -170;
robot.qlim(1,2) = 170;
robot.qlim(2,1) = -120;
robot.qlim(2,2) = 120;
robot.qlim(3,1) = -170;
robot.qlim(3,2) = 170;
robot.qlim(4,1) = -120;
robot.qlim(4,2) = 120;
robot.qlim(5,1) = -170;
robot.qlim(5,2) = 170;
robot.qlim(6,1) = -120;
robot.qlim(6,2) = 120;
robot.qlim(7,1) = -175;
robot.qlim(7,2) = 175;
robot.qlim = robot.qlim*pi/180;

% base del robot inclinata a 45°
robot.base = troty(45,'deg');

% max valocità dei giunti per LBR 14 iiwa
% dati in gradi/sec
joint_max_velocity(1) = 98;% 85 per il 7kg
joint_max_velocity(2) = 98;% 85 per il 7kg
joint_max_velocity(3) = 100;% 100 per il 7kg
joint_max_velocity(4) = 130;% 75 per il 7kg
joint_max_velocity(5) = 140;% 130 per il 7kg
joint_max_velocity(6) = 180;% 135 per il 7kg
joint_max_velocity(7) = 180;% 135 per il 7kg