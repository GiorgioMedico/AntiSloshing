% Creata da Claudio Mazzotti

function q1 = calcolo_q1(robot,T_W0_TCP,q2,q3,q4)
L2 = robot.links(1,3).d;
L3 = robot.links(1,5).d;
L4 = robot.links(1,7).d;

P_W = T_W0_TCP(1:3,4)-L4*T_W0_TCP(1:3,3);
P_W = P_W(1:3);

H1 = L2*sin(q2)+L3*(sin(q2)*cos(q4)-cos(q2)*cos(q3)*sin(q4));
K1 = L3*sin(q3)*sin(q4);
A1 = P_W(2);
B1 = P_W(1);

C1 = (B1-(K1*A1/H1))/((K1^2/H1)+H1);
S1 = (A1+C1*K1)/H1;

q1 = atan2(S1,C1);