% Creata da Claudio Mazzotti

function [q5,q6,q7] = calcolo_q5q6q7(robot,T_W0_TCP,q1,q2,q3,q4)


if isfinite(q1) && isfinite(q2) && isfinite(q3) && isfinite(q4)
    
T01 = trotz(q1)*transl(0,0,robot.links(1,1).d)*transl(robot.links(1,1).a,0,0)*trotx(robot.links(1,1).alpha);
T12 = trotz(q2)*transl(0,0,robot.links(1,2).d)*transl(robot.links(1,2).a,0,0)*trotx(robot.links(1,2).alpha);
T23 = trotz(q3)*transl(0,0,robot.links(1,3).d)*transl(robot.links(1,3).a,0,0)*trotx(robot.links(1,3).alpha);
T34 = trotz(q4)*transl(0,0,robot.links(1,4).d)*transl(robot.links(1,4).a,0,0)*trotx(robot.links(1,4).alpha);

T47 = (T01*T12*T23*T34)\T_W0_TCP;
R47 = T47(1:3,1:3);

% sol 1 --> q6:=(0,pi)
q5(1) = atan2(R47(2,3),R47(1,3));
q6(1) = atan2(sqrt(R47(1,3)^2+R47(2,3)^2),R47(3,3));
q7(1) = atan2(R47(3,2),-R47(3,1));

%sol 2 --> q6:=(-pi,0)
q5(2) = atan2(-R47(2,3),-R47(1,3));
q6(2) = atan2(-sqrt(R47(1,3)^2+R47(2,3)^2),R47(3,3));
q7(2) = atan2(-R47(3,2),R47(3,1));

else
   q5(1) = NaN; q5(2) = NaN;
   q6(1) = NaN; q6(2) = NaN;
   q7(1) = NaN; q7(2) = NaN;
end