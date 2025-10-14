function [q_sol] = SmartSix_IK_sym(robot,T06,sol)
a1 = robot.a(1); 
a2 = robot.a(2); 
a3 = robot.a(3); 

d1 = robot.d(1); 
d4 = robot.d(4); 
d6 = robot.d(6); 

R06 = T06(1:3,1:3);
p06 = T06(1:3,4);

p05 = p06 - R06(:,3)*d6;

s = p05(3) - d1;
d3w = sqrt(a3^2+d4^2);

switch sol
    case {0,1,4,5}
        q1 = atan2(p05(2),p05(1));
        r = sqrt(p05(1)^2+p05(2)^2) - a1;
    case {2,3,6,7}
        q1 = atan2(p05(2),p05(1)) + pi;
        r = sqrt(p05(1)^2+p05(2)^2) + a1;
    otherwise
        print("Invalid solution number");
end

d2w = sqrt(r^2+s^2);
phi = atan2(s, r);
gamma = acos((d2w^2+a2^2-d3w^2)/(2*d2w*a2));

switch sol
    case {0,4}
        q2 = - (phi+gamma);
    case {1,5}
        q2 = - (phi-gamma);
    case{2,6}
        q2 = - (pi - (phi+gamma));
    case{3,7}
        q2 = - (pi - (phi-gamma));
end

beta = atan2(d4,a3);
delta = acos((d3w^2+a2^2-d2w^2)/(2*d3w*a2));

switch sol
    case {0,3,4,7}
        q3 = -(beta-(pi-delta));
    case {1,2,5,6}
        q3 = -(beta+(pi-delta));
end

T03 = Adj(robot,[q1,q2,q3]);
R03 = T03(1:3,1:3);
R36 = R03'*R06;

switch sol
    case {0,1,2,3}
        q4 = atan2(R36(2,3),R36(1,3));
        q5 = - atan2(sqrt(R36(1,3)^2 + R36(2,3)^2),R36(3,3));
        q6 = atan2(R36(3,2),-R36(3,1));
    case {4,5,6,7}
        q4 = atan2(-R36(2,3),-R36(1,3));
        q5 = - atan2(-sqrt(R36(1,3)^2 + R36(2,3)^2),R36(3,3));
        q6 = atan2(-R36(3,2),R36(3,1));
end
q_sol = [q1,q2,q3,q4,q5,q6]';
for i=1:6
    q_sol(i) = robot.q_c0(i) + robot.S(i,i)*q_sol(i);
end


