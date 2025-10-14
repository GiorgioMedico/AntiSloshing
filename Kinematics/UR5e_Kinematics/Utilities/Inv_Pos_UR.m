function q = Inv_Pos_UR(robot,T06,sol)
 
d1 = robot.d(1); 
a2 = robot.a(2); 
a3 = robot.a(3); 

d4 = robot.d(4); 
d5 = robot.d(5); 
d6 = robot.d(6); 
        
R06 = T06(1:3,1:3);
rE = T06(1:3,4);

rOO5_0 = rE + R06*[0 0 -d6]';
alpha1 = asin(d4/sqrt(rOO5_0(1)^2+rOO5_0(2)^2));

% q1 = pi + atan2(rOO5_0(2),rOO5_0(1)) - alpha1;

if sol == 1
    q1 = atan2(rOO5_0(2),rOO5_0(1)) - acos(d4/sqrt(rOO5_0(1)^2+rOO5_0(2)^2)) + pi/2;
elseif sol == 0
    q1 = atan2(rOO5_0(2),rOO5_0(1)) + acos(d4/sqrt(rOO5_0(1)^2+rOO5_0(2)^2)) + pi/2;
elseif sol == 2
    q1 = atan2(rOO5_0(2),rOO5_0(1)) - acos(d4/sqrt(rOO5_0(1)^2+rOO5_0(2)^2)) + pi/2;
elseif sol == 3
    q1 = atan2(rOO5_0(2),rOO5_0(1)) + acos(d4/sqrt(rOO5_0(1)^2+rOO5_0(2)^2)) + pi/2;
end

% if sol == 1
%     q1 = pi - atan2(rOO5_0(2),rOO5_0(1)) - alpha1;
% else
%     q1 = pi + atan2(rOO5_0(2),rOO5_0(1)) - alpha1;
% end
    
R01 = Rx(pi/2)*Ry(q1);
R10 = R01';

R16 = R10*R06;
z6_1 = R16*[0 0 1]';
z1_1 = [0 0 1]';

% q5 = acos(z6_1'*z1_1);
if sol == 1
    q5 = acos((rE(1)*sin(q1) - rE(2)*cos(q1) - d4)/d6);
%     q5 = 2*pi - acos((rE(1)*sin(q1) - rE(2)*cos(q1) - d4)/d6);
elseif sol == 0
    q5 = acos((rE(1)*sin(q1) - rE(2)*cos(q1) - d4)/d6);
elseif sol == 2
    q5 = 2*pi - acos((rE(1)*sin(q1) - rE(2)*cos(q1) - d4)/d6);
elseif sol == 3
    q5 = 2*pi - acos((rE(1)*sin(q1) - rE(2)*cos(q1) - d4)/d6);
end
    
% q6 = atan2(-R16(3,2),R16(3,1));
q6 = atan2(-R16(3,2)/sin(q5),R16(3,1)/sin(q5));

R56 = Rz(q6);
R65 = R56';
R15 = R16*R65;

rO1O3_1 = [0 0 -d4]' + R15*[0 d5 0]' + R16*[0 0 -d6]' + R10*rE + R10*[0 0 -d1]';

% q3 = pi - acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
if sol == 1
%     q3 = pi + acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
    q3 = - pi + acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
elseif sol == 0
    q3 = pi - acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
elseif sol == 2
    q3 = - pi + acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
elseif sol == 3
    q3 = pi - acos((a2^2+a3^2-rO1O3_1(1)^2-rO1O3_1(2)^2)/(2*a2*a3));
end

alpha2 = acos((a2^2-a3^2+(rO1O3_1(1)^2+rO1O3_1(2)^2))/(2*a2*sqrt(rO1O3_1(1)^2+rO1O3_1(2)^2)));
% gamma2 = atan2(abs(rO1O3_1(2)),abs(rO1O3_1(1)));
gamma2 = atan2(rO1O3_1(2),abs(rO1O3_1(1)));

if sol == 1
    q2 =  - (-gamma2 - alpha2 + pi);
elseif sol == 0
    q2 =  - gamma2 - alpha2;
elseif sol == 2
    q2 =  - (-gamma2 - alpha2 + pi);
elseif sol == 3
    q2 =  - gamma2 - alpha2;
end

R01 = Rx(pi/2)*Ry(q1);
R12 = Rz(q2);
R23 = Rz(q3);
R03 = R01*R12*R23;
R30 = R03';
R45 = Rx(-pi/2)*Ry(-q5);
R56 = Rz(q6);
R46 = R45*R56;
R64 = R46';

R36 = R30*R06;
R34 = R36*R64;

q4 = atan2(R34(2,1),R34(1,1));

q = [q1 q2 q3 q4 q5 q6]';

end