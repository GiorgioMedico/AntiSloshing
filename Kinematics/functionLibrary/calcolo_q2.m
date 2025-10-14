% Creata da Claudio Mazzotti

function q2 = calcolo_q2(robot,T_W0_TCP,q3,q4)
L2 = robot.links(1,3).d;
L3 = robot.links(1,5).d;
L4 = robot.links(1,7).d;

P_W = T_W0_TCP(1:3,4)-L4*T_W0_TCP(1:3,3);
P_W = P_W(1:3);

H = L2+L3*cos(q4);
K = L3*sin(q4)*cos(q3);
A_1 = sqrt(P_W(1)^2+P_W(2)^2-(L3*sin(q3)*sin(q4))^2);
A_2 = -sqrt(P_W(1)^2+P_W(2)^2-(L3*sin(q3)*sin(q4))^2);
B = P_W(3);

% sol 1
C2_1 = (B-(K*A_1/H))/((K^2/H)+H);
S2_1 = (A_1+C2_1*K)/H;

if isreal(C2_1) && isreal(S2_1)
    q2(1) = atan2(S2_1,C2_1);
    
else
    q2(1) = NaN;
end


% sol 2
C2_2 = (B-(K*A_2/H))/((K^2/H)+H);
S2_2 = (A_2+C2_2*K)/H;


if isreal(C2_2) && isreal(S2_2)
    q2(2) = atan2(S2_2,C2_2);
else
    q2(2) = NaN;
end
