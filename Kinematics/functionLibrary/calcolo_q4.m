% Creata da Claudio Mazzotti

function q4 = calcolo_q4(robot,T_W0_TCP)
L2 = robot.links(1,3).d;
L3 = robot.links(1,5).d;
L4 = robot.links(1,7).d;

P_W = T_W0_TCP(1:3,4)-L4*T_W0_TCP(1:3,3);
P_W = P_W(1:3);


% O_S1 = [0,0,0]';
% L_23 = norm(P_W-O_S1);
% 
% alpha = acos(-(L3^2-L2^2-L_23^2)/(2*L_23*L2));
% C = L2*[cos(alpha),sin(alpha)]';
% B = L_23*[1,0]';
% 
% q4(1) = abs(acos(dot(C,B-C)/(L2*L3)));%q4>0
% q4(2) = -q4(1);%%q4<0



C4 = (P_W(1)^2+P_W(2)^2+P_W(3)^2-L2^2-L3^2)/(2*L2*L3);
S4 = sqrt(1-C4^2);

if isreal(C4) && isreal(S4)
q4(1) = atan2(S4,C4);
q4(2) = atan2(-S4,C4);
    
else
q4(1) = NaN;
q4(2) = NaN;
end





