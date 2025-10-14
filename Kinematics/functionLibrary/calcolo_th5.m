function th5 = calcolo_th5(robot,T06,th1)

n = T06(1:3,1); nx = n(1); ny = n(2); nz = n(3);
o = T06(1:3,2); ox = o(1); oy = o(2); oz = o(3);
a = T06(1:3,3); ax = a(1); ay = a(2); az = a(3);
p = T06(1:3,4); px = p(1); py = p(2); pz = p(3);
R06 = T06(1:3,1:3);
d4 = robot.links(1,4).d;
d6 = robot.links(1,6).d;

c1 = cos(th1); 
s1 = sin(th1);
p16_1y = -s1*px + c1*py;

if p16_1y > 0
    th5(1) =  -acos((abs(p16_1y) - d4)/d6);
    th5(2) =   acos((abs(p16_1y) - d4)/d6);
else 
    th5(1) =   acos((abs(p16_1y) - d4)/d6);
    th5(2) =  -acos((abs(p16_1y) - d4)/d6);
end

if abs(imag(th5)) < 1.0e-05
    th5 = real(th5);
end

% if th5(1) > pi
%         diff = th5(1) - pi;
%         th5(1) = -pi + diff;
% elseif th5(1) < -pi
%         diff = abs(th5(1) + pi);
%         th5(1) = pi - diff;
% end
% 
% if th5(2) > pi
%     diff = th5(2) - pi;
%     th5(2) = -pi + diff;
% elseif th5(2) < -pi
%         diff = abs(th5(2) + pi);
%         th5(2) = pi - diff;
% end


end