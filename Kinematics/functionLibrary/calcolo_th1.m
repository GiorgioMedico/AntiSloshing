function th1 = calcolo_th1(robot,T06)

n = T06(1:3,1); nx = n(1); ny = n(2); nz = n(3);
o = T06(1:3,2); ox = o(1); oy = o(2); oz = o(3);
a = T06(1:3,3); ax = a(1); ay = a(2); az = a(3);
p = T06(1:3,4); px = p(1); py = p(2); pz = p(3);
R06 = T06(1:3,1:3);
d4 = robot.links(1,4).d;
d6 = robot.links(1,6).d;

p05_0x = px - d6*ax;
p05_0y = py - d6*ay;
R = sqrt(p05_0x^2 + p05_0y^2);

if (px == 1300 & py == d4 + d6)
    th1(1) = pi;
    th1(2) = -pi;
else
    th1(1) = atan2(p05_0y,p05_0x) + asin(d4/R);
%     th1(2) = atan2(p05_0y,p05_0x) - asin(d4/R);
    th1(2) = (atan2(p05_0y,p05_0x) - asin(d4/R)) - pi;
end

% th1(1) = atan2(p05_0y,p05_0x) + acos(d4/R) + pi/2;
% th1(2) = atan2(p05_0y,p05_0x) - acos(d4/R) + pi/2;
% if th1(1) > pi
%         diff = th1(1) - pi;
%         th1(1) = -pi + diff;
% elseif th1(1) < -pi
%         diff = abs(th1(1) + pi);
%         th1(1) = pi - diff;
% end
% 
% if th1(2) > pi
%     diff = th1(2) - pi;
%     th1(2) = -pi + diff;
% elseif th1(2) < -pi
%         diff = abs(th1(2) + pi);
%         th1(2) = pi - diff;
% end

end

