function th6 = calcolo_th6(robot,T06,th1,th5)

n = T06(1:3,1); nx = n(1); ny = n(2); nz = n(3);
o = T06(1:3,2); ox = o(1); oy = o(2); oz = o(3);
a = T06(1:3,3); ax = a(1); ay = a(2); az = a(3);
p = T06(1:3,4); px = p(1); py = p(2); pz = p(3);
R06 = T06(1:3,1:3);
d4 = robot.links(1,4).d;
d6 = robot.links(1,6).d;

c1 = cos(th1); 
s1 = sin(th1);
s5 = sin(th5);

if abs(s5) < 1.0e-05
    th6 = NaN;
else
    if isreal((-ox*s1 + oy*c1)/s5) && isreal((nx*s1 - ny*c1)/s5)
    th6 = atan2((-ox*s1 + oy*c1)/s5,(nx*s1 - ny*c1)/s5);
    else 
    th6 = NaN;
    end
end

% if th6 > pi
%         diff = th6 - pi;
%         th6 = -pi + diff;
% elseif th6 < -pi
%         diff = abs(th6 + pi);
%         th6 = pi - diff;
% end

end