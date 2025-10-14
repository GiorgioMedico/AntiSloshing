function [th2, th3, th4] = calcolo_th2th3th4(robot,T06,th1,th6)

if isreal(th1) && isreal(th6)
    n = T06(1:3,1); nx = n(1); ny = n(2); nz = n(3);
    o = T06(1:3,2); ox = o(1); oy = o(2); oz = o(3);
    a = T06(1:3,3); ax = a(1); ay = a(2); az = a(3);
    p = T06(1:3,4); px = p(1); py = p(2); pz = p(3);
    R06 = T06(1:3,1:3);
    d1 = robot.links(1,1).d;
    a2 = robot.links(1,2).a;
    a3 = robot.links(1,3).a;
    d4 = robot.links(1,4).d;
    d5 = robot.links(1,5).d;
    d6 = robot.links(1,6).d;
    alpha6 = robot.links(1,5).alpha;

    c1 = cos(th1); 
    s1 = sin(th1);
    R56 = rotx(alpha6)*rotz(th6);

    R65 = R56';
    R01 = rotz(th1);
    R10 = R01';

    p15_1x = c1*(px - d6*ax) + s1*(py - d6*ay);
    p15_1z = pz - d6*az - d1;
%     p15_1x = c1*(px) + s1*(py);
%     p15_1z = pz - d1;
    z5_5 = [0; 0; 1];
    z5_6 = R65*z5_5;
    z5_0 = R06*z5_6;
    z5_1 = R10*z5_0;

    thP = atan2(z5_1(3),z5_1(1));

    px3R = p15_1x - d5*cos(thP);
    pz3R = p15_1z - d5*sin(thP);
%     px3R = p15_1x;
%     pz3R = p15_1z;
%     px3R = p15_1x - d5*sin(thP);
%     pz3R = p15_1z - d5*cos(thP);



    th3(1) = acos((px3R^2 + pz3R^2 - a2^2 - a3^2)/(2*a2*a3));
    A = a2 + a3*cos(th3(1)); 
    B = a3*sin(th3(1));
    num = pz3R - B*px3R/A;
    den = A + (B^2)/A;
    if isreal(num/den)
        th2(1) = atan2((num)/(den),(B/A)*(num)/(den) + (px3R/A));
%         th4(1) = thP - th2(1) - th3(1);    
        th4(1) = -pi/2 + thP - th2(1) - th3(1); 
    else
        th2(1) = NaN; 
        th3(1) = NaN; 
        th4(1) = NaN; 
    end
    
    th3(2) = -acos((px3R^2 + pz3R^2 - a2^2 - a3^2)/(2*a2*a3));
    A = a2 + a3*cos(th3(2)); 
    B = a3*sin(th3(2));
    num = pz3R - B*px3R/A;
    den = A + (B^2)/A;
    if isreal(num/den)
        th2(2) = atan2((num)/(den),(B/A)*(num)/(den) + (px3R/A));
%         th4(2) = thP - th2(2) - th3(2);   
        th4(2) = -pi/2 + thP - th2(2) - th3(2); 
    else
        th2(2) = NaN; 
        th3(2) = NaN; 
        th4(2) = NaN; 
    end
    
    
else 
    th2(1) = NaN; th2(2) = NaN;
    th3(1) = NaN; th3(2) = NaN;
    th4(1) = NaN; th4(2) = NaN;
    
end

% if th2(1) > pi
%         diff = th2(1) - pi;
%         th2(1) = -pi + diff;
% elseif th2(1) < -pi
%         diff = abs(th2(1) + pi);
%         th2(1) = pi - diff;
% end
% 
% if th2(2) > pi
%     diff = th2(2) - pi;
%     th2(2) = -pi + diff;
% elseif th2(2) < -pi
%         diff = abs(th2(2) + pi);
%         th2(2) = pi - diff;
%     end
% 
% if th3(1) > pi
%         diff = th3(1) - pi;
%         th3(1) = -pi + diff;
% elseif th3(1) < -pi
%         diff = abs(th3(1) + pi);
%         th3(1) = pi - diff;
% end
% 
% if th3(2) > pi
%     diff = th3(2) - pi;
%     th3(2) = -pi + diff;
% elseif th3(2) < -pi
%         diff = abs(th3(2) + pi);
%         th3(2) = pi - diff;
% end
% 
% if th4(1) > pi
%         diff = th4(1) - pi;
%         th4(1) = -pi + diff;
% elseif th4(1) < -pi
%         diff = abs(th4(1) + pi);
%         th4(1) = pi - diff;
% end
% 
% if th4(2) > pi
%     diff = th4(2) - pi;
%     th4(2) = -pi + diff;
% elseif th4(2) < -pi
%         diff = abs(th4(2) + pi);
%         th4(2) = pi - diff;
% end   

end