function [s,sd,sdd] = motion_law(s0,s1,sd0,sdd0,t)
%This function evaluates a rounded trapezoidal motion law with initial
%position value equal to s0 and final value equal to s1. Initial velocity
%and acceleration are defined by sd0 and sdd0. The time interval is defined
%by t.
    
    n = length(s0);
    T = t(end);

    t1 = t(t < 1./8*T)/T;
    t2 = t(1./8*T <= t & t < 3./8*T)/T;
    t3 = t(3./8*T <= t & t < 4./8*T)/T;
    t4 = t(4./8*T <= t & t < 5./8*T)/T;
    t5 = t(5./8*T <= t & t < 7./8*T)/T;
    t6 = t(7./8*T <= t & t <= 8./8*T)/T;
    
    h   = 2/(3*pi);
    hd  = h/(T);
    hdd = h/(T^2);

    q1 = h*(-1/16/pi + 2*pi*t1.^2 + 1/16/pi*cos(8*pi*t1));
    q2 = h*(-1/8/pi + pi/32*(1 - 16*t2 + 128*t2.^2));
    q3 = h*(-1/16/pi + pi/4*(-1 + 4*t3 + 8*t3.^2) + 1/16/pi*cos(8*pi*t3));
    q4 = -h*(-1/16/pi + pi/4*(5 - 20*t4 + 8*t4.^2) + 1/16/pi*cos(8*pi*t4));
    q5 = h*(1/8/pi + pi/32*(-65 + 240*t5 - 128*t5.^2));
    q6 = -h*(-1/16/pi + pi/2*(1 - 8*t6 + 4*t6.^2) + 1/16/pi*cos(8*pi*t6));
    
    qd1 = hd*(4*pi*t1 - 1/2*sin(8*pi*t1));
    qd2 = hd*(pi*(8*t2 - 1./2));
    qd3 = hd*(pi*(4*t3 + 1) - 1/2*sin(8*pi*t3));
    qd4 = -hd*(pi*(4*t4 - 5) - 1/2*sin(8*pi*t4));
    qd5 = hd*(pi*(-8*t5 + 15/2));
    qd6 = -hd*(pi*(4*t6 - 4) - 1/2*sin(8*pi*t6));

    qdd1 = 4*pi*hdd + 4*pi*hdd*sin(8*pi*t1 - pi/2);
    qdd2 = 8*pi*hdd*ones(1,length(t2));
    qdd3 = 4*pi*hdd + 4*pi*hdd*sin(8*pi*(t3 - 1./4) - pi/2);
    qdd4 = -4*pi*hdd + 4*pi*hdd*sin(8*pi*(t4 - 3./4) + pi/2);
    qdd5 = -8*pi*hdd*ones(1,length(t5));
    qdd6 = -4*pi*hdd + 4*pi*hdd*sin(8*pi*(t6 - 1) + pi/2);
    
    q   = [q1 q2 q3 q4 q5 q6];
    qd  = [qd1 qd2 qd3 qd4 qd5 qd6];
    qdd = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6];
    
    [s,sd,sdd] = scale(s0, s1, sd0, sdd0, q, qd, qdd, n);

end

function [s,sd,sdd] = scale(s0, s1, sd0, sdd0, q, qd, qdd, n)

step = s1 - s0;
s   = zeros(n,length(q));
sd  = zeros(n,length(q));
sdd = zeros(n,length(q));
for i = 1:n
    
    s(i,:)    =   q * step(i) + s0(i);
    sd(i,:)   =  qd * step(i) + sd0(i);
    sdd(i,:)  = qdd * step(i) + sdd0(i);
    
end


end