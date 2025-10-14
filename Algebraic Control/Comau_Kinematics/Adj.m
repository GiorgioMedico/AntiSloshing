function [Adj] = Adj(robot,q)
a = [robot.a(1), robot.a(2), robot.a(3),0,0,0];
d = [robot.d(1) 0 0 robot.d(4) 0 robot.d(6)];
alpha = [robot.alpha(1) robot.alpha(2) robot.alpha(3) robot.alpha(4) robot.alpha(5) robot.alpha(6)];

Adj = eye(4);
for i=1:length(q)
    Adj = Adj*TDH(a(i),alpha(i),d(i),q(i));
end
end

