clc
clear all
close all

robot = create_SmartSix();

q = [0 0 0 0 0 0]';
T06 = SmartSix_FK(q)

Jg = SmartSix_Jg(q)

% T06 = eye(4);
q_sol = SmartSix_IK(robot,T06,4)

for i = 1:8

    q_sol = SmartSix_IK(robot,T06,i-1);
    Q(i,:) = rad2deg(q_sol);

end

