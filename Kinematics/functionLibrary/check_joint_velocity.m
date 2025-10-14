% Creata da Claudio Mazzotti

function check_joint_velocity(qc,time_step,joint_max_velocity)

q1_D = (diff(qc(:,1)*180/pi)/time_step);
q2_D = (diff(qc(:,2)*180/pi)/time_step);
q3_D = (diff(qc(:,3)*180/pi)/time_step);
q4_D = (diff(qc(:,4)*180/pi)/time_step);
q5_D = (diff(qc(:,5)*180/pi)/time_step);
q6_D = (diff(qc(:,6)*180/pi)/time_step);
q7_D = (diff(qc(:,7)*180/pi)/time_step);

if all(q1_D<= abs(joint_max_velocity(1)) & q2_D<= abs(joint_max_velocity(2)) & q3_D<= abs(joint_max_velocity(3)) & q4_D<= abs(joint_max_velocity(4)) & q5_D<= abs(joint_max_velocity(5)) & q6_D<= abs(joint_max_velocity(6)) & q7_D<= abs(joint_max_velocity(7)))
disp(['TEST VELOCITA'' DEI GIUNTI: OK'])
else
disp(['TEST VELOCITA'' DEI GIUNTI:'])
end

if all(q1_D<= abs(joint_max_velocity(1)))
else
    disp(['Velocità J1 superata'])
end

if all(q2_D<= abs(joint_max_velocity(2)))
else
    disp(['Velocità J2 superata'])
end

if all(q3_D<= abs(joint_max_velocity(3)))
else
    disp(['Velocità J3 superata'])
end

if all(q4_D<= abs(joint_max_velocity(4)))
else
    disp(['Velocità J4 superata'])
end

if all(q5_D<= abs(joint_max_velocity(5)))
else
    disp(['Velocità J5 superata'])
end

if all(q6_D<= abs(joint_max_velocity(6)))
else
    disp(['Velocità J6 superata'])
end

if all(q7_D<= abs(joint_max_velocity(7)))
else
    disp(['Velocità J7 superata'])
end