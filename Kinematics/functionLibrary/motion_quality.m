% Creata da Claudio Mazzotti

function motion_quality(robot,qc,t)

figure
subplot(2,1,1);
plot(t,robot.maniplty(qc,'T'))
xlabel('time')
% ylabel('maniplty translation')
title('maniplty translation')

subplot(2,1,2);
plot(t,robot.maniplty(qc,'R'))
xlabel('time')
% ylabel('maniplty rotation')
title('maniplty rotation')