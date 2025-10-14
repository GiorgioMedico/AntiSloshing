% Creata da Claudio Mazzotti

function qLBRplot(t, q)
if nargin < 2
    q = t;
    t = (1:numrows(q))';
end
%clf
figure

subplot(2,1,1)
hold on
plot(t, q(:,[2 4 6]))
plot(t, -120*ones(size(t)),'k')
plot(t, 120*ones(size(t)),'k')
xlabel('time (sec)')
ylabel('q (gradi)')
legend('q2', 'q4', 'q6');
hold off

subplot(2,1,2)
hold on
plot(t, q(:,[1 3 5 7]), '--')
plot(t, -175*ones(size(t)),'k')
plot(t, 175*ones(size(t)),'k')
xlabel('time (sec)')
ylabel('q (gradi)')
legend('q1', 'q3', 'q5','q7');
hold off

% hold on
% plot(t, q(:,[2 4 6]))
% plot(t, q(:,[1 3 5 7]), '--')
% grid on
% xlabel('time')
% ylabel('q')
% %     legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6','q7');
% legend('q2', 'q4', 'q6', 'q1', 'q3', 'q5','q7');
% hold off

xlim([t(1), t(end)]);
