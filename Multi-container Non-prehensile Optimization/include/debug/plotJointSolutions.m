f_joint = figure();
set(f_joint, 'Name', 'Joint Results', 'NumberTitle', 'off', 'Position', [200, 200, 900, 480]);

t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(t, 'Optimal Joint Motion', 'Interpreter', 'latex', 'FontSize', 16);

% Joint Trajectories
nexttile
hold on; grid on;
plot(tvec, q_sol, 'LineWidth', 2.0)
xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$\mathbf{q}$ [rad]', 'Interpreter', 'latex', 'FontSize', 14)
title('Joint Trajectories', 'Interpreter', 'latex', 'FontSize', 14)
legend({'${q_1}$','${q_2}$','${q_3}$','${q_4}$','${q_5}$','${q_6}$'}, ...
       'Interpreter','latex','FontSize',10,'Location','northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% Joint Velocities (normalized)
nexttile
hold on; grid on;
plot(tvec, q_dot_sol./q_dot_max, 'LineWidth', 2.0)
xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$|\dot{\mathbf{q}}/\dot{\mathbf{q}}_{\max}|$ [-]', ...
       'Interpreter', 'latex', 'FontSize', 14)
title('Normalized Joint Velocities', 'Interpreter', 'latex', 'FontSize', 14)
legend({'$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$'}, ...
       'Interpreter','latex','FontSize',10,'Location','northeast')
line([0 t_end], [1 1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1, 'HandleVisibility','off')
line([0 t_end], [-1 -1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1, 'HandleVisibility','off')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% Save figure if requested
if save_fig
    save_name = strcat(fig_name,'_joints.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(f_joint, 'Renderer', 'painters')  % Forces vector output where possible
    set(f_joint, 'Units', 'Inches');
    fig_pos = get(f_joint, 'Position');
    set(f_joint, 'PaperUnits', 'Inches');
    set(f_joint, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(f_joint, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    
    print(f_joint, '-dpdf', save_name)
end


% q_sol_diffdiff = zeros(6,N+1);
% q_sol_diffdiff(:,1) = (q_dot_sol(:,1))/(t_end/(N+1));
% for i=2:N
%     q_sol_diffdiff(:,i) = (q_dot_sol(:,i+1)-q_dot_sol(:,i))/(t_end/(N+1));
% end
% 
% % Comparison of q_dot_sol and numerical derivative of q_sol
% figure()
% hold on
% grid on
% title("$\ddot{q}_{sol}$", Interpreter="latex")
% plot(tvec, q_sol_diffdiff, 'LineWidth', 2); % Numerical derivative
% xlabel("t in s", Interpreter="latex")
% ylabel("$\dot{\textbf{q}}$ in rad/s", Interpreter="latex")
% xlim([0 t_end])
% fontsize(gca, 15, 'points')
% lgd = legend(Interpreter="latex");
% fontsize(lgd, 10, 'points')
% ax = gca;
% set(ax.XLabel, 'FontSize', 15)
% set(ax.YLabel, 'FontSize', 15)
