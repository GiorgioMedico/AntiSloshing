% Final Summary Figure
f_summary = figure();
set(f_summary, 'Name', 'Final Results Summary', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 700]);

t = tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(t, fig_name, 'Interpreter', 'none', 'FontSize', 16)

%% 1. Sloshing height
nexttile(1)
hold on; grid on; box on
th_tray_sol = zeros(3,length(thz_sol));
for i=1:N+1
    th_tray_sol(:,i) = rotm2eul(R07_sol(:,3*(i-1)+1:3*i) ,"XYZ");
end
% Unwrap the Euler angles for each row
th_tray_sol_unwrapped = th_tray_sol;
for row = 1:3
    th_tray_sol_unwrapped(row,:) = unwrap(th_tray_sol(row,:));
end

plot(tvec, th_tray_sol_unwrapped, 'LineWidth', 2)
title("Angular Position", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\theta$ [rad]",Interpreter="latex")
legend('$\theta_x$','$\theta_y$','$\theta_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

%% 2. Joint trajectories
nexttile(2)
hold on; grid on; box on
plot(tvec, q_eval, 'LineWidth', 2)
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('${\textbf{q}}$ [rad]', 'Interpreter', 'latex')
legend(arrayfun(@(i) sprintf('$q_{%d}$', i), 1:size(q_eval,1), 'UniformOutput', false), ...
       'Interpreter', 'latex', 'Location', 'northeast')
title('Joint Trajectories', 'Interpreter', 'latex')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)


%% 3. Geometric Path (3D)
nexttile(3)
hold on; grid on; box on
plot3(p07_sol(1,:), p07_sol(2,:), p07_sol(3,:), 'LineWidth', 2)
xlabel('$x$ [m]', 'Interpreter', 'latex')
ylabel('$y$ [m]', 'Interpreter', 'latex')
zlabel('$z$ [m]', 'Interpreter', 'latex')
title('Geometric Path', 'Interpreter', 'latex')
axis equal
view(3)           % Set 3D view
rotate3d on       % Enable interactive rotation
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)


%% 4. Summary values
nexttile(4)
axis off
text(0, 1.0, sprintf('Computation Time: %.2f s', t_calc), 'Interpreter', 'latex', 'FontSize', 14)
text(0, 0.85, sprintf('End Time: %.2f s', tvec(N+1)), 'Interpreter', 'latex', 'FontSize', 14)
text(0, 0.70, sprintf('Max Acceleration: %.2f m/s$^2$', max(vecnorm(a07_sol))), 'Interpreter', 'latex', 'FontSize', 14)
text(0, 0.55, sprintf('Max Angular Acc.: %.2f rad/s$^2$', max(vecnorm(w07_dot_sol))), 'Interpreter', 'latex', 'FontSize', 14)
text(0, 0.40, sprintf('$\\theta_z(0)$: %.2f rad', thz_0), 'Interpreter', 'latex', 'FontSize', 14)
text(0, 0.25, sprintf('$\\theta_z(t_e)$: %.2f rad', thz_end), 'Interpreter', 'latex', 'FontSize', 14)
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)


%% Save fig 
if save_fig
    save_name = strcat(fig_name,'_Summary.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(f_summary, 'Renderer', 'painters')  % Forces vector output where possible
    set(f_summary, 'Units', 'Inches');
    fig_pos = get(f_summary, 'Position');
    set(f_summary, 'PaperUnits', 'Inches');
    set(f_summary, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(f_summary, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    
    print(f_summary, '-dpdf', save_name)
end