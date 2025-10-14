f_tray = figure();
set(f_tray, 'Name', 'Tray Results', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);

t = tiledlayout(2,3, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(t, 'Tray Motion Results', 'Interpreter', 'latex', 'FontSize', 16);

% 1. Position
nexttile
hold on; grid on;
plot(tvec, p07_sol, 'LineWidth', 2)
title("Position", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$p$ [m]",Interpreter="latex")
legend('$x$','$y$','$z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 2. Velocity
nexttile
hold on; grid on;
plot(tvec, v07_sol, 'LineWidth', 2)
title("Velocity", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$v$ [m/s]",Interpreter="latex")
legend('$v_x$','$v_y$','$v_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 3. Acceleration
nexttile
hold on; grid on;
plot(tvec, a07_sol, 'LineWidth', 2)
title("Acceleration", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$a$ [m/$s^2$]",Interpreter="latex")
legend('$a_x$','$a_y$','$a_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)


% th_tray_sol = zeros(3,length(thz_sol));
% th_tray_sol(3,:) = thz_sol;
th_tray_sol = zeros(3,length(thz_sol));
for i=1:N+1
    th_tray_sol(:,i) = rotm2eul(R07_sol(:,3*(i-1)+1:3*i) ,"XYZ");
end
% Unwrap the Euler angles for each row
th_tray_sol_unwrapped = th_tray_sol;
for row = 1:3
    th_tray_sol_unwrapped(row,:) = unwrap(th_tray_sol(row,:));
end

% 4. Angular Position
nexttile
hold on; grid on;
plot(tvec, th_tray_sol_unwrapped, 'LineWidth', 2)
title("Angular Position", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\theta$ [rad]",Interpreter="latex")
legend('$\theta_x$','$\theta_y$','$\theta_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 5. Angular Velocity
nexttile
hold on; grid on;
plot(tvec, w07_sol, 'LineWidth', 2)
title("Angular Velocity", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\omega$ [rad/s]",Interpreter="latex")
legend('$\omega_x$','$\omega_y$','$\omega_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 6. Angular Acceleration
nexttile
hold on; grid on;
plot(tvec, w07_dot_sol, 'LineWidth', 2)
title("Angular Acceleration", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\dot{\omega}$ [rad/$s^2$]",Interpreter="latex")
legend('$\dot \omega_x$','$\dot \omega_y$','$\dot \omega_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
