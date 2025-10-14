f_ee = figure();
set(f_ee, 'Name', 'End-Effector Results', 'NumberTitle', 'off', 'Position', [200, 200, 800, 480]);

t = tiledlayout(2,3, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(t, 'End-Effector Motion Results', 'Interpreter', 'latex', 'FontSize', 16);

% 1. Position
nexttile
hold on; grid on;
plot(tvec, p06_sol, 'LineWidth', 2)
title("Position", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$p$ [m]",Interpreter="latex")
legend('$x$','$y$','$z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 2. Velocity
nexttile
hold on; grid on;
plot(tvec, v06_sol, 'LineWidth', 2)
title("Velocity", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$v$ [m/s]",Interpreter="latex")
legend('$v_x$','$v_y$','$v_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 3. Acceleration
nexttile
hold on; grid on;
plot(tvec, a06_sol, 'LineWidth', 2)
title("Acceleration", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$a$ [m/$s^2$]",Interpreter="latex")
legend('$a_x$','$a_y$','$a_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

thee_sol = zeros(3,N+1);
for i=1:N+1
    thee_sol(:,i) = rotm2eul(R06_sol(:,3*(i-1)+1:3*i) ,"XYZ");
end
% Unwrap the Euler angles for each row
thee_sol_unwrapped = thee_sol;
for row = 1:3
    thee_sol_unwrapped(row,:) = unwrap(thee_sol(row,:));
end

% 4. Angular Position
nexttile
hold on; grid on;
plot(tvec, thee_sol_unwrapped, 'LineWidth', 2)
title("Angular Position", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\theta$ [rad]",Interpreter="latex")
legend('$\theta_x$','$\theta_y$','$\theta_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 5. Angular Velocity
nexttile
hold on; grid on;
plot(tvec, w06_sol, 'LineWidth', 2)
title("Angular Velocity", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\omega$ [rad/s]",Interpreter="latex")
legend('$\omega_x$','$\omega_y$','$\omega_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

% 6. Angular Acceleration
nexttile
hold on; grid on;
plot(tvec, w06_dot_sol, 'LineWidth', 2)
title("Angular Acceleration", "Interpreter","latex")
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel("$\dot{\omega}$ [rad/$s^2$]",Interpreter="latex")
legend('$\dot \omega_x$','$\dot \omega_y$','$\dot \omega_z$',Interpreter="latex",Location='northeast')
xlim([0 t_end])
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
