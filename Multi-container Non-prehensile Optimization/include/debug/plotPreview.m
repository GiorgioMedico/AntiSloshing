% === Compute Values ===
    s_ver = linspace(0, 1, 1001);
    theta_ver = full(get_thz_0(s_ver));
    r_ver = full(get_p_06_0(s_ver));
    q_ver = full(get_q(s_ver));
    
    % Preallocate
    det_J_ver = zeros(1, 1001);
    q_ver_norm = zeros(6, 1001);
    
    % Compute determinant of Jacobian
    for i = 1:1001
        J_ver = full(get_Jg(s_ver(i)));
        det_J_ver(i) = det(J_ver);
    end
    
    % Normalize joint positions
    for i = 1:6
        for j = 1:1001
            if j < 1001 && q_ver(i,j+1)-q_ver(i,j) > 1.5*pi 
                q_ver(i,j+1) = q_ver(i,j+1) - 2*pi;
            end
            if j < 1001 && q_ver(i,j+1)-q_ver(i,j) < -1.5*pi 
                q_ver(i,j+1) = q_ver(i,j+1) + 2*pi;
            end
            if q_ver(i,j) < 0
                q_ver_norm(i,j) = q_ver(i,j)/abs(q_min(i));
            else
                q_ver_norm(i,j) = q_ver(i,j)/abs(q_max(i));
            end
        end
    end
    
    % === Create Figure with Subplots ===
    figure('Name', 'Joint Analysis Preview', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 600]);
    
    % --- Subplot 1: Normalized Joint Positions ---
    subplot(2,1,1)
    plot(s_ver, q_ver_norm, 'LineWidth', 1.5)
    title('Normalized Joint Positions', 'Interpreter', 'latex')
    xlabel('$s$ [-]', 'Interpreter', 'latex')
    ylabel('$q_j/q_{j,\mathrm{max}}$ [-]', 'Interpreter', 'latex')
    yline(1, '--k', 'LineWidth', 1.2)
    yline(-1, '--k', 'LineWidth', 1.2)
    grid on; box on;
    xlim([0, 1])
    ylim([-1.2, 1.2])
    legend({'$1$', '$2$', '$3$', '$4$', '$5$', '$6$'}, 'Interpreter', 'latex', 'Location', 'best')
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)
    
    % --- Subplot 2: Geometric Jacobian Determinant ---
    subplot(2,1,2)
    plot(s_ver, det_J_ver, 'LineWidth', 1.5)
    title('Geometric Jacobian Determinant', 'Interpreter', 'latex')
    xlabel('$s$ [-]', 'Interpreter', 'latex')
    ylabel('$\det(J)$', 'Interpreter', 'latex')
    grid on; box on;
    xlim([0, 1])
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)