function plotInitialFinalJointConfigs(robot_visu, frames_visu, q_start, q_end, path, T_start, T_end, T67)
% plotInitialFinalJointConfigs - Plots initial/final robot configurations and reference frames.
%
% Inputs:
%   robot_visu  - Rigid body tree model for visualization
%   frames_visu - String for showing robot frames, "on" or "off"
%   q_start     - Initial joint configuration (symbolic or numeric vector)
%   q_end       - Final joint configuration (symbolic or numeric vector)
%   path        - A CasADi Function of the form path(s), where s ∈ [0, 1], 
%                 and path(s) returns a 3x1 vector representing a point in 
%                 3D space.
%   T_start     - Initial transformation matrix from base to frame 6
%   T_end       - Final transformation matrix from base to frame 6
%   T67         - Transformation from frame 6 to 7 (e.g. flange to tray)
%
% This function creates:
%   - A subplot comparing initial and final robot configurations
%   - Frame axes plotted for base (0), flange (6), and tray/tool (7)

    % Convert joint angles
    jointAnglesStart = full(q_start(:))';
    jointAnglesEnd   = full(q_end(:))';

    % ------ Evaluate B-spline trajectory --------
    N_eval = 1000;
    lut_map = path.map(N_eval); 
    par_vec = linspace(0, 1, N_eval);
    val_mat = full(lut_map(par_vec));

    % ---- Plot robot configurations in subplots ----
    f_q_i_end = figure();
    set(f_q_i_end, 'Name', 'Initial and Final Joint Configuration', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 480]);

    for i = 1:2
        subplot(1, 2, i)
        if i == 1
            angles = jointAnglesStart;
            titleStr = 'Initial Joint Configuration';
            T6 = T_start;
        else
            angles = jointAnglesEnd;
            titleStr = 'Final Joint Configuration';
            T6 = T_end;
        end

        % Compute corresponding T07
        T07 = T6 * T67;

        % Extract frame origins
        O0 = [0; 0; 0];
        O6 = T6(1:3, 4);
        O7 = T07(1:3, 4);

        % Extract rotation matrices
        R0 = eye(3);
        R6 = T6(1:3, 1:3);
        R7 = T07(1:3, 1:3);

        % Show robot and trajectory
        show(robot_visu, angles, 'Frames', frames_visu, 'PreservePlot', true);
        hold on
        plot3(val_mat(1,:), val_mat(2,:), val_mat(3,:), 'LineWidth', 2)
        title(titleStr)
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis equal
        xlim([-0.8, 1.4])
        ylim([-0.8, 1.0])
        zlim([0, 1.8])
        grid on
        view(3)

        % Plot reference frames for current configuration
        plotFrame(O0, R0, '0');
        plotFrame(O6, R6, '6');
        plotFrame(O7, R7, '7');
    end
end
