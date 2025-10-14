function plotTrajectory(path)
% plotTrajectory - Plots a 3D geometric path defined by a CasADi function
%
% This function evaluates a parametric 3D path and visualizes it in 3D space.
% The path is assumed to be a CasADi function representing a B-spline or 
% continuous trajectory defined over the normalized domain s ∈ [0, 1].
%
% Input:
%   path - A CasADi Function of the form path(s), where s ∈ [0, 1], and
%          path(s) returns a 3x1 vector representing a point in 3D space.
%
% Details:
%   - The function samples the path at 1000 evenly spaced points along s.
%   - It uses path.map(N_eval) to build a lookup table that vectorizes
%     the evaluation for efficiency.
%   - The resulting 3D trajectory is plotted using plot3.
%   - Axes are labeled and equalized for better visual proportions.
%   - Z-axis and Y-axis are both limited to start from zero when possible.
%
% Example:
%   plotTrajectory(get_p_07_0);  % Where get_p_07_0 is a CasADi function handle

    % ------ Evaluating the B-spline at Multiple Points -----------
    N_eval = 1000;
    lut_map = path.map(N_eval); % maps the parametric values to the B-spline values
    par_vec = linspace(0, 1, N_eval); % Parametric values ranging from 0 to 1, evenly spaced
    val_mat = full(lut_map(par_vec)); %  Matrix containing the evaluated points of the B-spline trajectory, where each row corresponds to a point in 3D space
    
    % Plot 3D path
    figure()
    grid minor
    hold on
    view(3)
    plot3(val_mat(1,:),val_mat(2,:),val_mat(3,:),'LineWidth',2)
    title("Spline geometric path")
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    axis equal
    % Set axes limits starting from 0
    xlim([min(val_mat(1,:)) max(val_mat(1,:))])
    % ylim([0 max(val_mat(2,:))])
    ylim([min(val_mat(2,:)) max(val_mat(2,:))])
    zlim([0 max(val_mat(3,:))])

    % max(val_mat(3,:))-min(val_mat(3,:))
    % abs(max(val_mat(1,:)))-abs(min(val_mat(1,:)))
    % abs(max(val_mat(2,:)))-abs(min(val_mat(2,:)))


end

