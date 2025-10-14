function robot_visu = visualizeSetup(robotURDF, tray_type, tray, container, ...
                                     toolOffset, toolRotationOffset, ...
                                     num_containers, p_7G_all)
% visualizeSetup - Creates and visualizes the robot with a tray and containers.
%
% Syntax:
%   robot_visu = visualizeSetup(robotURDF, tray_type, tray, container, ...
%                               toolRotationOffset, num_containers, ...
%                               p_7G_all, toolOffset)
%
% Inputs:
%   robotURDF           - string: path to URDF or XACRO file
%   tray_type           - string: 'plastic', 'wood', or 'custom'
%   tray                - struct: .length, .width, .thickness
%   container           - struct: .fill_level, .cyl_radius
%   toolRotationOffset  - matrix or vector: rotation offset of tool
%   num_containers      - int: number of containers on the tray
%   p_7G_all            - 3xN matrix: positions of container centers
%   toolOffset          - vector: translation offset of the tool
%
% Output:
%   robot_visu          - rigidBodyTree: visualized robot with tray setup

    % Import robot model
    robot_visu = importrobot(robotURDF);
    robot_visu.DataFormat = 'row';

    % Tray dimensions logic (adjust for plastic borders)
    borders = 0;
    if (strcmpi(tray_type, "plastic") && borders == 1)
        trayDimensions = [tray.length + 0.33, tray.width + 0.26, tray.thickness];
    else
        trayDimensions = [tray.length, tray.width, tray.thickness];
    end

    % Build and attach custom end-effector (tray and containers)
    tray_visu = trayExperiments(trayDimensions, toolRotationOffset, ...
                                container.fill_level, container.cyl_radius, ...
                                num_containers, p_7G_all', toolOffset);

    customEE = tray_visu.build();
    addSubtree(robot_visu, "axes_6", customEE);

    % Define initial joint configuration
    jointAngles = [0, 0, -pi/2, 0, -pi/2, 0];

    % Visualize the robot
    f_setup = figure();
    set(f_setup, 'Name', 'Robot Setup', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 480]);
    show(robot_visu, jointAngles, 'Frames', 'on', ...
         'PreservePlot', true, 'Visuals', 'on');
    hold on;
    title('Robot Setup');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 2]);
end
