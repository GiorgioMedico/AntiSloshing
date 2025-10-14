function [cylinderRadius, cylinderHeight, mass, comHeight] = getAndVisualizeContainerParametersTilt()
% getAndVisualizeContainerParameters creates an interactive GUI for container
% parameters and visualizes them dynamically.
%
% The GUI allows the user to input:
%   - Container radius
%   - Container height (the actual height of the container)
%   - Mass of the cylinder (including contents)
%   - Center of Mass Height (Z-coordinate of the COM relative to container base)
%
% Default values are provided. The 3D plot updates in real-time as the
% user changes the input parameters.
%
% Returns:
%   cylinderRadius: The final container radius entered by the user.
%   cylinderHeight: The final container height entered by the user.
%   mass: The final mass entered by the user.
%   comHeight: The final center of mass height entered by the user.

    % --- Default Values ---
    defaultRadius = 0.035; % meters
    defaultCylinderHeight = 0.0991; % meters (actual height of the physical container)
    defaultMass = 0.3420; % kg (total mass of cylinder + contents)
    % Initialize defaultComHeight to half the defaultCylinderHeight as requested
    defaultComHeight = defaultCylinderHeight / 2; % meters (Center of Mass height relative to container base)

    % Initialize output variables with default values
    cylinderRadius = defaultRadius;
    cylinderHeight = defaultCylinderHeight;
    mass = defaultMass;
    comHeight = defaultComHeight;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Container Parameters & Visualization', ...
                   'Position', [100 100 800 580], ... % Adjusted figure height
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src, fig)); % Pass fig handle to closeFigure

    % Store data in the figure appdata
    appData = struct();
    appData.radius = defaultRadius;
    appData.cylinderHeight = defaultCylinderHeight;
    appData.mass = defaultMass;
    appData.comHeight = defaultComHeight; % Store new parameter
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---
    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Parameters', ...
                         'Position', [20 20 250 540], ... % Adjusted input panel height
                         'FontWeight', 'bold');

    % Container Radius Input
    uilabel(inputPanel, 'Text', 'Container radius (m):', ...
            'Position', [20 480 200 22]); % Adjusted Y
    radiusField = uieditfield(inputPanel, 'numeric', ...
                               'Value', defaultRadius, ...
                               'Position', [20 450 200 22], ... % Adjusted Y
                               'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'radius'));

    % Container Height Input
    uilabel(inputPanel, 'Text', 'Container height (m):', ...
            'Position', [20 400 200 22]); % Adjusted Y
    cylinderHeightField = uieditfield(inputPanel, 'numeric', ...
                                       'Value', defaultCylinderHeight, ...
                                       'Position', [20 370 200 22], ... % Adjusted Y
                                       'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'cylinderHeight'));

    % Mass Input
    uilabel(inputPanel, 'Text', 'Mass (kg):', ...
            'Position', [20 320 200 22]); % Adjusted Y
    massField = uieditfield(inputPanel, 'numeric', ...
                             'Value', defaultMass, ...
                             'Position', [20 290 200 22], ... % Adjusted Y
                             'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'mass'));

    % Center of Mass Height Input
    uilabel(inputPanel, 'Text', 'Center of Mass Height (m):', ...
            'Position', [20 240 200 22]); % Adjusted Y
    comHeightField = uieditfield(inputPanel, 'numeric', ...
                                  'Value', defaultComHeight, ...
                                  'Position', [20 210 200 22], ... % Adjusted Y
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'comHeight'));

    % OK Button
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [20 60 90 30], ... % Adjusted Y
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    % Cancel Button
    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [130 60 90 30], ... % Adjusted Y
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Create UIAxes for Plotting ---
    ax = uiaxes(fig, 'Position', [300 50 480 500]); % Adjusted axes position/height
    % Store axes handle in appdata for plotting function
    setappdata(fig, 'axesHandle', ax);

    % --- Initial Plot ---
    % Call the plotting function once to display the default state
    updateCylinderPlot(ax, appData.radius, appData.cylinderHeight, appData.comHeight);

    % --- Wait for User Action with Error Handling ---
    try
        uiwait(fig);
    finally
        % Check if figure is still valid before attempting to get appdata
        % This block ensures outputs are set even if figure is closed manually
        if isvalid(fig)
            finalAppData = getappdata(fig, 'appData'); % Retrieve data
            if finalAppData.confirmed
                cylinderRadius = finalAppData.radius;
                cylinderHeight = finalAppData.cylinderHeight;
                mass = finalAppData.mass;
                comHeight = finalAppData.comHeight;
            else
                % If cancelled or closed, return defaults
                cylinderRadius = defaultRadius;
                cylinderHeight = defaultCylinderHeight;
                mass = defaultMass;
                comHeight = defaultComHeight;
            end
            delete(fig); % Delete the figure after retrieving data
        else
            % If figure became invalid (e.g., deleted by other means),
            % outputs remain at their initial default values.
        end
    end

    if isvalid(fig) % Check if figure is still valid before attempting to get appdata
        finalAppData = getappdata(fig, 'appData'); % Retrieve data
        if finalAppData.confirmed
            cylinderRadius = finalAppData.radius;
            cylinderHeight = finalAppData.cylinderHeight;
            mass = finalAppData.mass;
            comHeight = finalAppData.comHeight;
        end
        delete(fig); 
    end
end

% --- Callback Function for Input Fields ---
function updateParameters(src, fig, paramName)
% updateParameters updates the stored parameter value and redraws the plot.
% src: The uieditfield component that triggered the callback.
% fig: The handle to the main UI figure.
% paramName: The name of the parameter ('radius', 'cylinderHeight', 'mass', 'comHeight').

    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    ax = getappdata(fig, 'axesHandle');

    % Get the new value from the edit field
    newValue = src.Value;

    % Basic validation: ensure positive values for radius, containerHeight.
    % Non-negative for mass and comHeight.
    if (strcmp(paramName, 'radius') && newValue <= 0) || ...
       (strcmp(paramName, 'cylinderHeight') && newValue <= 0) || ...
       (strcmp(paramName, 'mass') && newValue < 0) || ...
       (strcmp(paramName, 'comHeight') && newValue < 0)
        uialert(fig, 'Please enter valid positive numbers for radius and container height. Non-negative for mass and center of mass height.', 'Input Error');
        % Revert to previous valid value if invalid input
        src.Value = appData.(paramName);
        return;
    end

    % Update the stored parameter
    appData.(paramName) = newValue;
    setappdata(fig, 'appData', appData);

    % Redraw the plot with updated parameters if a visual parameter changed
    if any(strcmp(paramName, {'radius', 'cylinderHeight', 'comHeight'}))
        updateCylinderPlot(ax, appData.radius, appData.cylinderHeight, appData.comHeight);
    end
    drawnow; % Ensure GUI updates are processed immediately
end

% --- Callback Function for OK Button ---
function okButtonPushed(fig)
% okButtonPushed sets the confirmed flag and resumes the figure.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = true;
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Callback Function for Cancel Button ---
function cancelButtonPushed(fig)
% cancelButtonPushed ensures the confirmed flag is false and resumes the figure.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Explicitly set to false
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Custom Close Request Function ---
function closeFigure(src, fig) % Added src as first argument as per MATLAB's callback signature
% closeFigure handles closing the figure, ensuring uiresume is called.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Treat closing as cancellation
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Function to Plot the Container ---
function updateCylinderPlot(ax, radius, cylinderHeight, comHeight)
% updateCylinderPlot generates a 3D plot of a cylinder with a COM line.
% Inputs:
%   ax: The uiaxes object to plot on.
%   radius: The radius of the cylinder.
%   cylinderHeight: The actual height of the cylinder object.
%   comHeight: The Z-coordinate of the center of mass.

    cla(ax); % Clear the specified axes

    % --- Draw the Container ---
    % Generate cylinder data
    num_facets = 50; % Number of points for smooth circle (already 50, kept for clarity)
    [X, Y, Z_unit] = cylinder(radius, num_facets);

    % Scale Z for the cylinder body
    Z_cylinder_body = Z_unit * cylinderHeight;

    % Plot the cylinder surface (side)
    hCylBody = surf(ax, X, Y, Z_cylinder_body);
    set(hCylBody, 'FaceColor', [0.7 0.7 0.8], 'FaceAlpha', 0.5, 'EdgeColor', [0.4 0.4 0.4]); % Softer gray, more transparent

    hold(ax, 'on'); % Hold on for the specific axes

    % Draw the cylinder base (bottom disk)
    theta_base = linspace(0, 2*pi, num_facets);
    x_base = radius * cos(theta_base);
    y_base = radius * sin(theta_base);
    z_base = zeros(size(theta_base)); % At Z = 0
    fill3(ax, x_base, y_base, z_base, [0.7 0.7 0.8], 'FaceAlpha', 0.8, 'EdgeColor', [0.4 0.4 0.4]);

    % Draw the cylinder top (top disk)
    x_top = radius * cos(theta_base);
    y_top = radius * sin(theta_base);
    z_top = cylinderHeight * ones(size(theta_base)); % At Z = cylinderHeight
    fill3(ax, x_top, y_top, z_top, [0.7 0.7 0.8], 'FaceAlpha', 0.8, 'EdgeColor', [0.4 0.4 0.4]);


    % --- Draw the Center of Mass (COM) height ---
    % COM height should be within the container's height range
    if comHeight >= 0 && comHeight <= cylinderHeight
        theta_com_line = linspace(0, 2*pi, 100);
        x_com_line = radius * cos(theta_com_line);
        y_com_line = radius * sin(theta_com_line);
        z_com_line = comHeight * ones(size(theta_com_line));
        plot3(ax, x_com_line, y_com_line, z_com_line, 'k--', 'LineWidth', 1.5); % Black dashed line for COM
        % Add a text label for COM
        text(ax, 0, radius * 0.8, comHeight, 'COM', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'k', 'FontWeight', 'bold');
    end

    % --- Set Plot Properties ---
    axis(ax, 'equal'); % Maintain aspect ratio
    grid(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Height (m)');
    title(ax, sprintf('Container Visualization\nRadius: %.3f m, Container H: %.3f m, COM H: %.3f m', ...
                      radius, cylinderHeight, comHeight));
    % Set Z-axis limits for better visualization
    plotZLimit = max([cylinderHeight, comHeight, radius * 2]) * 1.2;
    if plotZLimit == 0
        plotZLimit = 0.1;
    end
    zlim(ax, [0, plotZLimit]);
    view(ax, 3); % 3D view
    camlight(ax); % Add lighting for better 3D appearance
    lighting(ax, 'gouraud'); % Smooth lighting
    hold(ax, 'off');
end