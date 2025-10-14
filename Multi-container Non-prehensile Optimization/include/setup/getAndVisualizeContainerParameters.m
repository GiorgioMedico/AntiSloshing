function [cylinderRadius, liquidHeight, etaLim, cylinderHeight, mass, density, viscosity, comHeight] = getAndVisualizeContainerParameters()
% getAndVisualizeContainerParameters creates an interactive GUI for container
% parameters and visualizes them dynamically.
%
% The GUI allows the user to input:
%   - Container radius
%   - Liquid height
%   - eta_lim (the limit sloshing height)
%   - Container height (the actual height of the container)
%   - Mass of the cylinder
%   - Density of the liquid
%   - Viscosity of the liquid
%   - Center of Mass Height (Z-coordinate of the COM relative to container base)
%
% Default values are provided. The 3D plot updates in real-time as the
% user changes the input parameters.
%
% Returns:
%   cylinderRadius: The final container radius entered by the user.
%   liquidHeight: The final liquid height entered by the user.
%   etaLim: The final eta_lim value entered by the user.
%   cylinderHeight: The final container height entered by the user.
%   mass: The final mass entered by the user.
%   density: The final liquid density entered by the user.
%   viscosity: The final liquid viscosity entered by the user.
%   comHeight: The final center of mass height entered by the user.

    % --- Default Values ---
    defaultRadius = 0.035; % meters
    defaultLiquidHeight = 0.049; % meters
    defaultEtaLim = 0.015; % meters
    defaultCylinderHeight = 0.0991; % meters
    defaultMass = 0.1970; % kg
    defaultDensity = 998.2071; % kg/m^3 (density of water at ~20°C)
    defaultViscosity = 0.001; % Pa·s (dynamic viscosity of water at ~20°C)
    defaultComHeight = 0.0281; % meters (Center of Mass height relative to container base)

    % Initialize output variables with default values
    cylinderRadius = defaultRadius;
    liquidHeight = defaultLiquidHeight;
    etaLim = defaultEtaLim;
    cylinderHeight = defaultCylinderHeight;
    mass = defaultMass;
    density = defaultDensity;
    viscosity = defaultViscosity;
    comHeight = defaultComHeight;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Container Parameters & Visualization', ...
                   'Position', [100 100 800 780], ... % Adjusted figure height to accommodate new parameter
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src, fig)); % Pass fig handle to closeFigure

    % Store data in the figure appdata
    appData = struct();
    appData.radius = defaultRadius;
    appData.liquidHeight = defaultLiquidHeight;
    appData.etaLim = defaultEtaLim;
    appData.cylinderHeight = defaultCylinderHeight;
    appData.mass = defaultMass;
    appData.density = defaultDensity;
    appData.viscosity = defaultViscosity;
    appData.comHeight = defaultComHeight; % Store new parameter
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---
    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Parameters', ...
                         'Position', [20 20 250 740], ... 
                         'FontWeight', 'bold');

    % Container Radius Input
    uilabel(inputPanel, 'Text', 'Container radius (m):', ...
            'Position', [20 680 200 22]);
    radiusField = uieditfield(inputPanel, 'numeric', ...
                               'Value', defaultRadius, ...
                               'Position', [20 650 200 22], ...
                               'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'radius'));

    % Liquid Height Input
    uilabel(inputPanel, 'Text', 'Liquid height (m):', ...
            'Position', [20 600 200 22]);
    liquidHeightField = uieditfield(inputPanel, 'numeric', ...
                                     'Value', defaultLiquidHeight, ...
                                     'Position', [20 570 200 22], ...
                                     'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'liquidHeight'));

    % Eta_lim Input
    uilabel(inputPanel, 'Text', '<html>&eta;<sub>lim</sub> (m):</html>', ...
            'Position', [20 520 200 22], ...
            'Interpreter', 'html');
    etaLimField = uieditfield(inputPanel, 'numeric', ...
                               'Value', defaultEtaLim, ...
                               'Position', [20 490 200 22], ...
                               'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'etaLim'));

    % Container Height Input
    uilabel(inputPanel, 'Text', 'Container height (m):', ...
            'Position', [20 440 200 22]);
    cylinderHeightField = uieditfield(inputPanel, 'numeric', ...
                                       'Value', defaultCylinderHeight, ...
                                       'Position', [20 410 200 22], ...
                                       'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'cylinderHeight'));

    % Mass Input
    uilabel(inputPanel, 'Text', 'Mass (kg):', ...
            'Position', [20 360 200 22]);
    massField = uieditfield(inputPanel, 'numeric', ...
                             'Value', defaultMass, ...
                             'Position', [20 330 200 22], ...
                             'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'mass'));

    % Center of Mass Height Input 
    uilabel(inputPanel, 'Text', 'Center of Mass Height (m):', ...
            'Position', [20 280 200 22]);
    comHeightField = uieditfield(inputPanel, 'numeric', ...
                                  'Value', defaultComHeight, ...
                                  'Position', [20 250 200 22], ...
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'comHeight'));

    % Density Input
    uilabel(inputPanel, 'Text', 'Density (kg/m^3):', ...
            'Position', [20 200 200 22]);
    densityField = uieditfield(inputPanel, 'numeric', ...
                                'Value', defaultDensity, ...
                                'Position', [20 170 200 22], ...
                                'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'density'));

    % Viscosity Input
    uilabel(inputPanel, 'Text', 'Viscosity (Pa·s):', ...
            'Position', [20 120 200 22]);
    viscosityField = uieditfield(inputPanel, 'numeric', ...
                                  'Value', defaultViscosity, ...
                                  'Position', [20 90 200 22], ...
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'viscosity'));

    % OK Button
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [20 20 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    % Cancel Button
    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [130 20 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Create UIAxes for Plotting ---
    ax = uiaxes(fig, 'Position', [300 50 480 700]); % Adjusted axes position/height
    % Store axes handle in appdata for plotting function
    setappdata(fig, 'axesHandle', ax);

    % --- Initial Plot ---
    % Call the plotting function once to display the default state
    updateCylinderPlot(ax, appData.radius, appData.liquidHeight, appData.etaLim, appData.cylinderHeight, appData.comHeight);

    % --- Wait for User Action with Error Handling ---
    try
        uiwait(fig);
    finally
        if isvalid(fig)
            finalAppData = getappdata(fig, 'appData');
            if finalAppData.confirmed
                cylinderRadius = finalAppData.radius;
                liquidHeight = finalAppData.liquidHeight;
                etaLim = finalAppData.etaLim;
                cylinderHeight = finalAppData.cylinderHeight;
                mass = finalAppData.mass;
                density = finalAppData.density;
                viscosity = finalAppData.viscosity;
                comHeight = finalAppData.comHeight;
            else
                cylinderRadius = defaultRadius;
                liquidHeight = defaultLiquidHeight;
                etaLim = defaultEtaLim;
                cylinderHeight = defaultCylinderHeight;
                mass = defaultMass;
                density = defaultDensity;
                viscosity = defaultViscosity;
                comHeight = defaultComHeight;
            end
        end
    end
    
    if isvalid(fig) % Check if figure is still valid before attempting to get appdata
        finalAppData = getappdata(fig, 'appData'); % Retrieve data
        if finalAppData.confirmed
            cylinderRadius = finalAppData.radius;
            liquidHeight = finalAppData.liquidHeight;
            etaLim = finalAppData.etaLim;
            cylinderHeight = finalAppData.cylinderHeight;
            mass = finalAppData.mass;
            density = finalAppData.density;
            viscosity = finalAppData.viscosity;
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
% paramName: The name of the parameter ('radius', 'liquidHeight', 'etaLim', 'cylinderHeight', 'mass', 'density', 'viscosity', 'comHeight').
    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    ax = getappdata(fig, 'axesHandle');
    % Get the new value from the edit field
    newValue = src.Value;
    % Basic validation: ensure positive values for radius, non-negative for heights/mass, positive for density/viscosity
    if (strcmp(paramName, 'radius') && newValue <= 0) || ...
       (strcmp(paramName, 'liquidHeight') && newValue < 0) || ...
       (strcmp(paramName, 'etaLim') && newValue < 0) || ...
       (strcmp(paramName, 'cylinderHeight') && newValue <= 0) || ... % Container height must be positive
       (strcmp(paramName, 'mass') && newValue < 0) || ...
       (strcmp(paramName, 'density') && newValue <= 0) || ... % Density must be positive
       (strcmp(paramName, 'viscosity') && newValue <= 0) || ... % Viscosity must be positive
       (strcmp(paramName, 'comHeight') && newValue < 0) % COM Height can be 0 or positive
        uialert(fig, 'Please enter valid positive numbers for radius, container height, density, viscosity. Non-negative for liquid height, eta_lim, mass, and center of mass height.', 'Input Error');
        % Revert to previous valid value if invalid input
        src.Value = appData.(paramName);
        return;
    end
    % Update the stored parameter
    appData.(paramName) = newValue;
    setappdata(fig, 'appData', appData);
    % Redraw the plot with updated parameters
    % Only update plot if a visual parameter changed
    if any(strcmp(paramName, {'radius', 'liquidHeight', 'etaLim', 'cylinderHeight', 'comHeight'}))
        updateCylinderPlot(ax, appData.radius, appData.liquidHeight, appData.etaLim, appData.cylinderHeight, appData.comHeight);
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

% --- Function to Plot the Container (updated for uiaxes and eta_lim position) ---
function updateCylinderPlot(ax, radius, liquidHeight, etaLim, cylinderHeight, comHeight)
% updateCylinderPlot generates a 3D plot of a cylinder with liquid and a limit line.
% Inputs:
%   ax: The uiaxes object to plot on.
%   radius: The radius of the cylinder.
%   liquidHeight: The height of the liquid inside the cylinder.
%   etaLim: The height of the limit line (relative to liquid surface).
%   cylinderHeight: The actual height of the cylinder object.
%   comHeight: The Z-coordinate of the center of mass.
    cla(ax); % Clear the specified axes
    % --- Draw the Container ---
    % Generate cylinder data
    [X, Y, Z] = cylinder(radius, 50); % 50 points for smooth circle
    % Determine total plot height for cylinder and axis limits
    % The cylinder should be tall enough to contain liquid, eta_lim line,
    % and its own defined height.
    totalEtaLimHeight = liquidHeight + etaLim;
    % The cylinder's Z-coordinates are scaled by the provided cylinderHeight
    Z_cylinder_scaled = Z * cylinderHeight;
    % Set the overall plot Z-limit to comfortably fit the cylinder and any lines
    plotZLimit = max([cylinderHeight, liquidHeight, totalEtaLimHeight, radius * 2, comHeight]) * 1.2;
    if plotZLimit == 0 % Handle case where all inputs are 0
        plotZLimit = 0.1; % Give a minimum height for visibility
    end
    % Plot the cylinder surface using the scaled Z coordinates
    hCyl = surf(ax, X, Y, Z_cylinder_scaled);
    set(hCyl, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', [0.5 0.5 0.5]);
    hold(ax, 'on'); % Hold on for the specific axes
    % --- Draw the Liquid ---
    if liquidHeight > 0
        % Generate liquid surface data (a disk at height liquidHeight)
        theta = linspace(0, 2*pi, 50);
        x_liquid = radius * cos(theta);
        y_liquid = radius * sin(theta);
        z_liquid = liquidHeight * ones(size(theta));
        % Create a filled patch for the liquid surface
        hLiquid = fill3(ax, x_liquid, y_liquid, z_liquid, [0.3 0.6 0.9]); % Blue color
        set(hLiquid, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
        % Also draw the side of the liquid
        [X_l, Y_l, Z_l] = cylinder(radius, 50);
        Z_l = Z_l * liquidHeight; % Scale Z to liquid height
        hLiquidSide = surf(ax, X_l, Y_l, Z_l);
        set(hLiquidSide, 'FaceColor', [0.3 0.6 0.9], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    end
    % --- Draw the eta_lim line ---
    if etaLim >= 0
        % The red line is at liquid_height + eta_lim
        lineHeight = liquidHeight + etaLim;
        theta_line = linspace(0, 2*pi, 100);
        x_line = radius * cos(theta_line);
        y_line = radius * sin(theta_line);
        z_line = lineHeight * ones(size(theta_line));
        hEtaLim = plot3(ax, x_line, y_line, z_line, 'r--', 'LineWidth', 2); % Red dashed line
    end

    % --- Draw the Center of Mass (COM) height ---
    if comHeight >= 0 && comHeight <= cylinderHeight
        theta_com_line = linspace(0, 2*pi, 100);
        x_com_line = radius * cos(theta_com_line);
        y_com_line = radius * sin(theta_com_line);
        z_com_line = comHeight * ones(size(theta_com_line));
        plot3(ax, x_com_line, y_com_line, z_com_line, 'k:', 'LineWidth', 1.5); % Black dotted line
        % Add a text label for COM
        text(ax, 0, radius * 0.8, comHeight, 'COM', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'k');
    end

    % --- Set Plot Properties ---
    axis(ax, 'equal'); % Maintain aspect ratio
    grid(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Height (m)');
    title(ax, sprintf('Container Visualization\nRadius: %.3f m, Liquid: %.3f m, $\\eta_{lim}$: %.3f m, Container H: %.3f m, COM H: %.3f m', ...
                      radius, liquidHeight, etaLim, cylinderHeight, comHeight), 'Interpreter', 'latex');
    % Set Z-axis limits for better visualization
    zlim(ax, [0, plotZLimit]);
    view(ax, 3); % 3D view
    camlight(ax); % Add lighting for better 3D appearance
    lighting(ax, 'gouraud'); % Smooth lighting
    hold(ax, 'off');
end
