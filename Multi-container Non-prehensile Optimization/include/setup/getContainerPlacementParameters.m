function [numContainers, placementMode, diagRadius, trayType, trayLength, trayWidth, trayThickness, frictionCoefficient] = getContainerPlacementParameters()
% getContainerPlacementParameters creates an interactive GUI for defining
% container placement and tray parameters.
%
% The GUI allows the user to input:
%   - Number of containers
%   - Placement mode (with options 'auto', 'inline', 'circle', 'diag', 'manual')
%   - Diagonal radius (conditionally visible for 'inline', 'diag', or 'circle' modes)
%   - Tray type (with options 'plastic', 'wood', 'custom')
%   - Custom tray dimensions and friction coefficient (conditionally visible for 'custom' tray type)
%
% Default values are provided.
%
% Returns:
%   numContainers: The number of containers.
%   placementMode: The chosen placement mode string.
%   diagRadius: The diagonal radius (NaN if not applicable, default if applicable and cancelled).
%   trayType: The chosen tray type string.
%   trayLength: The tray length (NaN if not applicable, default if applicable and cancelled).
%   trayWidth: The tray width (NaN if not applicable, default if applicable and cancelled).
%   trayThickness: The tray thickness (NaN if not applicable, default if applicable and cancelled).
%   frictionCoefficient: The friction coefficient (NaN if not applicable, default if applicable and cancelled).

    % --- Default Values ---
    defaultNumContainers = 4;
    defaultPlacementMode = 'diag'; % 'auto', 'inline', 'circle', 'diag', 'manual'
    defaultDiagRadius = 0.115; % meters

    defaultTrayType = 'plastic'; % 'plastic', 'wood', 'custom'
    defaultTrayLength = 0.5; % meters (for custom)
    defaultTrayWidth = 0.3; % meters (for custom)
    defaultTrayThickness = 0.005; % meters (for custom)
    defaultFrictionCoefficient = 0.3; % (for custom)

    % Initialize output variables (these will be overwritten by finalAppData or defaults)
    numContainers = defaultNumContainers;
    placementMode = defaultPlacementMode;
    diagRadius = defaultDiagRadius;
    trayType = defaultTrayType;
    trayLength = NaN;
    trayWidth = NaN;
    trayThickness = NaN;
    frictionCoefficient = NaN;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Container Placement Parameters', ...
                   'Position', [100 100 450 650], ... % Figure size adjusted to fit content well
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src));

    % Store data in the figure's appdata for easy access by callbacks
    appData = struct();
    appData.numContainers = defaultNumContainers;
    appData.placementMode = defaultPlacementMode;
    appData.diagRadius = defaultDiagRadius;
    appData.trayType = defaultTrayType;
    appData.trayLength = defaultTrayLength;
    appData.trayWidth = defaultTrayWidth;
    appData.trayThickness = defaultTrayThickness;
    appData.frictionCoefficient = defaultFrictionCoefficient;
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---

    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Parameters', ...
                         'Position', [20 20 410 610], ... % Panel height adjusted
                         'FontWeight', 'bold');

    % --- Container Parameters Section ---
    uilabel(inputPanel, 'Text', 'Number of Containers:', ...
            'Position', [20 550 180 22]);
    numContainersField = uieditfield(inputPanel, 'numeric', ...
                                     'Value', defaultNumContainers, ...
                                     'Position', [200 550 190 22], ...
                                     'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'numContainers'));

    uilabel(inputPanel, 'Text', 'Placement Mode:', ...
            'Position', [20 510 180 22]);
    placementModeDropdown = uidropdown(inputPanel, ...
                                       'Items', {'auto', 'inline', 'circle', 'diag', 'manual'}, ...
                                       'Value', defaultPlacementMode, ...
                                       'Position', [200 510 190 22], ...
                                       'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'placementMode'));

    % Subtext for Placement Mode explanation
    placementModeExplanationLabel = uilabel(inputPanel, ...
                                            'Text', '', ... % Will be updated dynamically
                                            'Position', [20 440 370 60], ... % Position adjusted for multiline text
                                            'VerticalAlignment', 'top', ...
                                            'FontColor', [0.5 0.5 0.5]); % Greyed out for explanation

    % Distance/Radius (conditionally visible)
    diagRadiusLabel = uilabel(inputPanel, 'Text', 'Distance/Radius (m):', ...
                               'Position', [20 400 180 22]); % Adjusted position
    diagRadiusField = uieditfield(inputPanel, 'numeric', ...
                                   'Value', defaultDiagRadius, ...
                                   'Position', [200 400 190 22], ... % Adjusted position
                                   'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'diagRadius'));

    % --- Tray Parameters Section ---
    uilabel(inputPanel, 'Text', 'Tray Type:', ...
            'Position', [20 320 180 22]); % Adjusted position
    trayTypeDropdown = uidropdown(inputPanel, ...
                                  'Items', {'plastic', 'wood', 'custom'}, ...
                                  'Value', defaultTrayType, ...
                                  'Position', [200 320 190 22], ... % Adjusted position
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'trayType'));

    % Custom Tray Parameters (conditionally visible)
    trayLengthLabel = uilabel(inputPanel, 'Text', 'Tray Length (m):', ...
                              'Position', [20 260 180 22]); % Adjusted position
    trayLengthField = uieditfield(inputPanel, 'numeric', ...
                                  'Value', defaultTrayLength, ...
                                  'Position', [200 260 190 22], ...
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'trayLength'));

    trayWidthLabel = uilabel(inputPanel, 'Text', 'Tray Width (m):', ...
                             'Position', [20 220 180 22]); % Adjusted position
    trayWidthField = uieditfield(inputPanel, 'numeric', ...
                                 'Value', defaultTrayWidth, ...
                                 'Position', [200 220 190 22], ...
                                 'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'trayWidth'));

    trayThicknessLabel = uilabel(inputPanel, 'Text', 'Tray Thickness (m):', ...
                                 'Position', [20 180 180 22]); % Adjusted position
    trayThicknessField = uieditfield(inputPanel, 'numeric', ...
                                     'Value', defaultTrayThickness, ...
                                     'Position', [200 180 190 22], ...
                                     'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'trayThickness'));

    frictionCoefficientLabel = uilabel(inputPanel, 'Text', 'Friction Coefficient:', ...
                                      'Position', [20 140 180 22]); % Adjusted position
    frictionCoefficientField = uieditfield(inputPanel, 'numeric', ...
                                          'Value', defaultFrictionCoefficient, ...
                                          'Position', [200 140 190 22], ...
                                          'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'frictionCoefficient'));

    % Store handles to conditionally visible components
    setappdata(fig, 'diagRadiusComponents', [diagRadiusLabel, diagRadiusField]);
    setappdata(fig, 'customTrayComponents', [trayLengthLabel, trayLengthField, ...
                                             trayWidthLabel, trayWidthField, ...
                                             trayThicknessLabel, trayThicknessField, ...
                                             frictionCoefficientLabel, frictionCoefficientField]);
    setappdata(fig, 'placementModeExplanationLabel', placementModeExplanationLabel); % Store handle for explanation label

    % --- OK and Cancel Buttons ---
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [80 60 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [240 60 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Initial Visibility and Explanation Update ---
    % Call the update function once to set initial visibility and explanation based on defaults
    updateConditionalVisibility(fig);
    updatePlacementModeExplanation(fig, defaultPlacementMode); % Set initial explanation

    % --- Wait for User Action ---
    uiwait(fig);

    % --- Retrieve Final Values and Close Figure ---
    finalAppData = getappdata(fig, 'appData');
    if finalAppData.confirmed
        numContainers = finalAppData.numContainers;
        placementMode = finalAppData.placementMode;
        trayType = finalAppData.trayType;

        % Assign conditional outputs based on final state
        if strcmp(finalAppData.placementMode, 'inline') || strcmp(finalAppData.placementMode, 'circle') || strcmp(finalAppData.placementMode, 'diag')
            diagRadius = finalAppData.diagRadius;
        else
            diagRadius = NaN; % Not applicable for 'auto' or 'manual'
        end

        % manualPositions is explicitly removed and will not be an output

        if strcmp(finalAppData.trayType, 'custom')
            trayLength = finalAppData.trayLength;
            trayWidth = finalAppData.trayWidth;
            trayThickness = finalAppData.trayThickness;
            frictionCoefficient = finalAppData.frictionCoefficient;
        else
            trayLength = NaN; % Not applicable
            trayWidth = NaN;
            trayThickness = NaN;
            frictionCoefficient = NaN;
        end
    else
        % If cancelled, return default values
        numContainers = defaultNumContainers;
        placementMode = defaultPlacementMode;
        trayType = defaultTrayType;

        % Assign conditional outputs based on initial default state
        if strcmp(defaultPlacementMode, 'inline') || strcmp(defaultPlacementMode, 'circle') || strcmp(defaultPlacementMode, 'diag')
            diagRadius = defaultDiagRadius;
        else
            diagRadius = NaN; % Not applicable for 'auto' or 'manual'
        end

        % manualPositions is explicitly removed and will not be an output

        if strcmp(defaultTrayType, 'custom')
            trayLength = defaultTrayLength;
            trayWidth = defaultTrayWidth;
            trayThickness = defaultTrayThickness;
            frictionCoefficient = defaultFrictionCoefficient;
        else
            trayLength = NaN; % Not applicable
            trayWidth = NaN;
            trayThickness = NaN;
            frictionCoefficient = NaN;
        end
    end

    if isvalid(fig)
        delete(fig);
    end

end


% --- Callback Function for Input Fields and Dropdowns ---
function updateParameters(src, fig, paramName)
% updateParameters updates the stored parameter value and triggers visibility/explanation update.
    appData = getappdata(fig, 'appData');
    newValue = src.Value;

    % Basic validation for numeric fields
    if isnumeric(newValue)
        if (strcmp(paramName, 'numContainers') && (newValue <= 0 || mod(newValue,1) ~= 0)) || ... % Must be positive integer
           (strcmp(paramName, 'diagRadius') && newValue <= 0) || ...
           (strcmp(paramName, 'trayLength') && newValue <= 0) || ...
           (strcmp(paramName, 'trayWidth') && newValue <= 0) || ...
           (strcmp(paramName, 'trayThickness') && newValue <= 0) || ...
           (strcmp(paramName, 'frictionCoefficient') && (newValue < 0 || newValue > 1)) % Friction usually 0-1
            uialert(fig, ['Please enter a valid number for ' strrep(paramName, 'num', 'number of ') '.'], 'Input Error');
            src.Value = appData.(paramName); % Revert to previous valid value
            return;
        end
    end
    
    % Update the stored parameter
    appData.(paramName) = newValue;
    setappdata(fig, 'appData', appData);

    % Update visibility if a mode/type parameter changed
    if strcmp(paramName, 'placementMode') || strcmp(paramName, 'trayType')
        updateConditionalVisibility(fig);
    end

    % Update explanation if placementMode changed
    if strcmp(paramName, 'placementMode')
        updatePlacementModeExplanation(fig, newValue);
    end
    drawnow; % Ensure GUI updates are processed immediately
end

% --- Function to Update Conditional Visibility ---
function updateConditionalVisibility(fig)
% updateConditionalVisibility toggles the visibility of input fields based on
% placementMode and trayType selections.
    appData = getappdata(fig, 'appData');

    % Get handles to conditionally visible components
    diagRadiusComponents = getappdata(fig, 'diagRadiusComponents');
    customTrayComponents = getappdata(fig, 'customTrayComponents');

    % --- Placement Mode Logic ---
    currentMode = appData.placementMode;
    if strcmp(currentMode, 'inline') || strcmp(currentMode, 'circle') || strcmp(currentMode, 'diag')
        set(diagRadiusComponents, 'Visible', 'on');
    else
        set(diagRadiusComponents, 'Visible', 'off');
    end

    % --- Tray Type Logic ---
    if strcmp(appData.trayType, 'custom')
        set(customTrayComponents, 'Visible', 'on');
    else
        set(customTrayComponents, 'Visible', 'off');
    end
end

% --- Function to Update Placement Mode Explanation ---
function updatePlacementModeExplanation(fig, currentMode)
% updatePlacementModeExplanation updates the text of the explanation label
% based on the selected placement mode.
    explanationLabel = getappdata(fig, 'placementModeExplanationLabel');
    
    switch currentMode
        case 'auto'
            explanation = 'Containers will be automatically placed on an evenly spaced square mesh grid on the tray.';
        case 'inline'
            explanation = 'Containers will be placed in a single line. The "Distance" will define the spacing between the center of each container.';
        case 'circle'
            explanation = 'Containers will be arranged in a circular pattern. The "Radius" will specify the radius of this circle.';
        case 'diag'
            explanation = 'Containers will be placed along the primary diagonal paths of the tray surface. The "Distance" will define the distance of each container to the tray center.';
        case 'manual'
            explanation = 'Container positions will be need to be defined externally after this window closes. No further input is required here.';
        otherwise
            explanation = ''; % Should not happen
    end
    set(explanationLabel, 'Text', explanation);
end


% --- Callback Function for OK Button ---
function okButtonPushed(fig)
% okButtonPushed sets the confirmed flag and resumes the figure.
    % disp('okButtonPushed: OK button pushed. Calling uiresume.'); % Trace
    appData = getappdata(fig, 'appData');
    appData.confirmed = true;
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Callback Function for Cancel Button ---
function cancelButtonPushed(fig)
% cancelButtonPushed ensures the confirmed flag is false and resumes the figure.
    disp('cancelButtonPushed: Cancel button pushed. Calling uiresume.'); % Trace
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Explicitly set to false
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Custom Close Request Function ---
function closeFigure(fig)
% closeFigure handles closing the figure, ensuring uiresume is called.
    disp('closeFigure: CloseRequestFcn triggered. Calling uiresume.'); % Trace
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Treat closing as cancellation
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end