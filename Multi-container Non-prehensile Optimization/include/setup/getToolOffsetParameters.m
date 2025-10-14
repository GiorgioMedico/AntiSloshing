function [toolOffset, toolRotationOffset] = getToolOffsetParameters()
% getToolOffsetParameters creates an interactive GUI for defining tool offset parameters.
%
% The GUI allows the user to input:
%   - toolOffset: A 3x1 vector representing the offset from the end-effector frame to the tray frame.
%   - toolRotationOffset: A scalar representing the rotation offset about the Z-axis.
%
% Default values are provided.
%
% Returns:
%   toolOffset: The final 3x1 tool offset vector entered by the user (or default if cancelled).
%   toolRotationOffset: The final scalar tool rotation offset entered by the user (or default if cancelled).

    % --- Default Values ---
    defaultToolOffset = [0; 0; 0.056]; % meters
    defaultToolRotationOffset = 0; % radians (about Z-axis)

    % Initialize output variables with default values
    toolOffset = defaultToolOffset;
    toolRotationOffset = defaultToolRotationOffset;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Define Tool Offset Parameters', ...
                   'Position', [100 100 400 300], ... % Adjusted figure size
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src, fig));

    % Store data in the figure appdata
    appData = struct();
    appData.toolOffset = defaultToolOffset;
    appData.toolRotationOffset = defaultToolRotationOffset;
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---
    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Tool Offset Parameters', ...
                         'Position', [20 20 360 260], ...
                         'FontWeight', 'bold');

    % Tool Offset Input (3x1 vector)
    uilabel(inputPanel, 'Text', 'Tool Offset (3x1 vector, [x;y;z]):', ...
            'Position', [20 180 200 22]);
    toolOffsetField = uieditfield(inputPanel, 'text', ... % Use 'text' for vector input
                                  'Value', mat2str(defaultToolOffset), ... % Convert vector to string
                                  'Position', [20 150 320 22], ...
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'toolOffset'));
    uilabel(inputPanel, 'Text', 'e.g., [0;0;0.056] or 0;0;0.056', ...
            'Position', [20 130 320 20], ...
            'FontColor', [0.5 0.5 0.5], 'FontSize', 9);

    % Tool Rotation Offset Input (scalar)
    uilabel(inputPanel, 'Text', 'Tool Rotation Offset (rad, about Z):', ...
            'Position', [20 80 200 22]);
    toolRotationOffsetField = uieditfield(inputPanel, 'numeric', ...
                                          'Value', defaultToolRotationOffset, ...
                                          'Position', [20 50 320 22], ...
                                          'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'toolRotationOffset'));

    % OK Button
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [80 10 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    % Cancel Button
    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [190 10 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Wait for User Action with Error Handling ---
    try
        uiwait(fig);
    finally
        if isvalid(fig)
            finalAppData = getappdata(fig, 'appData');
            if finalAppData.confirmed
                toolOffset = finalAppData.toolOffset;
                toolRotationOffset = finalAppData.toolRotationOffset;
            else
                % If Cancel was pressed or window closed, return default values
                toolOffset = defaultToolOffset;
                toolRotationOffset = defaultToolRotationOffset;
            end
            delete(fig); % Delete the figure here, after all data retrieval
        else
            % If figure became invalid (e.g., deleted by other means),
            % outputs remain at their initial default values.
        end
    end

    if isvalid(fig)
        finalAppData = getappdata(fig, 'appData');
        if finalAppData.confirmed
            toolOffset = finalAppData.toolOffset;
            toolRotationOffset = finalAppData.toolRotationOffset;
        else
            % If Cancel was pressed or window closed, return default values
            toolOffset = defaultToolOffset;
            toolRotationOffset = defaultToolRotationOffset;
        end
        delete(fig); % Delete the figure here, after all data retrieval
    end
end

% --- Callback Function for Input Fields ---
function updateParameters(src, fig, paramName)
% updateParameters updates the stored parameter value.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    newValue = src.Value;

    % --- Input Validation and Parsing ---
    if strcmp(paramName, 'toolOffset')
        try
            parsedValue = eval(newValue); % Evaluate string like '[0;0;0.056]'
            if ~isnumeric(parsedValue) || ~isvector(parsedValue) || length(parsedValue) ~= 3
                error('Input must be a 3x1 numeric vector, e.g., [0;0;0.056] or 0;0;0.056');
            end
            newValue = parsedValue(:); % Ensure it's a column vector
        catch ME
            uialert(fig, ['Invalid input for Tool Offset: ' ME.message], 'Input Error', 'Icon', 'error');
            src.Value = mat2str(appData.toolOffset); % Revert to previous valid value
            return;
        end
    elseif strcmp(paramName, 'toolRotationOffset')
        if ~isnumeric(newValue) || ~isscalar(newValue)
            uialert(fig, 'Tool Rotation Offset must be a numeric scalar.', 'Input Error', 'Icon', 'error');
            src.Value = appData.toolRotationOffset; % Revert
            return;
        end
    end

    % Update the stored parameter
    appData.(paramName) = newValue;
    setappdata(fig, 'appData', appData);
    drawnow; % Ensure GUI updates are processed immediately
end

% --- Callback Function for OK Button ---
function okButtonPushed(fig)
% okButtonPushed sets the confirmed flag and resumes the figure.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    
    % Final validation before confirming
    % For toolOffset, re-evaluate to catch any last-minute invalid changes
    toolOffsetField = findobj(fig, 'Type', 'uieditfield', 'Tag', '', 'Value', mat2str(appData.toolOffset)); % Find by initial value
    if isempty(toolOffsetField) % Fallback if initial value search fails (e.g., if value changed)
        toolOffsetField = findobj(fig, 'Type', 'uieditfield', 'Position', [20 150 320 22]); % Find by position
    end

    if ~isempty(toolOffsetField)
        try
            parsedOffset = eval(toolOffsetField.Value);
            if ~isnumeric(parsedOffset) || ~isvector(parsedOffset) || length(parsedOffset) ~= 3
                error('Invalid final input for Tool Offset. Please ensure it is a 3x1 vector.');
            end
            appData.toolOffset = parsedOffset(:); % Update with final validated value
        catch ME
            uialert(fig, ['Final Tool Offset Validation Error: ' ME.message], 'Input Error', 'Icon', 'error');
            return; % Do not confirm if invalid
        end
    end

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
function closeFigure(src, fig)
% closeFigure handles closing the figure, ensuring uiresume is called.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Treat closing as cancellation
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end