function [path_type, mode, p0, thz_0, thz_end, cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getGeometricPathParameters()
% getGeometricPathParameters creates an interactive GUI for defining
% geometric path parameters and visualizes the path.
%
% The GUI allows the user to input:
%   - path_type: Type of trajectory (e.g., 'arc', 'pp', 'manual').
%   - mode: 'absolute' or 'relative' positioning.
%   - p0: Initial position vector (3x1), visible only if mode is 'relative'
%         and path_type is NOT 'manual'.
%   - thz_0: Initial Angular position (can accept 'pi' or numeric values),
%   - thz_end: Final Angular position (can accept 'pi' or numeric values),
%   - For 'manual' path_type:
%       - Number of control points (N).
%       - N 3D vectors [x;y;z] for the control points.
%
% The GUI dynamically plots the 3D path based on selected parameters.
%
% Returns:
%   path_type: The selected path type string.
%   mode: The selected mode string.
%   p0: The initial position vector (NaN(3,1) if mode is 'absolute',
%       or if path_type is 'manual' in absolute mode, or if cancelled).
%   thz_0: The final angular position (default if cancelled).
%   thz_end: The final angular position (default if cancelled).
%   cpx_final, cpy_final, cpz_final: The final calculated control points.
%                                    Empty if 'manual' mode inputs are invalid or cancelled.
%   n_ctrl_pts_final: The final number of control points.

    % --- Default Values ---
    defaultPathType = 'arc'; % Default path type
    defaultMode = 'absolute'; % Default mode
    defaultP0 = [0; 0; 0]; % Default initial position for 'relative' mode
    defaultThz0 = 0; % Default start angle
    defaultThzEnd = pi; % Default end angle
    defaultNumManualCtrlPts = 5; % Default number of control points for 'manual' mode

    % Initialize output variables (these will be overwritten in the finally block)
    path_type = defaultPathType;
    mode = defaultMode;
    p0 = defaultP0; % Will be NaN if not applicable
    thz_0 = defaultThz0; % Ensure thz_0 is initialized
    thz_end = defaultThzEnd; % Ensure thz_end is initialized
    cpx_final = [];
    cpy_final = [];
    cpz_final = [];
    n_ctrl_pts_final = 0;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Geometric Path Selection', ...
                   'Position', [100 100 1000 700], ... % Adjusted figure size to accommodate plot
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src, fig));

    % Store data in the figure's appdata for easy access by callbacks
    appData = struct();
    appData.path_type = defaultPathType;
    appData.mode = defaultMode;
    appData.p0 = defaultP0;
    appData.thz_0 = defaultThz0;
    appData.thz_end = defaultThzEnd;
    appData.num_manual_ctrl_pts = defaultNumManualCtrlPts;
    appData.manual_ctrl_pts = NaN(3, defaultNumManualCtrlPts); % 3xN matrix for manual control points
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---
    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Path Parameters', ...
                         'Position', [20 20 410 660], ...
                         'FontWeight', 'bold', ...
                         'Scrollable', 'on'); % Make input panel scrollable

    % Path Type Dropdown
    uilabel(inputPanel, 'Text', 'Path Type:', ...
            'Position', [20 600 150 22]);
    pathTypeDropdown = uidropdown(inputPanel, ...
                                  'Items', {'arc', 'pp', 'sl', 'circle','b', 'manual'}, ...
                                  'Value', defaultPathType, ...
                                  'Position', [180 600 210 22], ...
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'path_type'));

    % --- Standard Path Parameters (conditionally visible for non-'manual' types) ---
    % Mode Dropdown
    modeLabel = uilabel(inputPanel, 'Text', 'Mode:', ...
                        'Position', [20 540 150 22]);
    modeDropdown = uidropdown(inputPanel, ...
                              'Items', {'absolute', 'relative'}, ...
                              'Value', defaultMode, ...
                              'Position', [180 540 210 22], ...
                              'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'mode'));

    % p0 (Initial Position) Input (conditionally visible)
    p0Label = uilabel(inputPanel, 'Text', 'Initial Position (p0, [x;y;z]):', ...
                      'Position', [20 480 150 22]);
    p0Field = uieditfield(inputPanel, 'text', ...
                          'Value', mat2str(defaultP0), ... % Convert vector to string
                          'Position', [180 480 210 22], ...
                          'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'p0'));
    p0ExampleLabel = uilabel(inputPanel, 'Text', 'e.g., [0;0;0]', ...
            'Position', [20 460 370 20], ...
            'FontColor', [0.5 0.5 0.5]);

    % thz_0 (Initial Angle) Input 
    thz0Label = uilabel(inputPanel, 'Text', 'Start Angle (θ_z(0)):', ...
            'Position', [20 400 120 22]); % Keep this label visible
    thz0Field = uieditfield(inputPanel, 'text', ...
                              'Value', num2str(defaultThz0), ...
                              'Position', [130 400 50 22], ...
                              'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'thz_0'));
    thz0ExampleLabel = uilabel(inputPanel, 'Text', 'e.g., pi or 3.14', ... % Hint for user
            'Position', [20 380 370 20], ...
            'FontColor', [0.5 0.5 0.5]);

    % thz_end (Final Angle) Input
    thzEndLabel = uilabel(inputPanel, 'Text', 'End Angle (θ_z(T)):', ...
            'Position', [210 400 120 22]); % Keep this label visible
    thzEndField = uieditfield(inputPanel, 'text', ...
                              'Value', num2str(defaultThzEnd), ...
                              'Position', [320 400 50 22], ...
                              'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'thz_end'));
    thzEndExampleLabel = uilabel(inputPanel, 'Text', 'e.g., pi or 3.14', ... % Hint for user
            'Position', [20 380 370 20], ...
            'FontColor', [0.5 0.5 0.5]);

    % Store handles to standard path components for conditional visibility
    setappdata(fig, 'standardPathComponents', [modeLabel, modeDropdown, ...
                                               p0Label, p0Field, p0ExampleLabel]);
    setappdata(fig, 'p0Components', [p0Label, p0Field, p0ExampleLabel]); % Keep this separate for mode-based visibility

    % --- Manual Path Parameters (conditionally visible for 'manual' type) ---
    % Y positions adjusted to accommodate always-visible thz_end
    numCtrlPtsLabel = uilabel(inputPanel, 'Text', 'Number of Control Points (N):', ...
                              'Position', [20 340 150 22], 'Visible', 'off'); 
    numCtrlPtsField = uieditfield(inputPanel, 'numeric', ...
                                  'Value', defaultNumManualCtrlPts, ...
                                  'Position', [180 340 210 22], 'Visible', 'off', ... 
                                  'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'num_manual_ctrl_pts'));
    
    manualCtrlPtsHeaderLabel = uilabel(inputPanel, 'Text', 'Manual 3D Control Points ([x;y;z]):', ...
                                         'Position', [20 300 300 22], 'Visible', 'off', 'FontWeight', 'bold'); 
    manualCtrlPtsHelpLabel = uilabel(inputPanel, 'Text', 'e.g., [1;2;3]', ...
                                       'Position', [20 280 370 20], 'Visible', 'off', 'FontColor', [0.5 0.5 0.5], 'FontSize', 9); 
    
    % Store handles to manual path components
    setappdata(fig, 'manualPathFixedComponents', [numCtrlPtsLabel, numCtrlPtsField, ...
                                                  manualCtrlPtsHeaderLabel, manualCtrlPtsHelpLabel]);
    setappdata(fig, 'manualInputPanelParent', inputPanel); % Parent for dynamic creation
    setappdata(fig, 'manualCtrlPtsFields', gobjects(0)); % Placeholder for dynamic fields
    setappdata(fig, 'manualCtrlPtsLabels', gobjects(0)); % Placeholder for dynamic labels


    % --- OK and Cancel Buttons ---
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [80 10 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));
    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [240 10 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Create UIAxes for Plotting ---
    ax = uiaxes(fig, 'Position', [450 50 520 600]); % Axes for 3D path plot
    setappdata(fig, 'pathAxesHandle', ax);

    % --- Initial Visibility and Plot Update ---
    updateConditionalVisibility(fig);
    updatePathPlot(getappdata(fig, 'pathAxesHandle'), ...
                   appData.path_type, appData.mode, appData.p0, appData.thz_0, appData.thz_end, ...
                   appData.num_manual_ctrl_pts, appData.manual_ctrl_pts);

    % --- Wait for User Action with Error Handling ---
    try
        uiwait(fig);
    finally
        if isvalid(fig)
            finalAppData = getappdata(fig, 'appData'); % Retrieve data BEFORE deleting
            path_type = finalAppData.path_type; % Always retrieve path_type
            mode = finalAppData.mode; % Always retrieve mode
            thz_0 = finalAppData.thz_0; % Always retrieve final thz_0
            thz_end = finalAppData.thz_end; % Always retrieve final thz_end

            if finalAppData.confirmed
                if strcmp(finalAppData.path_type, 'manual')
                    if strcmp(finalAppData.mode, 'relative')
                        p0 = finalAppData.p0; % p0 is the offset for relative manual
                    else
                        p0 = NaN(3,1); % Not applicable for absolute manual
                    end
                    % Get the final control points from appData for manual mode
                    cpx_final = finalAppData.manual_ctrl_pts(1,:);
                    cpy_final = finalAppData.manual_ctrl_pts(2,:);
                    cpz_final = finalAppData.manual_ctrl_pts(3,:);
                    n_ctrl_pts_final = finalAppData.num_manual_ctrl_pts;
                else % Not 'manual' path type
                    if strcmp(finalAppData.mode, 'relative')
                        p0 = finalAppData.p0;
                    else
                        p0 = NaN(3,1); % Not applicable
                    end
                    % Recalculate final path for return values using getPath
                    [cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getPath(path_type, mode, p0);
                end
            else % Cancelled or closed, return defaults or NaNs where applicable
                path_type = defaultPathType;
                mode = defaultMode;
                p0 = NaN(3,1); % Always NaN if cancelled
                thz_0 = defaultThz0; % Always default if cancelled
                thz_end = defaultThzEnd; % Always default if cancelled
                cpx_final = []; % Empty if cancelled
                cpy_final = [];
                cpz_final = [];
                n_ctrl_pts_final = 0; % 0 if cancelled
            end
            delete(fig); % Delete the figure here, after all data retrieval
        else
            % Figure already invalid, output variables remain at initial NaN/default values.
        end
    end


    finalAppData = getappdata(fig, 'appData');
    if finalAppData.confirmed
        % disp('getGeometricPathParameters: OK button pressed. Assigning current values.'); % Trace
        path_type = finalAppData.path_type;
        mode = finalAppData.mode;
        thz_0 = finalAppData.thz_0;
        thz_end = finalAppData.thz_end;

        if strcmp(finalAppData.mode, 'relative')
            p0 = finalAppData.p0;
        else
            p0 = NaN(3,1); % Not applicable
        end

        if strcmp(finalAppData.path_type, 'manual')
            P_final = finalAppData.manual_ctrl_pts(:,:);
            if strcmp(finalAppData.mode, 'relative')
                [cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getPath(path_type, mode, p0, finalAppData.num_manual_ctrl_pts, P_final);
            else
                [cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getPath(path_type, mode, finalAppData.num_manual_ctrl_pts, P_final);
            end
        else
            % Recalculate final path for return values
            [cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getPath(path_type, mode, p0);
        end

    else
        % disp('getGeometricPathParameters: Cancel button pressed or window closed. Assigning default values.'); % Trace
        path_type = defaultPathType;
        mode = defaultMode;
        thz_0 = defaultThz0;
        thz_end = defaultThzEnd;

        if strcmp(defaultMode, 'relative')
            p0 = defaultP0;
        else
            p0 = NaN(3,1); % Not applicable
        end
        % Recalculate default path for return values
        [cpx_final, cpy_final, cpz_final, n_ctrl_pts_final] = getPath(path_type, mode, p0);
    end

    if isvalid(fig)
        delete(fig);
    end


end

% --- Callback Function for Input Fields and Dropdowns ---
function updateParameters(src, fig, paramName)
% updateParameters updates the stored parameter value and triggers visibility/plot update.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    newValue = src.Value;

    % --- Input Validation and Parsing ---
    if strcmp(paramName, 'p0')
        try
            parsedValue = eval(newValue);
            if ~isnumeric(parsedValue) || ~isvector(parsedValue) || length(parsedValue) ~= 3
                error('Input must be a 3x1 numeric vector, e.g., [0;0;0]');
            end
            newValue = parsedValue(:);
        catch ME
            uialert(fig, ['Invalid input for Initial Position: ' ME.message], 'Input Error');
            src.Value = mat2str(appData.p0);
            return;
        end
    elseif strcmp(paramName, 'thz_0')
        try
            parsedValue = eval(newValue);
            if ~isnumeric(parsedValue) || ~isscalar(parsedValue) || parsedValue < 0
                error('Input must be a non-negative numeric value or expression (e.g., pi, 3.14).');
            end
            newValue = parsedValue;
        catch ME
            uialert(fig, ['Invalid input for Start Angle: ' ME.message], 'Input Error');
            src.Value = num2str(appData.thz_0);
            return;
        end
    elseif strcmp(paramName, 'thz_end')
        try
            parsedValue = eval(newValue);
            if ~isnumeric(parsedValue) || ~isscalar(parsedValue) || parsedValue < 0
                error('Input must be a non-negative numeric value or expression (e.g., pi, 3.14).');
            end
            newValue = parsedValue;
        catch ME
            uialert(fig, ['Invalid input for End Angule: ' ME.message], 'Input Error');
            src.Value = num2str(appData.thz_end);
            return;
        end
    elseif strcmp(paramName, 'num_manual_ctrl_pts')
        if ~isnumeric(newValue) || newValue <= 0 || mod(newValue,1) ~= 0
            uialert(fig, 'Number of Control Points must be a positive integer.', 'Input Error');
            src.Value = appData.num_manual_ctrl_pts; % Revert
            return;
        end
        % If num_manual_ctrl_pts changes, resize the manual_ctrl_pts matrix
        appData.manual_ctrl_pts = NaN(3, newValue); % Reset points to NaN when N changes
    end

    appData.(paramName) = newValue;
    setappdata(fig, 'appData', appData);

    % --- Update Visibility and Plot ---
    updateConditionalVisibility(fig); % Always call for dynamic UI changes
    
    % Get updated appData after potential changes in num_manual_ctrl_pts by updateConditionalVisibility
    appData = getappdata(fig, 'appData'); 

    updatePathPlot(getappdata(fig, 'pathAxesHandle'), ...
                   appData.path_type, appData.mode, appData.p0, appData.thz_0, appData.thz_end, ...
                   appData.num_manual_ctrl_pts, appData.manual_ctrl_pts);
    drawnow;
end

% --- Function to Update Conditional Visibility ---
function updateConditionalVisibility(fig)
% updateConditionalVisibility toggles the visibility of input fields based on path_type and mode.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');

    % Get handles to components
    standardPathComponents = getappdata(fig, 'standardPathComponents');
    p0Components = getappdata(fig, 'p0Components'); % Specific for p0 visibility based on mode
    manualPathFixedComponents = getappdata(fig, 'manualPathFixedComponents');
    
    manualInputPanelParent = getappdata(fig, 'manualInputPanelParent');

    % Destroy existing dynamic manual input fields if any
    currentManualFields = getappdata(fig, 'manualCtrlPtsFields');
    currentManualLabels = getappdata(fig, 'manualCtrlPtsLabels');
    if ~isempty(currentManualFields) && isvalid(currentManualFields(1))
        delete(currentManualFields);
        setappdata(fig, 'manualCtrlPtsFields', gobjects(0));
    end
    if ~isempty(currentManualLabels) && isvalid(currentManualLabels(1))
        delete(currentManualLabels);
        setappdata(fig, 'manualCtrlPtsLabels', gobjects(0));
    end

    % --- Path Type Logic ---
    if strcmp(appData.path_type, 'manual')
        set(standardPathComponents, 'Visible', 'off'); % Hide standard path inputs
        set(manualPathFixedComponents, 'Visible', 'on'); % Show manual path inputs

        % Dynamically create manual 3D control point inputs
        numCtrlPts = appData.num_manual_ctrl_pts;
        newManualFields = gobjects(numCtrlPts, 1);
        newManualLabels = gobjects(numCtrlPts, 1);
        
        % Position calculation relative to inputPanel (parent)
        currentY = 260; % Adjusted Y: Start below manualCtrlPtsHelpLabel

        for i = 1:numCtrlPts
            newManualLabels(i) = uilabel(manualInputPanelParent, 'Text', sprintf('Point %d (x;y;z):', i), ...
                                         'Position', [20 currentY - (i-1)*35 120 22], 'Visible', 'on');
            
            initialVecValue = appData.manual_ctrl_pts(:, i);
            if any(isnan(initialVecValue))
                initialValueStr = ''; % Display empty if NaN
            else
                initialValueStr = mat2str(initialVecValue); % Display as [x;y;z]
            end

            % Pass 'fig' and 'i' to the callback for live updates
            newManualFields(i) = uieditfield(manualInputPanelParent, 'text', ... % 'text' style
                                              'Value', initialValueStr, ...
                                              'Position', [150 currentY - (i-1)*35 200 22], ...
                                              'Tag', sprintf('manualCtrlPtField_%d', i), ...
                                              'ValueChangedFcn', @(src,event) updateManualCtrlPtInput(src, fig, i));
        end
        setappdata(fig, 'manualCtrlPtsFields', newManualFields);
        setappdata(fig, 'manualCtrlPtsLabels', newManualLabels);

    else % Not 'manual' path type
        set(standardPathComponents, 'Visible', 'on'); % Show standard path inputs
        set(manualPathFixedComponents, 'Visible', 'off'); % Hide manual path inputs

        % --- Mode Logic for standard paths ---
        if strcmp(appData.mode, 'relative')
            set(p0Components, 'Visible', 'on');
        else
            set(p0Components, 'Visible', 'off');
        end
    end
    % Ensure dynamic resizing of input panel if content overflows for scrolling
end

% --- Callback for Individual Manual Control Point Input Fields ---
function updateManualCtrlPtInput(src, fig, index)
% updateManualCtrlPtInput validates the 3D vector input and stores it, then updates plot.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    currentValueStr = src.Value;

    try
        parsedVector = eval(currentValueStr);
        if ~isnumeric(parsedVector) || ~isvector(parsedVector) || length(parsedVector) ~= 3
            error('Input must be a 3x1 numeric vector, e.g., [1;2;3] or 1;2;3');
        end
        
        parsedVector = parsedVector(:); % Ensure it's a column vector

        appData.manual_ctrl_pts(:, index) = parsedVector;
        setappdata(fig, 'appData', appData);
        src.BackgroundColor = [1 1 1];
        
        % --- LIVE PLOT UPDATE ---
        % Pass all relevant manual data to updatePathPlot
        updatePathPlot(getappdata(fig, 'pathAxesHandle'), ...
                       appData.path_type, appData.mode, appData.p0, appData.thz_0, appData.thz_end, ...
                       appData.num_manual_ctrl_pts, appData.manual_ctrl_pts);
        drawnow; % Ensure immediate refresh
    catch ME
        uialert(fig, ME.message, 'Input Error', 'Icon', 'error');
        src.BackgroundColor = [1 0.9 0.9]; % Highlight error
        % Even on error, update the plot to show existing valid points, but mark the invalid one with NaN
        appData.manual_ctrl_pts(:, index) = NaN(3,1); % Mark as invalid
        setappdata(fig, 'appData', appData);
        updatePathPlot(getappdata(fig, 'pathAxesHandle'), ...
                       appData.path_type, appData.mode, appData.p0, appData.thz_0, appData.thz_end, ...
                       appData.num_manual_ctrl_pts, appData.manual_ctrl_pts);
        drawnow;
    end
end

% --- Callback Function for OK Button ---
function okButtonPushed(fig)
% okButtonPushed sets the confirmed flag and resumes the figure after final validation.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    
    % --- Final Validation for Manual Path Type ---
    if strcmp(appData.path_type, 'manual')
        manualCtrlPtsFields = getappdata(fig, 'manualCtrlPtsFields');
        allManualInputsValid = true;
        
        % Ensure all required fields exist and are valid (important if N changed)
        if length(manualCtrlPtsFields) ~= appData.num_manual_ctrl_pts
            uialert(fig, 'Number of control point fields does not match specified N. Please re-enter N or correct inputs.', 'Validation Error', 'Icon', 'error');
            return; % Do not proceed if fields are inconsistent
        end

        for i = 1:appData.num_manual_ctrl_pts
            currentValueStr = manualCtrlPtsFields(i).Value;
            try
                parsedVector = eval(currentValueStr);
                if isempty(parsedVector) || ~isnumeric(parsedVector) || numel(parsedVector) ~= 3
                    error('Missing or invalid 3D vector for Point %d.', i);
                end
                parsedVector = parsedVector(:); % Ensure column vector
                appData.manual_ctrl_pts(:, i) = parsedVector; % Update with final valid value
                manualCtrlPtsFields(i).BackgroundColor = [1 1 1]; % Reset background
            catch ME
                uialert(fig, ME.message, 'Validation Error', 'Icon', 'error');
                manualCtrlPtsFields(i).BackgroundColor = [1 0.9 0.9]; % Highlight invalid
                allManualInputsValid = false;
            end
        end
        
        if ~allManualInputsValid
            setappdata(fig, 'appData', appData); % Save potentially updated (but still invalid) appData
            return; % Do not confirm if any manual input is invalid
        end
    end

    % --- Confirm and Resume ---
    appData.confirmed = true;
    setappdata(fig, 'appData', appData);
    uiresume(fig);
end

% --- Callback Function for Cancel Button ---
function cancelButtonPushed(fig)
% cancelButtonPushed ensures the confirmed flag is false and resumes the figure.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false;
    setappdata(fig, 'appData', appData);
    uiresume(fig);
end

% --- Custom Close Request Function ---
function closeFigure(src, fig)
% closeFigure handles closing the figure, ensuring uiresume is called.
    if ~isvalid(fig)
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false;
    setappdata(fig, 'appData', appData);
    uiresume(fig);
end

% --- Helper Function: Rotation Matrix around Z-axis ---
function R = Rz(theta)
    % Rz creates a 3x3 rotation matrix around the Z-axis.
    R = [cos(theta) -sin(theta) 0;
         sin(theta)  cos(theta) 0;
         0           0          1];
end

% --- Function to Update Path Plot ---
function updatePathPlot(ax, path_type, mode, p0, thz_0, thz_end, num_manual_ctrl_pts, manual_ctrl_pts)
% updatePathPlot generates a 3D plot of the selected path.
% Inputs:
%   ax: The uiaxes object to plot on.
%   path_type: Selected path type.
%   mode: Selected mode ('absolute' or 'relative').
%   p0: Initial position vector (for 'relative' mode).
%   thz_0: Initial Angle (ALWAYS passed for display purposes).
%   thz_end: End Angle (ALWAYS passed for display purposes).
%   num_manual_ctrl_pts: Number of manual control points (if path_type is 'manual').
%   manual_ctrl_pts: 3xN matrix of manual control points (if path_type is 'manual').

    cla(ax); % Clear the specified axes
    hold(ax, 'on');

    local_cpx = [];
    local_cpy = [];
    local_cpz = [];
    local_n_ctrl_pts = 0;
    
    % Flag to indicate if spline calculation should be attempted
    shouldCalculateSpline = true; 

    try
        if strcmp(path_type, 'manual')
            % For manual path, we have to check if all points are valid first
            if ~isempty(manual_ctrl_pts) && ~any(isnan(manual_ctrl_pts(:))) && (size(manual_ctrl_pts, 2) == num_manual_ctrl_pts)
                % All points are valid and accounted for, so calculate path via getPath
                if strcmp(mode, 'relative')
                    [local_cpx, local_cpy, local_cpz, local_n_ctrl_pts] = getPath(path_type, mode, p0, num_manual_ctrl_pts, manual_ctrl_pts);
                else % absolute
                    [local_cpx, local_cpy, local_cpz, local_n_ctrl_pts] = getPath(path_type, mode, num_manual_ctrl_pts, manual_ctrl_pts);
                end
            else
                % Not all manual control points are filled or are invalid.
                % We will still plot the valid individual points, but skip spline calculation.
                local_cpx = manual_ctrl_pts(1, ~isnan(manual_ctrl_pts(1,:)));
                local_cpy = manual_ctrl_pts(2, ~isnan(manual_ctrl_pts(2,:)));
                local_cpz = manual_ctrl_pts(3, ~isnan(manual_ctrl_pts(3,:)));
                local_n_ctrl_pts = length(local_cpx); % Number of valid points entered so far
                shouldCalculateSpline = false; % Do not attempt spline
            end
        else
            % For other path types, call getPath
            [local_cpx, local_cpy, local_cpz, local_n_ctrl_pts] = getPath(path_type, mode, p0);
        end
    catch ME
        % Display error message on plot if path calculation fails
        text(ax, 0, 0, 0, sprintf('Error in path calculation:\n%s', ME.message), ...
             'Color', 'red', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        title(ax, 'Path Visualization Error');
        axis(ax, 'equal');
        grid(ax, 'on');
        hold(ax, 'off');
        return;
    end

    % Always plot control points if available
    if ~isempty(local_cpx)
        plot3(ax, local_cpx, local_cpy, local_cpz, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5); % Black circles for control points
    end

    % --- CasADi B-spline Interpolation ---
    % Only attempt if all conditions met (especially for manual mode)
    if shouldCalculateSpline && local_n_ctrl_pts >= 2
        try
            ctrl_pts_matrix = [local_cpx; local_cpy; local_cpz]; % Combine control points into a 3xN matrix
            dim_spline = 3;
            degree = 4; % Changed to 3rd degree for better flexibility with fewer points

            num_linspace_points = max(1, local_n_ctrl_pts - degree + 1);
            knots = [zeros(1, degree) - 1e-3, linspace(0, 1, num_linspace_points), ones(1, degree) + 1e-3];

            get_p = casadi.Function.bspline('evaluateSpline', {knots}, ctrl_pts_matrix(:), {degree}, dim_spline);
            s_eval = linspace(0, 1, 200); % 200 points for smoothness
    
           
            % Evaluate the spline using CasADi function
            spline_pts_casadi = full(get_p(s_eval));
            spline_x = spline_pts_casadi(1,:);
            spline_y = spline_pts_casadi(2,:);
            spline_z = spline_pts_casadi(3,:);
            
            plot3(ax, spline_x, spline_y, spline_z, 'b-', 'LineWidth', 1.5); % Blue line for the spline
            
        catch ME_CasADi
            % Display error message on plot if CasADi fails
            text(ax, 0, 0, 0, sprintf('CasADi spline error:\n%s\n(Is CasADi installed and on path?)', ME_CasADi.message), ...
                 'Color', 'red', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            title(ax, 'Path Visualization Error (CasADi)');
            % Control points are already plotted
        end
    end

    % --- Set Plot Properties ---
    axis(ax, 'equal'); % Maintain aspect ratio
    grid(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    
    if strcmp(path_type, 'manual')
        title(ax, sprintf('Path Type: %s, N: %d, thz_0: %.2f, thz_end: %.2f', path_type, num_manual_ctrl_pts, thz_0,thz_end));
    else
        title(ax, sprintf('Path Type: %s, Mode: %s, thz_0: %.2f, thz_end: %.2f', path_type, mode, thz_0, thz_end));
    end
    view(ax, 3); % 3D view
    
    % Adjust axis limits dynamically based on control points
    % Use all manual_ctrl_pts directly for limits, even if some are NaN,
    % because the plot might show partially filled points.
    effective_cpx = manual_ctrl_pts(1, ~isnan(manual_ctrl_pts(1,:))) ;
    effective_cpy = manual_ctrl_pts(2, ~isnan(manual_ctrl_pts(2,:))) ;
    effective_cpz = manual_ctrl_pts(3, ~isnan(manual_ctrl_pts(3,:))) ;

    % If path_type is not manual, use the points from local_cpx, _cpy, _cpz
    if ~strcmp(path_type, 'manual')
         effective_cpx = local_cpx;
         effective_cpy = local_cpy;
         effective_cpz = local_cpz;
    end

    if ~isempty(effective_cpx)
        min_x = min(effective_cpx); max_x = max(effective_cpx);
        min_y = min(effective_cpy); max_y = max(effective_cpy);
        min_z = min(effective_cpz); max_z = max(effective_cpz);
        
        % Add padding to limits, ensure non-zero range
        padding = 0.2; % meters
        xRange = max(0.5, max_x - min_x); % Ensure min range for visibility
        yRange = max(0.5, max_y - min_y);
        zRange = max(0.5, max_z - min_z);

        xlim(ax, [min_x - padding * xRange/0.5, max_x + padding * xRange/0.5]);
        ylim(ax, [min_y - padding * yRange/0.5, max_y + padding * yRange/0.5]);
        zlim(ax, [min_z - padding * zRange/0.5, max_z + padding * zRange/0.5]);
    else
        % Default limits if no points are plotted
        xlim(ax, [-1 1]);
        ylim(ax, [-1 1]);
        zlim(ax, [0 2]);
    end
    hold(ax, 'off');
end