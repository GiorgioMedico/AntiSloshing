function [save_fig, save_csv, save_mat, csv_joint_offset, animations] = getSimulationParameters()
% getSimulationParameters creates an interactive GUI for defining
% simulation output and control parameters.
%
% The GUI allows the user to input:
%   - save_fig: Boolean to save figures.
%   - save_csv: Boolean to save data to CSV.
%   - save_mat: Boolean to save data to MAT file.
%   - csv_joint_offset: A 6x1 vector offset for CSV joint data.
%   - animations: Boolean to enable/disable animations.
%
% Default values are provided. Descriptions for boolean parameters are
% displayed dynamically.
%
% Returns:
%   save_fig: Boolean value for saving figures.
%   save_csv: Boolean value for saving CSV.
%   save_mat: Boolean value for saving MAT.
%   csv_joint_offset: 6x1 numeric vector.
%   animations: Boolean value for animations.

    % --- Default Values ---
    defaultSaveFig = false;
    defaultSaveVideo = false;
    defaultSaveCsv = false;
    defaultSaveMat = false;
    defaultCsvJointOffset = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
    defaultSolved = false;
    defaultAnimations = false;

    % Initialize output variables with default values
    save_fig = defaultSaveFig;
    save_csv = defaultSaveCsv;
    save_mat = defaultSaveMat;
    csv_joint_offset = defaultCsvJointOffset;
    animations = defaultAnimations;

    % --- Create UI Figure ---
    fig = uifigure('Name', 'Simulation Output Parameters', ...
                   'Position', [100 100 500 700], ... 
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src, fig));

    % Store data in the figure's appdata for easy access by callbacks
    appData = struct();
    appData.save_fig = defaultSaveFig;
    appData.save_csv = defaultSaveCsv;
    appData.save_mat = defaultSaveMat;
    appData.csv_joint_offset = defaultCsvJointOffset;
    appData.animations = defaultAnimations;
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---

    % Panel for inputs
    inputPanel = uipanel(fig, 'Title', 'Simulation Parameters', ...
                         'Position', [20 20 460 660], ...
                         'FontWeight', 'bold');

    % Helper function for creating boolean text fields and their labels/explanations
    function [textField, explanationLabel] = createBooleanControl(parent, text, value, yPos, explanationText, paramName)
        uilabel(parent, 'Text', text, ...
                'Position', [20 yPos 200 22]);
        
        % Convert logical value to string 'true' or 'false' for initial display
        initialTextValue = char(string(value)); 

        textField = uieditfield(parent, 'text', ... 
                                'Value', initialTextValue, ...
                                'Position', [220 yPos 220 22], ...
                                'ValueChangedFcn', @(src,event) updateParameters(src, fig, paramName));
        explanationLabel = uilabel(parent, ...
                                   'Text', explanationText, ...
                                   'Position', [20 (yPos - 40) 420 35], ... % Adjusted for multiline
                                   'VerticalAlignment', 'top', ...
                                   'FontColor', [0.5 0.5 0.5]);
    end

    % --- Save Figure ---
    [saveFigField, saveFigExplanation] = createBooleanControl(inputPanel, 'Save Figure (true/false):', defaultSaveFig, 600, ...
        'If true, simulation figures will be saved to disk.', 'save_fig');

    % --- Save CSV ---
    [saveCsvField, saveCsvExplanation] = createBooleanControl(inputPanel, 'Save CSV (true/false):', defaultSaveCsv, 530, ...
        'If true, generated joint trajectory will be exported to a CSV file.', 'save_csv');

    % --- Save MAT ---
    [saveMatField, saveMatExplanation] = createBooleanControl(inputPanel, 'Save MAT (true/false):', defaultSaveMat, 460 , ...
        'If true, eta results will be saved to a .mat file.', 'save_mat');

    % --- Animations ---
    [animationsField, animationsExplanation] = createBooleanControl(inputPanel, 'Animations (true/false):', defaultAnimations, 390, ...
        'If true, visual animations of the simulation will be displayed.', 'animations');

    % --- CSV Joint Offset ---
    uilabel(inputPanel, 'Text', 'CSV Joint Offset (6x1 vector):', ...
            'Position', [20 180 200 22]);
    csvJointOffsetField = uieditfield(inputPanel, 'text', ...
                                      'Value', mat2str(defaultCsvJointOffset), ... % Convert vector to string
                                      'Position', [220 180 220 22], ...
                                      'ValueChangedFcn', @(src,event) updateParameters(src, fig, 'csv_joint_offset'));
    uilabel(inputPanel, 'Text', 'Enter as [val1; val2; ...; val6]', ...
            'Position', [20 150 420 20], ...
            'FontColor', [0.5 0.5 0.5]);


    % --- OK and Cancel Buttons ---
    uibutton(inputPanel, 'Text', 'OK', ...
             'Position', [100 60 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    uibutton(inputPanel, 'Text', 'Cancel', ...
             'Position', [260 60 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Wait for User Action with Error Handling ---
    try
        uiwait(fig);
    finally
        disp('getSimulationParameters: uiwait returned. Entering finally block.'); % Trace
        if isvalid(fig)
            finalAppData = getappdata(fig, 'appData'); % Retrieve data BEFORE deleting
            if finalAppData.confirmed
                % disp('getSimulationParameters: OK button pressed. Assigning current values.'); % Trace
                save_fig = finalAppData.save_fig;
                save_csv = finalAppData.save_csv;
                save_mat = finalAppData.save_mat;
                csv_joint_offset = finalAppData.csv_joint_offset;
                animations = finalAppData.animations;
            else
                disp('getSimulationParameters: Cancel button pressed or window closed. Assigning default values.'); % Trace
                % If cancelled, return default values
                save_fig = defaultSaveFig;
                save_csv = defaultSaveCsv;
                save_mat = defaultSaveMat;
                csv_joint_offset = defaultCsvJointOffset;
                animations = defaultAnimations;
            end
            delete(fig); % Delete the figure here, after all data retrieval
            disp('getSimulationParameters: Figure deleted in finally block.'); % Trace
        else
            disp('getSimulationParameters: Figure already invalid.'); % Trace
            % Output variables remain at their initial NaN/default values.
        end
    end

    finalAppData = getappdata(fig, 'appData');
    if finalAppData.confirmed
        save_fig = finalAppData.save_fig;
        save_csv = finalAppData.save_csv;
        save_mat = finalAppData.save_mat;
        csv_joint_offset = finalAppData.csv_joint_offset;
        animations = finalAppData.animations;
    else
        save_fig = defaultSaveFig;
        save_csv = defaultSaveCsv;
        save_mat = defaultSaveMat;
        csv_joint_offset = defaultCsvJointOffset;
        animations = defaultAnimations;
    end


    if isvalid(fig)
        delete(fig);
    end

end

% --- Callback Function for Input Fields ---
function updateParameters(src, fig, paramName)
% updateParameters updates the stored parameter value.
    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        disp('updateParameters: Figure is invalid, skipping update.');
        return;
    end

    appData = getappdata(fig, 'appData');
    newValue = src.Value;

    % Convert text input to logical for boolean parameters
    if any(strcmp(paramName, {'save_fig', 'save_csv', 'save_mat', 'animations'}))
        % Validate input: allow 'true' or 'false' (case-insensitive)
        if strcmpi(newValue, 'true')
            newValue = true;
        elseif strcmpi(newValue, 'false')
            newValue = false;
        else
            uialert(fig, ['Invalid input for ' paramName '. Please enter ''true'' or ''false''.'], 'Input Error');
            % Revert to previous valid value
            src.Value = char(string(appData.(paramName)));
            return;
        end
    end

    % Special handling for csv_joint_offset (string to numeric array)
    if strcmp(paramName, 'csv_joint_offset')
        try
            % Evaluate the string as a MATLAB expression (e.g., '[0.6;0;0;0;0;0]')
            parsedValue = eval(newValue);
            if ~isnumeric(parsedValue) || ~isvector(parsedValue) || length(parsedValue) ~= 6
                error('Input must be a 6x1 numeric vector, e.g., [0.6;0;0;0;0;0]');
            end
            newValue = parsedValue(:); % Ensure it's a column vector
        catch ME
            uialert(fig, ['Invalid input for CSV Joint Offset: ' ME.message], 'Input Error');
            src.Value = mat2str(appData.csv_joint_offset); % Revert to previous valid value
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
    % disp('okButtonPushed: OK button pushed. Calling uiresume.'); % Trace
    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        disp('okButtonPushed: Figure is invalid, skipping uiresume.');
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
    disp('cancelButtonPushed: Cancel button pushed. Calling uiresume.'); % Trace
    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        disp('cancelButtonPushed: Figure is invalid, skipping uiresume.');
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
    disp('closeFigure: CloseRequestFcn triggered. Calling uiresume.'); % Trace
    % Ensure figure is still valid before proceeding
    if ~isvalid(fig)
        disp('closeFigure: Figure is invalid, skipping uiresume.');
        return;
    end
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Treat closing as cancellation
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end
