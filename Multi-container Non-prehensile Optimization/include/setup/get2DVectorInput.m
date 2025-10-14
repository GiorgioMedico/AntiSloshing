function [outputVectors] = get2DVectorInput(numContainers)
% get2DVectorInput creates an interactive GUI for defining multiple 2D vectors.
%
% This function prompts the user to input 'numContainers' number of 2D vectors.
% Each vector should be entered in the format [x y] or x,y.
%
% Inputs:
%   numContainers: The number of 2D vectors the user needs to specify.
%                  Must be a positive integer.
%
% Returns:
%   outputVectors: A 2xnumContainers matrix where each column is a 2D vector
%                  specified by the user. If the user cancels or closes the window,
%                  a 2xnumContainers matrix of NaNs is returned.

    % --- Input Validation ---
    if ~isnumeric(numContainers) || ~isscalar(numContainers) || numContainers <= 0 || mod(numContainers, 1) ~= 0
        error('numContainers must be a positive integer.');
    end

    % --- Default Values and Initialization ---
    % Initialize outputVectors with NaNs. This will be returned if cancelled.
    defaultVectors = NaN(2, numContainers);

    % --- Create UI Figure ---
    figWidth = 400;
    % Adjust figure height dynamically to fit inputs, with a max height
    maxFigContentHeight = 500; % Max height for the scrollable area
    minFigHeight = 180; % Minimum height for header, buttons, etc.
    
    % Calculate content height needed for vectors + header + spacing
    vectorsContentHeight = numContainers * 35; % 35 pixels per label+field row
    
    % Panel height will be maxFigContentHeight, or less if fewer vectors
    panelHeight = min(maxFigContentHeight, vectorsContentHeight + 60); % Add some padding for panel title

    % Total figure height
    figHeight = minFigHeight + panelHeight;
    
    fig = uifigure('Name', 'Define 2D Vectors', ...
                   'Position', [100 100 figWidth figHeight], ...
                   'WindowStyle', 'modal', ...
                   'CloseRequestFcn', @(src,event) closeFigure(src));

    % Store data in the figure's appdata for easy access by callbacks
    appData = struct();
    appData.vectors = defaultVectors; % Store vectors as a 2xN matrix
    appData.numContainers = numContainers;
    appData.confirmed = false; % Flag to check if OK was pressed
    setappdata(fig, 'appData', appData);

    % --- Create UI Components ---

    % Panel for inputs - make it scrollable if content exceeds height
    inputPanel = uipanel(fig, 'Title', 'Enter 2D Vectors (e.g., [1.0 2.5] or 1.0, 2.5)', ...
                         'Position', [20 70 figWidth-40 panelHeight], ... % Positioned above buttons
                         'FontWeight', 'bold', ...
                         'Scrollable', 'on');

    % Array to store handles of the input fields for later access
    vectorFields = gobjects(numContainers, 1);

    % Dynamically create labels and edit fields for each vector
    startY = panelHeight - 30; % Starting Y position for the first element within the panel
    for i = 1:numContainers
        % Label for the vector input
        uilabel(inputPanel, 'Text', sprintf('Vector %d (x, y):', i), ...
                'Position', [20 startY - (i-1)*35 120 22]);

        % Edit field for the vector input
        vectorFields(i) = uieditfield(inputPanel, 'text', ... % <--- CORRECTED STYLE HERE
                                      'Value', '', ... % Default empty
                                      'Position', [150 startY - (i-1)*35 200 22], ...
                                      'Tag', sprintf('vectorField_%d', i), ... % Add a tag for easy identification
                                      'ValueChangedFcn', @(src,event) updateVectorInput(src, fig, i));
    end
    
    % Store handles to the vector input fields
    setappdata(fig, 'vectorFields', vectorFields);

    % --- OK and Cancel Buttons ---
    uibutton(fig, 'Text', 'OK', ...
             'Position', [figWidth/2 - 100 20 90 30], ...
             'ButtonPushedFcn', @(src,event) okButtonPushed(fig));

    uibutton(fig, 'Text', 'Cancel', ...
             'Position', [figWidth/2 + 10 20 90 30], ...
             'ButtonPushedFcn', @(src,event) cancelButtonPushed(fig));

    % --- Wait for User Action ---
    uiwait(fig);

    % --- Retrieve Final Values and Close Figure ---
    finalAppData = getappdata(fig, 'appData');
    if finalAppData.confirmed
        % If OK was pressed, return the current valid vectors
        outputVectors = finalAppData.vectors;
    else
        % If Cancel was pressed or window closed, return the default (NaNs)
        outputVectors = defaultVectors;
    end

    % Clean up the figure
    if isvalid(fig)
        delete(fig);
    end

end


% --- Callback Function for Individual Vector Input Fields ---
function updateVectorInput(src, fig, index)
% updateVectorInput validates the 2D vector input and stores it.
    appData = getappdata(fig, 'appData');
    currentValueStr = src.Value;

    try
        % Attempt to evaluate the string as a numeric array
        % Using str2num is convenient but be aware of its limitations for complex parsing
        parsedVector = str2num(currentValueStr); %#ok<ST2NM>

        % Validate that it's a 1x2 or 2x1 vector
        if isempty(parsedVector) || ~isnumeric(parsedVector) || numel(parsedVector) ~= 2
            error('Invalid format. Please enter two numbers (e.g., [1 2] or 1,2).');
        end
        
        % Ensure it's a column vector for storage in 2xN matrix
        if isrow(parsedVector)
            parsedVector = parsedVector';
        end

        % Store the validated vector in the appData matrix
        appData.vectors(:, index) = parsedVector;
        setappdata(fig, 'appData', appData);
        src.BackgroundColor = [1 1 1]; % Reset background to white on valid input
    catch ME
        uialert(fig, ME.message, 'Input Error', 'Icon', 'error');
        src.BackgroundColor = [1 0.9 0.9]; % Highlight invalid input with light red
        % Keep the invalid value in the field, let the user correct it.
        % The corresponding column in appData.vectors remains its previous valid value or NaN.
    end
end

% --- Callback Function for OK Button ---
function okButtonPushed(fig)
% okButtonPushed sets the confirmed flag and resumes the figure after final validation.
    appData = getappdata(fig, 'appData');
    vectorFields = getappdata(fig, 'vectorFields');
    
    % Perform final validation on all fields
    allInputsValid = true;
    for i = 1:appData.numContainers
        currentValueStr = vectorFields(i).Value;
        try
            parsedVector = str2num(currentValueStr); %#ok<ST2NM>
            if isempty(parsedVector) || ~isnumeric(parsedVector) || numel(parsedVector) ~= 2
                error('Missing or invalid 2D vector for Container %d.', i);
            end
            if isrow(parsedVector)
                parsedVector = parsedVector';
            end
            appData.vectors(:, i) = parsedVector; % Update appData with final values
            vectorFields(i).BackgroundColor = [1 1 1]; % Ensure background is white
        catch ME
            uialert(fig, ME.message, 'Validation Error', 'Icon', 'error');
            vectorFields(i).BackgroundColor = [1 0.9 0.9]; % Highlight invalid input
            allInputsValid = false;
        end
    end

    if allInputsValid
        appData.confirmed = true;
        setappdata(fig, 'appData', appData);
        uiresume(fig); % Resume execution of the main function
    else
        % If not all inputs are valid, do not close the window, let the user correct them.
        % The uialert serves as feedback.
    end
end

% --- Callback Function for Cancel Button ---
function cancelButtonPushed(fig)
% cancelButtonPushed ensures the confirmed flag is false and resumes the figure.
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Explicitly set to false
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end

% --- Custom Close Request Function ---
function closeFigure(fig)
% closeFigure handles closing the figure, ensuring uiresume is called.
    appData = getappdata(fig, 'appData');
    appData.confirmed = false; % Treat closing as cancellation
    setappdata(fig, 'appData', appData);
    uiresume(fig); % Resume execution of the main function
end