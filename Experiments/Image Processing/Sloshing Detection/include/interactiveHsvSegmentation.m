% function [final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max] = interactiveHsvSegmentation(initialFrame)
% %interactiveHsvSegmentation Interactively segment an image using HSV thresholds.
% %   This function allows you to adjust H, S, and V min/max values
% %   and see the segmentation results continuously updated.
% %   Press Enter without typing values to return the current thresholds.
% %   Type "q" to quit without saving.
% %
% %   Input:
% %     initialFrame - The initial RGB image to segment.
% %
% %   Output:
% %     final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max
% %     The HSV threshold values that were active when Enter was pressed.
% 
% final_h_min = [];
% final_h_max = [];
% final_s_min = [];
% final_s_max = [];
% final_v_min = [];
% final_v_max = [];
% 
% if nargin < 1
%     error("No frame provided");
% end
% 
% % Convert initial frame to double for consistent processing
% initialFrame = im2double(initialFrame);
% 
% % Initial HSV values
% h_min = 88;
% h_max = 120;
% s_min = 130;
% s_max = 255;
% v_min = 0;
% v_max = 255;
% 
% % Create a figure for displaying the original and segmented images
% figure('Name', 'Interactive HSV Segmentation', 'NumberTitle', 'off');
% subplot(1, 2, 1);
% imshow(initialFrame);
% title('Original Image');
% 
% segmented_img_handle = subplot(1, 2, 2);
% title('Segmented Image');
% 
% % Instructions for the user
% disp('----------------------------------------------------');
% disp('Interactive HSV Segmentation Tool');
% disp('  1. Enter 6 numbers (h_min h_max s_min s_max v_min v_max) to update values.');
% disp('  2. Press **Enter** (without typing values) to save the current values and quit.');
% disp('  3. Type "q" and press Enter to quit without saving.');
% disp('----------------------------------------------------');
% 
% command = '';
% while true
% 
%     % Convert RGB to HSV
%     hsv_frame = rgb2hsv(initialFrame);
% 
%     % Normalize min/max values to [0, 1] range for HSV
%     % As noted previously, this normalization assumes your rgb2hsv outputs H in [0,180] and S/V in [0,255].
%     % If using standard MATLAB rgb2hsv (outputs all [0,1]), adjust normalization accordingly.
%     minval = [h_min/180, s_min/255, v_min/255];
%     maxval = [h_max/180, s_max/255, v_max/255];
% 
%     % Initialize logical image
%     segmented_frame = true(size(hsv_frame, 1), size(hsv_frame, 2));
% 
%     % Apply thresholding for each channel
%     for ii = 1 : 3
%         segmented_frame = segmented_frame & (hsv_frame(:,:,ii) >= minval(ii) & hsv_frame(:,:,ii) <= maxval(ii));
%     end
% 
%     % Invert the segmentation
%     segmented_frame_display = ~segmented_frame;
% 
%     % Display the segmented image
%     imshow(segmented_frame_display, 'Parent', segmented_img_handle);
%     title(segmented_img_handle, sprintf('Segmented Image\nH: [%d, %d] S: [%d, %d] V: [%d, %d]', h_min, h_max, s_min, s_max, v_min, v_max));
%     drawnow; % Update the figure
% 
%     % Get user input
%     prompt = sprintf('\nCurrent values: H:[%d %d] S:[%d %d] V:[%d %d]\nEnter new values, press Enter to save, or "q" to quit: >> ', ...
%         h_min, h_max, s_min, s_max, v_min, v_max);
%     command = input(prompt, 's');
% 
%     if isempty(command)
%         % User pressed Enter without typing anything - save current values
%         disp('Enter pressed. Saving current values and exiting.');
%         final_h_min = h_min;
%         final_h_max = h_max;
%         final_s_min = s_min;
%         final_s_max = s_max;
%         final_v_min = v_min;
%         final_v_max = v_max;
%         break; % Exit the loop
%     elseif strcmp(command, 'q')
%         final_h_min = h_min;
%         final_h_max = h_max;
%         final_s_min = s_min;
%         final_s_max = s_max;
%         final_v_min = v_min;
%         final_v_max = v_max;
%         % User typed 'q' - quit without saving
%         disp('Quit command received. Exiting without saving.');
% 
%         break; % Exit the loop
%     end
% 
%     % Try to parse the input as new values
%     new_values = str2num(command); %#ok<ST2NM>
% 
%     if numel(new_values) == 6
%         % Validate ranges for HSV
%         if all(new_values >= 0) && new_values(1) <= 180 && new_values(2) <= 180 && ...
%            new_values(3) <= 255 && new_values(4) <= 255 && ...
%            new_values(5) <= 255 && new_values(6) <= 255 && ...
%            new_values(1) <= new_values(2) && ...
%            new_values(3) <= new_values(4) && ...
%            new_values(5) <= new_values(6)
% 
%             h_min = new_values(1);
%             h_max = new_values(2);
%             s_min = new_values(3);
%             s_max = new_values(4);
%             v_min = new_values(5);
%             v_max = new_values(6);
% 
%             disp('Values updated.');
%         else
%             disp('Invalid input. Please ensure values are within valid HSV ranges (H: 0-180, S/V: 0-255) and min <= max for each channel.');
%         end
%     else
%         disp('Invalid input format. Please enter 6 numbers (h_min h_max s_min s_max v_min v_max), press Enter to save, or "q" to quit.');
%     end
% end
% 
% close(gcf); % Close the figure
% end

function [final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max] = interactiveHsvSegmentation(initialFrame)
%interactiveHsvSegmentation Interactively segment an image using HSV thresholds.
%   This function allows you to adjust H, S, and V min/max values
%   and see the segmentation results continuously updated.
%   Press Enter without typing values to return the current thresholds.
%   Type "q" to quit without saving.
%
%   Input:
%     initialFrame - The initial RGB image to segment.
%
%   Output:
%     final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max
%     The HSV threshold values that were active when Enter was pressed.


final_h_min = [];
final_h_max = [];
final_s_min = [];
final_s_max = [];
final_v_min = [];
final_v_max = [];

if nargin < 1 || isempty(initialFrame)
    error("An initial RGB image frame must be provided for interactive HSV segmentation.");
end

% Convert initial frame to double for consistent processing
initialFrame = im2double(initialFrame);

% Store initial default HSV values (for 'Cancel' action)
default_h_min = 88;
default_h_max = 120;
default_s_min = 130;
default_s_max = 255;
default_v_min = 0;
default_v_max = 255;

% Current active HSV values (start at defaults)
h_min = default_h_min;
h_max = default_h_max;
s_min = default_s_min;
s_max = default_s_max;
v_min = default_v_min;
v_max = default_v_max;

% Create a figure for displaying the original and segmented images
hFig = figure('Name', 'Interactive HSV Segmentation', 'NumberTitle', 'off');

% Adjust figure size for better visibility (pixels)
set(hFig, 'Position', [50, 100, 1400, 700]); % Wider for two subplots, taller for detail

subplot(1, 2, 1);
imshow(initialFrame);
title('Original Image');
xlabel('This is your raw image. Observe the liquid''s color.');

segmented_img_handle = subplot(1, 2, 2);
hImg = imshow(false(size(initialFrame, 1), size(initialFrame, 2))); % Initialize with empty logical image
title('Segmented Image');
xlabel('This view shows the pixels that match your current HSV thresholds.');

% --- User Instructions in Command Window ---
disp(' ');
disp('======================================================================================');
disp('                  --- Interactive HSV Segmentation Tool ---');
disp('======================================================================================');
disp(' ');
disp('  **Goal:** Refine the Hue (H), Saturation (S), and Value (V) thresholds');
disp('  to accurately segment your liquid. The segmented areas will appear WHITE.');
disp(' ');
disp('  **HSV Ranges for Input:**');
disp('  * **Hue (H):** 0 to 180 (Represents color type: e.g., red, green, blue. Circular range.)');
disp('  * **Saturation (S):** 0 to 255 (Represents color purity/intensity, from dull to vivid)');
disp('  * **Value (V):** 0 to 255 (Represents brightness, from dark to bright)');
disp(' ');
disp('  ------------------------------------------------------------------------------------');
disp('  **How to Use (using the Pop-Up Input Dialog):**');
disp('  1. A dialog box will appear, pre-filled with the current HSV values.');
disp('  2. Adjust any of the **SIX input fields** (H_min, H_max, S_min, S_max, V_min, V_max).');
disp('  3. Click **"OK"** to apply your changes. The segmented image will update,');
disp('     and the dialog will re-appear for further tuning.');
disp(' ');
disp('  **When you are satisfied with the segmentation:**');
disp('  * Simply click **"OK"** *without changing any values* in the dialog.');
disp('    This will save the currently displayed thresholds and close the tool.');
disp(' ');
disp('  **To use default values and exit:**');
disp('  * Click the **"Cancel"** button in the dialog at any time. This will exit');
disp('    the tool and return the *initial default HSV values*.');
disp('======================================================================================');
disp(' ');
% --- End User Instructions ---

% Convert RGB to HSV (MATLAB's rgb2hsv outputs H,S,V in [0,1] range for double inputs)
hsv_frame = rgb2hsv(initialFrame);

while true % Loop for continuous adjustment
    % Normalize min/max values from user's 0-180/0-255 ranges to MATLAB's 0-1 internal HSV data range.
    minval = [h_min/180, s_min/255, v_min/255];
    maxval = [h_max/180, s_max/255, v_max/255];

    % Initialize logical image (all true, then AND with thresholds)
    segmented_logical = true(size(hsv_frame, 1), size(hsv_frame, 2));

    % Apply thresholding for each channel
    for ii = 1 : 3
        segmented_logical = segmented_logical & (hsv_frame(:,:,ii) >= minval(ii) & hsv_frame(:,:,ii) <= maxval(ii));
    end
    
    segmented_frame_display = ~segmented_logical; % Removed the inversion here.

    % Display the segmented image
    set(hImg, 'CData', segmented_frame_display); % Updated to use corrected variable
    title(segmented_img_handle, sprintf('Segmented Image\nH: [%d, %d] S: [%d, %d] V: [%d, %d]', h_min, h_max, s_min, s_max, v_min, v_max));
    drawnow; % Update the figure display

    % --- Pop-up Input Dialog Configuration ---
    % Combined current values and general instruction in the dialog title.
    % Prompt labels for each of the 6 individual input fields.
    prompt_dlg = {'Hue Min:', 'Hue Max:', ...
                  'Saturation Min:', 'Saturation Max:', ...
                  'Value Min:', 'Value Max:'};
    
    dlgtitle_dlg = sprintf('Adjust HSV Thresholds (Current: H:[%d %d] S:[%d %d] V:[%d %d])', h_min, h_max, s_min, s_max, v_min, v_max);
    
    % Dimensions for each input field (single row, 20 chars wide for compactness).
    dims_dlg = [1 20]; % Applies to all prompt lines
    
    % Default input for each of the 6 editable fields, pre-filled with current values.
    definput_dlg = {num2str(h_min), num2str(h_max), ...
                    num2str(s_min), num2str(s_max), ...
                    num2str(v_min), num2str(v_max)};

    % Wait for the user to interact with the input dialog
    answer_dlg = inputdlg(prompt_dlg, dlgtitle_dlg, dims_dlg, definput_dlg);

    % Handle user input from the dialog
    if isempty(answer_dlg) % User clicked 'Cancel' button
        % --- Cancel returns default values ---
        disp(' ');
        disp('--- "Cancel" clicked. Returning to default HSV thresholds. ---');
        final_h_min = default_h_min;
        final_h_max = default_h_max;
        final_s_min = default_s_min;
        final_s_max = default_s_max;
        final_v_min = default_v_min;
        final_v_max = default_v_max;
        break; % Exit the loop
    end

    % Parse new values from each input cell
    new_values = zeros(1,6);
    input_changed = false; % Flag to detect if any value was modified
    valid_parse = true;

    for i = 1:6
        % Convert input string to number
        val_str = answer_dlg{i};
        val = str2double(val_str);
        
        % Check if input is a valid number
        if isnan(val)
            valid_parse = false;
            break; % Exit loop if any value is not a valid number
        end
        new_values(i) = val;

        % Check if the current value is different from the original value in the dialog
        % This is to detect if the user made any changes to the pre-filled input.
        current_def_val = str2double(definput_dlg{i});
        if current_def_val ~= val % Compare numerical values
            input_changed = true;
        end
    end

    % --- Confirmation Logic ---
    if ~input_changed && valid_parse % If no changes were made and input was valid
        disp(' ');
        disp('--- "OK" clicked without changes. Saving current HSV thresholds and exiting. ---');
        final_h_min = h_min; % Save current active values
        final_h_max = h_max;
        final_s_min = s_min;
        final_s_max = s_max;
        final_v_min = v_min;
        final_v_max = v_max;
        break; % Exit the loop
    end

    % Proceed with validation and update if input_changed OR if it's the first interaction
    if valid_parse && input_changed % Only update if parse was successful AND changes were made
        % Validate ranges for HSV (H:0-180, S/V:0-255) and min <= max for each channel
        if all(new_values >= 0) && new_values(1) <= 180 && new_values(2) <= 180 && ... % Hue (0-180)
           new_values(3) <= 255 && new_values(4) <= 255 && ... % Saturation (0-255)
           new_values(5) <= 255 && new_values(6) <= 255 && ... % Value (0-255)
           new_values(1) <= new_values(2) && ... % H_min <= H_max
           new_values(3) <= new_values(4) && ... % S_min <= S_max
           new_values(5) <= new_values(6)        % V_min <= V_max
            
            h_min = new_values(1);
            h_max = new_values(2);
            s_min = new_values(3);
            s_max = new_values(4);
            v_min = new_values(5);
            v_max = new_values(6);
            
            disp('  HSV values updated successfully! See the segmented image for changes. The dialog will re-appear.');
        else
            disp(' '); % Add a newline for better readability
            warning('Invalid input: Please ensure values are within valid HSV ranges (H:0-180, S/V:0-255) and min <= max for each channel. The dialog will re-appear.');
            disp('  Example valid range: H:[0 180] S:[0 255] V:[0 255]');
        end
    elseif ~valid_parse % If parse failed (e.g., non-numeric input)
        disp(' ');
        warning('Invalid input format. Please ensure all 6 fields contain valid numbers. The dialog will re-appear.');
    end
    % If input_changed is false and valid_parse is true, it means user clicked OK without changes,
    % which is handled by the "Save and Exit" block above. So no 'else' needed here.

end % End of while true loop

% Close the figure after exiting the loop
if ishandle(hFig)
    close(hFig);
end

end