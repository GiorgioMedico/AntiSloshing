function [final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max] = callInteractiveHsvForDiameter(initialFrame)
%callInteractiveHsvForDiameter Calls interactiveHsvSegmentation for diameter detection.
%   This function serves as a wrapper to `interactiveHsvSegmentation`.
%   Before starting the interactive segmentation, it displays a pop-up
%   message to advise the user about the purpose of the segmentation
%   (diameter detection) and the potential need for aggressive values.
%
%   Input:
%     initialFrame - The initial RGB image to segment, passed directly
%                    to interactiveHsvSegmentation.
%
%   Output:
%     final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max
%     The HSV threshold values returned by interactiveHsvSegmentation.

% --- Step 1: Display the advisory pop-up window ---
msgbox_title = 'Segmentation for Diameter Detection';
msgbox_message = {
    'Attention: The upcoming interactive HSV segmentation is specifically for',
    'detecting the liquid''s diameter.',
    '',
    'This often requires more aggressive or precise HSV threshold values',
    'to isolate the main body and boundaries of the liquid effectively.',
    '',
    'Please adjust the values carefully to achieve a clear and accurate',
    'segmentation of the liquid boundary for diameter calculation.'
};

% Use 'modal' to make sure the user sees and acknowledges the message
% before the interactive segmentation tool opens.
mb = msgbox(msgbox_message, msgbox_title, 'help', 'modal');

% Wait for the user to click 'OK' on the message box.
uiwait(mb);

% --- Step 2: Call the interactiveHsvSegmentation function ---
% Pass the initialFrame directly to it and capture all its return values.
[final_h_min, final_h_max, final_s_min, final_s_max, final_v_min, final_v_max] = interactiveHsvSegmentation(initialFrame);

end