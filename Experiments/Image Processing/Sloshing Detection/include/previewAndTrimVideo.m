% function [final_startTime_L, final_endTime_L, final_startTime_R, final_endTime_R] = previewAndTrimVideo(videoObj_L, initial_startTime_L, videoObj_R, initial_startTime_R)
% % previewAndTrimVideo: Interactively previews one synchronized video and
% %   allows the user to define precise start/end times for motion for both videos.
% %
% %   Inputs:
% %     videoObj_L          - VideoReader object for the left camera (already set to its initial synchronized time).
% %     initial_startTime_L - The initial synchronized start time (in seconds) for the left video.
% %     videoObj_R          - VideoReader object for the right camera (already set to its initial synchronized time).
% %     initial_startTime_R - The initial synchronized start time (in seconds) for the right video.
% %
% %   Outputs:
% %     final_startTime_L - User-defined start time (in seconds) for motion in left video.
% %     final_endTime_L   - User-defined end time (in seconds) for motion in left video.
% %     final_startTime_R - User-defined start time (in seconds) for motion in right video.
% %     final_endTime_R   - User-defined end time (in seconds) for motion in right video.
% 
%     final_startTime_L = [];
%     final_endTime_L = [];
%     final_startTime_R = [];
%     final_endTime_R = [];
% 
%     % Calculate effective end times for the segments to be previewed
%     % These are the actual total durations from the initial synchronized start point
%     effective_endTime_L = videoObj_L.Duration; % This is the video's total duration
%     effective_endTime_R = videoObj_R.Duration;
% 
%     % We'll preview the LEFT video. Ensure its CurrentTime is set to its initial synchronized start.
%     videoObj_L.CurrentTime = initial_startTime_L;
% 
%     % Set up figure for video display
%     hFig = figure('Name', 'Video Preview (Press any key to pause, then Enter to continue)', 'NumberTitle', 'off', 'KeyPressFcn', @(src,event)setappdata(src,'stopPlayback',true));
%     hAx = axes('Parent', hFig, 'Visible', 'off'); % Use axes directly for cleaner display
% 
%     disp(' ');
%     disp('--- Video Preview for Motion Trimming ---');
%     disp('  Observe the motion in the video (Left Camera view).');
%     disp('  * Press **ANY key** on the video window to pause playback.');
%     disp('  * Once paused, press **Enter** in the Command Window to input times.');
%     disp('  * Type "q" and press Enter to quit without defining motion times.');
%     disp('----------------------------------------------------');
% 
%     playing = true;
%     while playing
%         % Reset to start for replay (important if user wants to watch again)
%         videoObj_L.CurrentTime = initial_startTime_L;
%         setappdata(hFig, 'stopPlayback', false); % Reset stop flag
% 
%         while hasFrame(videoObj_L) && videoObj_L.CurrentTime < effective_endTime_L
%             frame = readFrame(videoObj_L);
%             imshow(frame, 'Parent', hAx);
%             title(hAx, sprintf('Left Video (Time: %.2f s / %.2f s)', videoObj_L.CurrentTime, effective_endTime_L));
%             drawnow limitrate; % Update display
% 
%             % Check if user pressed a key to pause
%             if getappdata(hFig, 'stopPlayback')
%                 disp('Playback paused.');
%                 fprintf('Current Time (Left): %.2f s\n', videoObj_L.CurrentTime);
%                 playing = false; % Stop the inner playback loop
%                 break; % Exit inner loop
%             end
%         end
% 
%         if playing % If playback finished without user interruption
%             disp('Video segment finished playing.');
%             playing = false; % Exit outer loop
%         end
%     end
% 
%     % Get user input for motion times (relative to the previewed video's timeline)
%     current_time_at_pause = videoObj_L.CurrentTime; % The time when the video was paused or finished
% 
%     promptMsg_start = sprintf('Enter motion START time (in seconds, recommended: %.2f s to %.2f s): ', initial_startTime_L, effective_endTime_L);
%     input_start = input(promptMsg_start, 's');
% 
%     if strcmpi(input_start, 'q')
%         disp('Skipping motion time input. Exiting without defining new boundaries.');
%         close(hFig);
%         return; % Exit function without returning values
%     end
% 
%     input_start_val = str2double(input_start);
%     if isnan(input_start_val) || input_start_val < initial_startTime_L || input_start_val >= effective_endTime_L
%         warning('Invalid input for start time. Using initial synchronized start time (%.2f s).', initial_startTime_L);
%         motion_start_L_sec = initial_startTime_L;
%     else
%         motion_start_L_sec = input_start_val;
%     end
% 
%     promptMsg_end = sprintf('Enter motion END time (in seconds, must be > %.2f s and <= %.2f s): ', motion_start_L_sec, effective_endTime_L);
%     input_end = input(promptMsg_end, 's');
% 
%     input_end_val = str2double(input_end);
%     if isnan(input_end_val) || input_end_val <= motion_start_L_sec || input_end_val > effective_endTime_L
%         warning('Invalid input for end time. Using effective end time (%.2f s).', effective_endTime_L);
%         motion_end_L_sec = effective_endTime_L;
%     else
%         motion_end_L_sec = input_end_val;
%     end
% 
%     % --- Calculate corresponding times for the Right video ---
%     % The 'tau' relationship: If tau > 0, L starts 'tau' seconds before R.
%     % So, L_time = R_time + tau, or R_time = L_time - tau.
%     % If tau < 0, R starts 'abs(tau)' seconds before L.
%     % So, R_time = L_time + abs(tau), or R_time = L_time - (-abs(tau)) = L_time + tau.
%     % In both cases, the offset between CurrentTime in L and R is simply 'tau'.
%     % Relative_offset_from_L_to_R = (initial_startTime_R - initial_startTime_L)
%     % This offset should be equal to -tau if tau is defined as L_time - R_time.
% 
%     % Let's use the initial_startTime_L and initial_startTime_R to establish the fixed offset.
%     % offset_L_to_R = initial_startTime_L - initial_startTime_R;
%     % The actual offset is just initial_startTime_R - initial_startTime_L
% 
%     % The duration of the selected motion is the same for both videos.
%     motion_duration = motion_end_L_sec - motion_start_L_sec;
% 
%     % The critical part: how the motion start/end in the Left video translates to the Right video.
%     % Since videoObj_L.CurrentTime = initial_startTime_L, and videoObj_R.CurrentTime = initial_startTime_R,
%     % the *relative* time from the beginning of the "synchronized segment" is the same for both.
% 
%     % So, the offset from the beginning of the previewed segment is:
%     relative_motion_start_from_preview_start = motion_start_L_sec - initial_startTime_L;
% 
%     % Apply this relative offset to the Right video's initial synchronized start time
%     motion_start_R_sec = initial_startTime_R + relative_motion_start_from_preview_start;
%     motion_end_R_sec = motion_start_R_sec + motion_duration;
% 
%     % Ensure the calculated times are within the original video bounds (just in case of user error)
%     final_startTime_L = max(initial_startTime_L, motion_start_L_sec);
%     final_endTime_L = min(effective_endTime_L, motion_end_L_sec);
% 
%     final_startTime_R = max(initial_startTime_R, motion_start_R_sec);
%     final_endTime_R = min(effective_endTime_R, motion_end_R_sec);
% 
%     disp(' ');
%     disp('--- Final Trimmed Video Segment Times ---');
%     fprintf('  Left Video : From %.2f s to %.2f s\n', final_startTime_L, final_endTime_L);
%     fprintf('  Right Video: From %.2f s to %.2f s\n', final_startTime_R, final_endTime_R);
%     disp('------------------------------------------');
% 
%     close(hFig); % Close the figure after input
% end
% 
% 
function [final_startTime_L, final_endTime_L, final_startTime_R, final_endTime_R] = previewAndTrimVideo(videoObj_L, initial_startTime_L, videoObj_R, initial_startTime_R)
% previewAndTrimVideo: Interactively previews one synchronized video and
%   allows the user to define precise start/end times for motion for both videos.
%
%   Inputs:
%     videoObj_L          - VideoReader object for the left camera (already set to its initial synchronized time).
%     initial_startTime_L - The initial synchronized start time (in seconds) for the left video.
%     videoObj_R          - VideoReader object for the right camera (already set to its initial synchronized time).
%     initial_startTime_R - The initial synchronized start time (in seconds) for the right video.
%
%   Outputs:
%     final_startTime_L - User-defined start time (in seconds) for motion in left video.
%     final_endTime_L   - User-defined end time (in seconds) for motion in left video.
%     final_startTime_R - User-defined start time (in seconds) for motion in right video.
%     final_endTime_R   - User-defined end time (in seconds) for right video.


    final_startTime_L = [];
    final_endTime_L = [];
    final_startTime_R = [];
    final_endTime_R = [];

    % Calculate effective end times for the segments to be previewed
    effective_endTime_L = videoObj_L.Duration;
    effective_endTime_R = videoObj_R.Duration;

    % Set up figure for video display
    hFig = figure('Name', 'Video Preview for Trimming', 'NumberTitle', 'off', 'KeyPressFcn', @(src,event)setappdata(src,'stopPlayback',true));
    hAx = axes('Parent', hFig, 'Visible', 'off');

    % Flag to control the main loop: continue_loop = true means keep showing video and asking for input
    continue_loop = true;

    while continue_loop % This loop controls replaying the video and re-asking for input
        disp(' ');
        disp('================================================================');
        disp('         --- Interactive Video Trimming Guide ---');
        disp('================================================================');
        disp(' ');
        disp('  **Step 1: Observe the Video Motion.**');
        disp('  The Left Camera video will now play, starting from its synchronized point.');
        disp('  Watch the liquid behavior to identify the exact start and end of the relevant motion.');
        disp(' ');
        disp('  **To Pause Playback:**');
        disp('  * Click on the video window and press **ANY keyboard key** (e.g., Spacebar).');
        disp('  * The video will pause, and its current time will be displayed in the Command Window.');
        disp(' ');
        disp('  **If you need to re-watch the video or correct input:**');
        disp('  * In the input dialog that appears, click the **"Cancel"** button. The video will replay.');
        disp('  * Or, if the dialog is already closed, run the function again.');
        disp(' ');
        disp('  **Once ready to define times:**');
        disp('  * After pausing, enter the times in the input dialog and click **"OK"** to proceed.');
        disp('================================================================');
        disp(' ');

        % --- Video Playback Loop ---
        videoObj_L.CurrentTime = initial_startTime_L; % Always reset for replay
        setappdata(hFig, 'stopPlayback', false); % Reset stop flag for new playback

        playback_finished_naturally = true;
        while hasFrame(videoObj_L) && videoObj_L.CurrentTime < effective_endTime_L
            frame = readFrame(videoObj_L);
            imshow(frame, 'Parent', hAx);
            title(hAx, sprintf('Left Video (Current Time: %.2f s / Total Synced Duration: %.2f s)', videoObj_L.CurrentTime, effective_endTime_L));
            drawnow limitrate; % Update display

            % Check if user pressed a key to pause
            if getappdata(hFig, 'stopPlayback')
                disp(' ');
                disp('----------------------------------------------------');
                disp('  Playback paused by user.');
                fprintf('  Current Time (Left Camera): **%.2f s**\n', videoObj_L.CurrentTime);
                disp('  Consider this time as a reference for your start/end points.');
                disp('----------------------------------------------------');
                playback_finished_naturally = false; % User interrupted
                break; % Exit inner playback loop
            end
        end

        if playback_finished_naturally
            disp(' ');
            disp('----------------------------------------------------');
            disp('  Video segment finished playing.');
            fprintf('  Final Time (Left Camera): **%.2f s**\n', videoObj_L.CurrentTime);
            disp('----------------------------------------------------');
        end

        % --- Get user input for motion times using inputdlg ---
        disp(' ');
        disp('================================================================');
        disp('  **Step 2: Input Your Desired Motion Start and End Times.**');
        disp('  A dialog box will now appear. Please enter the precise start and end');
        disp('  times (in seconds) for the liquid motion you want to analyze.');
        disp('  These times are relative to the *synchronized start* of the video.');
        fprintf('  The video plays from %.2f s to %.2f s.\n', initial_startTime_L, effective_endTime_L);
        disp(' ');
        disp('  Remember: Click "Cancel" in the dialog to replay the video.');
        disp('================================================================');
        disp(' ');

        prompt = {sprintf('Enter Motion START Time (seconds, e.g., %.2f):', initial_startTime_L), ...
                  sprintf('Enter Motion END Time (seconds, e.2f):', effective_endTime_L)};
        dlgtitle = 'Define Liquid Motion Segment Times';
        dims = [1 60];
        % Default values can be the initial_startTime and effective_endTime for convenience
        definput = {sprintf('%.2f', initial_startTime_L), sprintf('%.2f', effective_endTime_L)};

        answer = inputdlg(prompt, dlgtitle, dims, definput);

        % Handle input dialog outcome
        if isempty(answer) % User clicked 'Cancel'
            disp(' ');
            disp('--- Input cancelled. Replaying video for re-evaluation. ---');
            % continue_loop remains true, so the outer while loop will iterate again
            % This will reset CurrentTime and replay the video
        else % User clicked 'OK'
            input_start_val = str2double(answer{1});
            input_end_val = str2double(answer{2});

            % Validate and assign start time
            if isnan(input_start_val) || input_start_val < initial_startTime_L || input_start_val >= effective_endTime_L
                warning('Invalid input for start time. Using initial synchronized start time (%.2f s).', initial_startTime_L);
                motion_start_L_sec = initial_startTime_L;
            else
                motion_start_L_sec = input_start_val;
            end

            % Validate and assign end time
            if isnan(input_end_val) || input_end_val <= motion_start_L_sec || input_end_val > effective_endTime_L
                warning('Invalid input for end time. Using effective end time (%.2f s).', effective_endTime_L);
                motion_end_L_sec = effective_endTime_L;
            else
                motion_end_L_sec = input_end_val;
            end

            % --- Calculate corresponding times for the Right video ---
            motion_duration = motion_end_L_sec - motion_start_L_sec;
            relative_motion_start_from_preview_start = motion_start_L_sec - initial_startTime_L;

            motion_start_R_sec = initial_startTime_R + relative_motion_start_from_preview_start;
            motion_end_R_sec = motion_start_R_sec + motion_duration;

            % Ensure the calculated times are within the original video bounds (just in case of user error)
            final_startTime_L = max(initial_startTime_L, motion_start_L_sec);
            final_endTime_L = min(effective_endTime_L, motion_end_L_sec);

            final_startTime_R = max(initial_startTime_R, motion_start_R_sec);
            final_endTime_R = min(effective_endTime_R, motion_end_R_sec);

            disp(' ');
            disp('================================================================');
            disp('        --- Trimmed Video Segment Times Confirmed ---');
            disp('================================================================');
            fprintf('  Left Video : From %.2f s to %.2f s\n', final_startTime_L, final_endTime_L);
            fprintf('  Right Video: From %.2f s to %.2f s\n', final_startTime_R, final_endTime_R);
            disp('  These times will now be used for further processing.');
            disp('================================================================');
            disp(' ');

            continue_loop = false; % Exit the main loop as valid input was received
        end
    end % End of while continue_loop

    % Close the figure only after successful input and exiting the loop
    if ishandle(hFig)
        close(hFig);
    end
end