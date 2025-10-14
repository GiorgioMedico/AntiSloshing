function [startTime, endTime] = removeFrames(videoObj, startTime, numFramesToRemoveStart, numFramesToRemoveEnd)
%removeFrames Adjusts start and end times of a video by removing frames from the start and end.
%
%   [startTime, endTime] = removeFrames(videoObj, startTime, numFramesToRemoveStart, numFramesToRemoveEnd)
%
%   INPUTS:
%       videoObj               - VideoReader object representing the video.
%       startTime              - Original start time of the video
%       numFramesToRemoveStart - Number of frames to remove from the start.
%       numFramesToRemoveEnd   - Number of frames to remove from the end.
%
%   OUTPUTS:
%       startTime - Adjusted start time (in seconds) after removing frames from the start.
%       endTime   - Adjusted end time (in seconds) after removing frames from the end.
%
%   DESCRIPTION:
%       This function removes a specified number of frames from the beginning and end
%       of a video by adjusting the start and end times. It ensures the end time does
%       not exceed the video duration.
%
%   EXAMPLE:
%       [startTime, endTime] = removeFrames(videoObj, 100, 50);
%
%   Author: Simone Soprani
%   Date: May 2025

    % Compute time to remove
    startTimeAdjustment = numFramesToRemoveStart / videoObj.FrameRate;
    endTimeAdjustment   = numFramesToRemoveEnd / videoObj.FrameRate;

    % Adjust start and end times
    startTime = startTime + startTimeAdjustment;
    endTime   = min(videoObj.Duration - endTimeAdjustment, videoObj.Duration);

end
