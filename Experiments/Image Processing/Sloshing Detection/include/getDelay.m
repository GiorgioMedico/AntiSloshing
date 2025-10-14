function tau = getDelay(left_video_path,right_video_path)
%getDelay Estimate the audio delay between two video files.
%
%   tau = getDelay(left_video_path, right_video_path) estimates the average
%   audio delay (in seconds) between two stereo audio signals extracted from
%   video files. It uses cross-correlation of each stereo channel to find the
%   lag that maximizes similarity, then averages the delay of both channels.
%
%   INPUTS:
%       left_video_path  - Path to the first video file (left).
%       right_video_path - Path to the second video file (right).
%
%   OUTPUT:
%       tau              - Estimated audio delay (in seconds) between the two videos.
%
%   DESCRIPTION:
%   1) Reads the stereo audio signals from the left and right video files.
%   2) Displays the waveforms of each audio track for visual inspection.
%   3) Calculates the cross-correlation of each channel (left and right).
%   4) Finds the lag corresponding to the maximum correlation for each channel.
%   5) Converts these lag values to time delays (assuming a GoPro audio sampling rate of 48 kHz).
%   6) Averages the delay estimates from both stereo channels to produce tau.

    rate = 48*1000; % GoPros record audio at a fixed sample rate of 48 kHz
    [L_audio, fs_L]=audioread(left_video_path, [1,inf]);
    subplot(2,1,1)
    plot(L_audio);
    [R_audio, fs_R]=audioread(right_video_path, [1,inf]);
    subplot(2,1,2)
    plot(R_audio);
    
    [r1, lags1]=xcorr(L_audio(:,1), R_audio(:,1));
    [~,delay1]=max(r1);
    tau1 = lags1(delay1)/rate;
    [r2, lags2]=xcorr(L_audio(:,2), R_audio(:,2));
    [~,delay2]=max(r2);
    tau2 = lags2(delay2)/rate;
    
    tau = (tau1 + tau2)/2
end

