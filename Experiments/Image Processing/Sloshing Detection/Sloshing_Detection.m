%% Sloshing Detection - Image Processing
%--------------------------------------------------------------------------
% Date:            2025-05
% Authors:         Simone Soprani, Roberto Di Leva, Daniele Ceccarelli
%
% Description:     
%   Image processing and computer vision script to detect the sloshing height
%   from videos captured by two cameras. Processes synchronized video streams,
%   analyzes motion, and extracts key sloshing parameters.
%
% Related Publication:
%   [Paper Title]
%   Authors: Roberto Di Leva, Simone Soprani, Marco Carricato, 
%            Gianluca Palli and Luigi Biagiotti 
%   Published in: [Journal/Conference], [Year]
%   Repository link: [GitHub or DOI URL]
%
% Usage:
%   Run this script to process videos and extract sloshing height data.
%
%--------------------------------------------------------------------------

clc 
close all
clear

video= "RD_3D_0.3m_6.3s_720deg";


load("GoProRightParams_VideoMode.mat");
% load(fullfile("Aruco","Poses","poses_aruco_1010_1.mat"));
load(fullfile("Aruco","Poses","poses_aruco_1.mat"));
addpath("include");

%% Options

record_synchro = false; % Set true to save the synchronized videos
record_BW = false; % Set true to save the segmented videos
inRange = true; % Set true to select color range
downsampling = 1; % Set a value to remove number of frames considered
inRange_open = 1; % Set true to select color range and apply opening operation
auto_remove_frames = 1; % Set true to preview video and remove frames at start and end 
manual_remove_frames = 0; % Set true to manually remove frames

%% Debug 
show_hsv = 1; 
show_edge = 1; % Set true to visualize the detected edge during execution
show_mask = 1; % Set true to visualize the detected mask during execution
show_reprojection = 1; % Set true to visualize the 3D peak reprojection during execution

save_fig = 1; % Set true to save the figures
save_mat = 1; % Set true to save the mat files

%% Initial Parameters

D = 100; % size of the diameter of the container in [mm]
h0 = 75; % height of the liquid at rest in [mm]

%% Video name and paths
out_traj_name = strcat(video,'.mat');

% left_video_path = fullfile("GoPro",date,"GoPro_Left",strcat(video,".MP4"));
% right_video_path = fullfile("GoPro",date,"GoPro_Right",strcat(video,".MP4"));
left_video_path = fullfile("GoPro","GoPro_Left",strcat(video,".MP4"));
right_video_path = fullfile("GoPro","GoPro_Right",strcat(video,".MP4"));

if ~exist(fullfile('Data','mat','full_slosh'), 'dir')
    mkdir(fullfile('Data','mat','full_slosh'));
end

if ~exist(fullfile('Data','mat','etas_pixels'), 'dir')
    mkdir(fullfile('Data','mat','etas_pixels'));
end

save_full_exp_path = fullfile('Data','mat','full_slosh',out_traj_name);
save_etas_exp_path = fullfile('Data','mat','etas_pixels',out_traj_name);


%% Open video object

if record_BW == true
    bw_colormap = colormap(gray(2));
    if inRange
        v_L_inRange = VideoWriter(strcat(video,'_inRange_Cropped_L'), 'Motion JPEG AVI');
        v_R_inRange = VideoWriter(strcat(video,'_inRange_Cropped_R'), 'Motion JPEG AVI');
        open(v_L_inRange);
        open(v_R_inRange);
        if inRange_open > 0
            v_L_inRange_open = VideoWriter(strcat(video,'_inRange_Cropped_open3_L'), 'Motion JPEG AVI');
            v_R_inRange_open = VideoWriter(strcat(video,'_inRange_Cropped_open3_R'), 'Motion JPEG AVI');
            open(v_L_inRange_open);
            open(v_R_inRange_open);
        end
    else
        v_L_BW = VideoWriter(strcat(video,'BW_Left_Cropped'), 'Motion JPEG AVI');
        v_R_BW = VideoWriter(strcat(video,'BW_Right_Cropped'), 'Motion JPEG AVI');
        v_L_closed = VideoWriter(strcat(video,'Closed_Left_Cropped'), 'Motion JPEG AVI');
        v_R_closed = VideoWriter(strcat(video,'Closed_Right_Cropped'), 'Motion JPEG AVI');
        v_L_opened = VideoWriter(strcat(video,'Opened_Left_Cropped'), 'Motion JPEG AVI');
        v_R_opened = VideoWriter(strcat(video,'Opened_Right_Cropped'), 'Motion JPEG AVI');
        open(v_L_BW);
        open(v_R_BW);
        open(v_L_closed);
        open(v_R_closed);
        open(v_L_opened);
        open(v_R_opened);
    end
end

%% Audio Sync
% To synchronise the two videos we find the delay between them by finding
% the lag that maximizes the cross-correlation between the two audio
% signals of the videos
tau = getDelay(left_video_path,right_video_path);

% Used only to record the synchronized videos
if record_synchro==true
    v_L = VideoWriter(strcat("Video Cut\", video, "_L_Cut_Full.avi"), 'Motion JPEG AVI');
    v_R = VideoWriter(strcat("Video Cut\", video, "_R_Cut_Full.avi"), 'Motion JPEG AVI');
    open(v_L);
    open(v_R);
end

% If tau is positive, the left video starts early wrt the right one
% If tau is negative, the right video starts early wrt the left one
if tau>0
    videoObj_L = VideoReader(left_video_path, 'CurrentTime', tau);
    videoObj_R = VideoReader(right_video_path);
    startTime_L = tau;  
    startTime_R = 0; 
else
    videoObj_L = VideoReader(left_video_path);
    videoObj_R = VideoReader(right_video_path, 'CurrentTime', abs(tau));
    startTime_L = 0;  
    startTime_R = abs(tau); 
end

%% Shorten videos

if auto_remove_frames

    % Call the function to preview and get the desired motion start/end times
    [startTime_L, endTime_L, startTime_R, endTime_R] = ...
        previewAndTrimVideo(videoObj_L, startTime_L, videoObj_R, startTime_R);
    
    % --- precise start and end times for the relevant motion ---
    if ~isempty(startTime_L) % Check if the user didn't cancel the input
        fprintf('\nAnalysis will focus on:\n');
        fprintf('  Left Video : From %.2f s to %.2f s\n', startTime_L, endTime_L);
        fprintf('  Right Video: From %.2f s to %.2f s\n', startTime_R, endTime_R);
    
        % IMPORTANT: Resetting CurrentTime for subsequent processing
        videoObj_L.CurrentTime = startTime_L;
        videoObj_R.CurrentTime = startTime_R;
    
    else
        disp('Video trimming was skipped or cancelled. Proceeding with full synchronized videos.');
    end
    
elseif manual_remove_frames   
    %% Cut number of frames to analyze
    
    % Get total frame counts
    totalFrames_L = floor(videoObj_L.Duration * videoObj_L.FrameRate);
    totalFrames_R = floor(videoObj_R.Duration * videoObj_R.FrameRate);
    
    % Define the number of frames to remove
    n = 300;  % Remove first n frames % frames = s*60fps
    m = 180;  % Remove last m frames
    
    [startTime_L, endTime_L] = removeFrames(videoObj_L, startTime_L, n, m);
    [startTime_R, endTime_R] = removeFrames(videoObj_R, startTime_R, n, m);
    
    % Reinitialize video objects with the new trimmed start times
    videoObj_L = VideoReader(left_video_path, 'CurrentTime', startTime_L);
    videoObj_R = VideoReader(right_video_path, 'CurrentTime', startTime_R);

end
%% Range selection for segmentation
% Capture the first frame to check the diameter pixels and then start
% elaborating the rest of the video considering only the pixels of the
% container

frame_L = readFrame(videoObj_L);
frame_R = readFrame(videoObj_R);

if inRange
    down = downsampling; 
    frame_L = frame_L(1:down:end,1:down:end,:);
    frame_R = frame_R(1:down:end,1:down:end,:);
    if show_hsv 
        [h_min, h_max, s_min, s_max, v_min, v_max] = interactiveHsvSegmentation(frame_R);
    else
        h_min = 88;
        % h_max = 119;
        h_max = 120;
        s_min = 50;
        % s_min = 80;
        % s_min = 120;
        s_max = 255;
        v_min = 0;
        v_max = 255;
    end
    minval = [h_min/180 s_min/255 v_min/255]; %// Define three element vector here for each colour plane
    maxval = [h_max/180 s_max/255 v_max/255]; %// Define three element vector here for each colour plane
    if inRange_open > 0
        se_open = strel('disk',inRange_open);
    end
else
    se_close_L = strel('disk', 1);
    se_close_L2 = strel('disk', 2);
    se_open_R = strel('disk', 2);
    se_close_R = strel('disk', 1);
    se_close_R2 = strel('disk', 2);
end

%% Check inRange values
if show_hsv
    [h_min_d, h_max_d, s_min_d, s_max_d, v_min_d, v_max_d] = callInteractiveHsvForDiameter(frame_R);
end

%% First frame diameter cropping

[d_ind1_L, d_ind2_L] = findDiameter(frame_L, D, h0, inRange, [h_min_d, h_max_d, s_min_d, s_max_d, v_min_d, v_max_d])
[d_ind1_R, d_ind2_R] = findDiameter(frame_R, D, h0, inRange, [h_min_d, h_max_d, s_min_d, s_max_d, v_min_d, v_max_d])

DpxR = d_ind2_R-d_ind1_R;
DpxL = d_ind1_L- d_ind2_L;

%% Frame iterations
% Define output folders, save single frame images
folder = fullfile('Data','Frames',video);

left_folder = fullfile(folder,'Left');
right_folder = fullfile(folder,'Right');

% Create folders if they do not exist
if ~exist(left_folder, 'dir')
    mkdir(left_folder);
end
if ~exist(right_folder, 'dir')
    mkdir(right_folder);
end

disp("Starting iterations")
tic
k=0;

while(hasFrame(videoObj_L) && hasFrame(videoObj_R) && ...
      videoObj_L.CurrentTime < endTime_L && videoObj_R.CurrentTime < endTime_R)
    k=k+1;

    %% Image Processing
    % read current frame, VideoReader will update pointer to next frame
    if(k~=0)
        frame_L = readFrame(videoObj_L);
        frame_R = readFrame(videoObj_R);

        if save_fig
            % Define file names
            left_filename = fullfile(left_folder, sprintf('%d.jpg', k));
            right_filename = fullfile(right_folder, sprintf('%d.jpg', k));
            % Save frames as JPEG images
            imwrite(frame_L, left_filename);
            imwrite(frame_R, right_filename);
        end
    end
    
    % if range is specified
    if inRange

        % get frame (downsampled if down>1 (take less pixels), all colors)
        frame_L = frame_L(1:down:end,1:down:end,:);
        frame_R = frame_R(1:down:end,1:down:end,:);
        % resize frame by taking only columns between the d indeces,
        % horizontal shrinking
        frame_L = frame_L(:,d_ind1_L:d_ind2_L,:);
        frame_R = frame_R(:,d_ind1_R:d_ind2_R,:);
        % convert to hsv color space
        hsv_frame_L = rgb2hsv(frame_L);
        hsv_frame_R = rgb2hsv(frame_R);
        % initialize all black image of the size of the frame
        segmented_L_noblob = true(size(hsv_frame_L,1), size(hsv_frame_L,2));
        segmented_R_noblob = true(size(hsv_frame_R,1), size(hsv_frame_R,2));
        % for each one of the triplet (hsv) take logical AND between all
        % black image pixels and the value (true/false) of the comparison
        % between colored pixels and the inRange values
        % it colors white the pixels which are outside the range
        for ii = 1:3
            segmented_L_noblob = segmented_L_noblob & (hsv_frame_L(:,:,ii) >= minval(ii) & hsv_frame_L(:,:,ii) <= maxval(ii)); 
            segmented_R_noblob = segmented_R_noblob & (hsv_frame_R(:,:,ii) >= minval(ii) & hsv_frame_R(:,:,ii) <= maxval(ii)); 
        end
        
        % extract all connected components (in order to eliminate spurious
        % blobs)
        % 1 is the number of objects to include
        segmented_L = bwareafilt(segmented_L_noblob,1);
        segmented_R = bwareafilt(segmented_R_noblob,1);
        % flip the pixels color
        segmented_L = ~segmented_L;
        segmented_R = ~segmented_R;
        % perform open if necessary (to increase white, fill holes)
        if inRange_open>0
            segmented_L_open = imopen(segmented_L, se_open);
            segmented_L = segmented_L_open;
            segmented_R_open = imopen(segmented_R, se_open);
            segmented_R = segmented_R_open;
        end
    % if range is not specified take red channel
    else
        red_L = frame_L(:,d_ind1_L:d_ind2_L,1); 
        red_R = frame_R(:,d_ind1_R:d_ind2_R,1);
        level_L = 0.30;
        BW_L = imbinarize(red_L, level_L);
        BW_R = imbinarize(red_R);
        closed_L= imclose(BW_L, se_close_L);
        closed_R= imclose(BW_R, se_close_R);
        opened_L = imopen(closed_L,se_open_L);
        opened_R = imopen(closed_R,se_open_R);
        segmented_L = imclose(opened_L, se_close_L2);
        segmented_R = imclose(opened_R, se_close_R2);
    end

    %% Extract surface and highest point for both cameras
    % iterate columns between the diameter indices (cropped frame)

    %% left video
    % apply Canny edge detector 
    edges_L = edge(segmented_L, 'Canny');  
    for j = 1:(d_ind2_L-d_ind1_L)
        edge_rows = find(edges_L(:, j));

        % surface pixels array (height_L = surface of the liquid)
        % height_L: index = column, value = row
        if isempty(edge_rows)
            height_L(j) = size(segmented_L,1);
        else
            height_L(j) = min(edge_rows);  % Take the highest (smallest row index)
        end
    end
    
    
    % find highest point in the liquid surface
    % find lowest value of height (row), return index (col(i)) and value
    % (row(i))
    [maxheight_L, ind_maxheight_L] = min(height_L);
    % ind_maxheight_L (u) = column of the cropped frame which has highest peak
    % maxheight_L (v) = row of the cropped frame which has highest peak
    % => (u,v) coordinates of peak in cropped frame image

    % take index and height in non cropped frame
    eta_index_L(k)=ind_maxheight_L+d_ind1_L; %u
    eta_L(k)=maxheight_L; %v
    % => (u,v) coordinates of peak in complete frame image

    % Display detected point and surface, and mask overlay
    if mod(k,15)==0

        if show_edge
            figure(987)
            hold on
            title(['Left Camera - Frame ', num2str(k)]);
            plot(height_L)
            plot(ind_maxheight_L,maxheight_L,'Marker','o','Color','r')
        end

        if show_mask
            figure(12345678)
            hold on
            imshow(frame_L);
            % imshow(~segmented_L);
            h = imshow(cat(3, ones(size(segmented_L)), zeros(size(segmented_L)), zeros(size(segmented_L)))); % Red overlay
            set(h, 'AlphaData', double(~segmented_L) * 0.5); % Only white pixels visible at 50% opacity
            hold off
            drawnow
        end

    end
    
    %% right video
    edges_R = edge(segmented_R, 'Canny');  
    for j = 1:(d_ind2_R-d_ind1_R)
        edge_rows = find(edges_R(:, j));

        if isempty(edge_rows)
            height_R(j) = size(segmented_R,1);
        else
            height_R(j) = min(edge_rows);  % Take the highest (smallest row index)
        end
    end
    
    % [row, column]
    [maxheight_R, ind_maxheight_R] = min(height_R);

    eta_index_R(k)=ind_maxheight_R+d_ind1_R;
    eta_R(k)=maxheight_R;

    if mod(k,15)==0

        if show_edge
            figure(988)
            hold on
            title(['Right Camera - Frame ', num2str(k)]);
            plot(height_R)
            plot(ind_maxheight_R,maxheight_R,'Marker','o','Color','r')
        end

        if show_mask
            figure(12345679)
            hold on
            imshow(frame_R);
            % imshow(~segmented_L);
            h = imshow(cat(3, ones(size(segmented_R)), zeros(size(segmented_R)), zeros(size(segmented_R)))); % Red overlay
            set(h, 'AlphaData', double(~segmented_R) * 0.5); % Only white pixels visible at 50% opacity
            hold off
            drawnow
        end
    end

    %% Video Processing
    % Overlay red circles at the positions of max height for both frames
    circle_radius = 10; % Adjust this value for the desired size of the circle
    circle_color = [255, 0, 0]; % Red color

    if record_BW == true
        % Add the circle for the left frame (v_L_inRange)
        segmented_L_rgb = repmat(uint8(segmented_L) * 255, [1 1 3]); % Convert BW to RGB
        segmented_L_rgb = insertShape(segmented_L_rgb, 'FilledCircle', [ind_maxheight_L, eta_L(k), circle_radius], 'Color', circle_color);
    
        % Add the circle for the right frame (v_R_inRange)
        segmented_R_rgb = repmat(uint8(segmented_R) * 255, [1 1 3]); % Convert BW to RGB
        segmented_R_rgb = insertShape(segmented_R_rgb, 'FilledCircle', [ind_maxheight_R, eta_R(k), circle_radius], 'Color', circle_color);
         
        if inRange
            % writeVideo(v_L_inRange, im2frame(uint8(segmented_L), bw_colormap));
            % writeVideo(v_R_inRange, im2frame(uint8(segmented_R), bw_colormap));
            writeVideo(v_L_inRange, im2frame(uint8(segmented_L_rgb)));
            writeVideo(v_R_inRange, im2frame(uint8(segmented_R_rgb)));
            if inRange_open
                writeVideo(v_L_inRange_open, im2frame(uint8(segmented_L_open), bw_colormap));
                writeVideo(v_R_inRange_open, im2frame(uint8(segmented_R_open), bw_colormap));
            end
        else
            writeVideo(v_L_BW,im2frame(uint8(BW_L), bw_colormap));
            writeVideo(v_R_BW,im2frame(uint8(BW_R), bw_colormap));
            writeVideo(v_L_closed,im2frame(uint8(segmented_L), bw_colormap));
            writeVideo(v_R_closed,im2frame(uint8(segmented_R), bw_colormap));
            writeVideo(v_L_opened,im2frame(uint8(opened_L), bw_colormap));
            writeVideo(v_R_opened,im2frame(uint8(opened_R), bw_colormap));
        end
    end
end
toc

%% close video files
if record_BW == true
    if inRange
        close(v_L_inRange);
        close(v_R_inRange);
        if inRange_open>0
            close(v_L_inRange_open);
            close(v_R_inRange_open);
        end
    else
        close(v_L_BW);
        close(v_R_BW);
        close(v_L_closed);
        close(v_R_closed);
        close(v_L_opened);
        close(v_R_opened);
    end
end

nframes = k; 

%% POST PROCESSING


%% Peaks (lowest row) in complete frame image 
% eta_index = u coordinate in image
% eta = v coordinate in image
peaks_L = [eta_index_L; eta_L]; %2 x n,
peaks_R = [eta_index_R; eta_R]; %2 x n

time_length = size(eta_index_R,2);
slosh_times = linspace(0, (time_length / 60), time_length);

figure()
plot(slosh_times,eta_L)
title("Left camera slosh height IN PIXELS")
xlabel("time")
ylabel("Pixels (with (0,0) being at top left))")
grid on

figure()
plot(slosh_times,eta_R)
title("Right camera slosh height IN PIXELS")
xlabel("time")
ylabel("Pixels (with (0,0) being at top left))")
grid on

%% Triangulation

% Undistort Pixel Coordinates
% Assuming GoProRightParams_VideoMode contains the camera intrinsics
peaks_L_ud = undistortPoints(peaks_L', GoProRightParams_VideoMode);
peaks_R_ud = undistortPoints(peaks_R', GoProRightParams_VideoMode);

% Define Camera Projection Matrices (P = K * [R | t])
% Intrinsics (K)
K = GoProRightParams_VideoMode.K;

% Extrinsics (Aruco to Camera Transform)
% T_a_to_L = inv(poses_L.A); % Aruco to Left Camera
% T_a_to_R = inv(poses_R.A); % Aruco to Right Camera
T_a_to_L = poses_L.A; % Aruco to Left Camera
T_a_to_R = poses_R.A; % Aruco to Right Camera

disp('poses_L.A (Camera to Aruco):');
disp(poses_L.A);

disp('T_a_to_L (Aruco to Camera):');
disp(T_a_to_L);

% Should be approximately the identity matrix
disp('Check Inversion (Aruco to Camera * Camera to Aruco):');
disp(T_a_to_L * poses_L.A);

% Decompose to Rotation (R) and Translation (t)
R_L = T_a_to_L(1:3, 1:3);
t_L = T_a_to_L(1:3, 4);

R_R = T_a_to_R(1:3, 1:3);
t_R = T_a_to_R(1:3, 4);

% Projection Matrices
P_L = K * [R_L, t_L];
P_R = K * [R_R, t_R];

disp('Translation Vector (Left Camera):');
disp(t_L);
disp('Translation Vector (Right Camera):');
disp(t_R);

% Triangulate Using MATLAB's triangulate Function
world_points = triangulate(peaks_L_ud, peaks_R_ud, P_L, P_R);
% Transform in hom coord
world_points_h = [world_points, ones(size(world_points, 1), 1)]'; 

% take first height
h0_point = world_points_h(2,1);

disp('Triangulated 3D Points in Aruco Frame:');
disp(world_points);


%% Display sloshing height 

figure
plot(world_points(:,2))
title("Unfiltered Sloshing Height")
ylabel("Height [mm]")
xlabel("Time [s]")

% Find the first peak in the height data
[maximum, ~] = max(world_points(:,2)-h0_point) % Find index of first peak

[peaks, peak_indices] = findpeaks(world_points(:,2), slosh_times);

start_ = 0;
end_ = slosh_times(end);
Ymax = max(world_points(:,2)-h0_point); 
Ymin = 0;

fig = figure
hold on
plot(slosh_times,abs(world_points(:,2)-h0_point), 'LineWidth', 2.5)
title("Sloshing Height",'FontSize', 18, 'FontWeight', 'bold')
ylabel("Height [mm]",'FontSize', 16, 'FontWeight', 'bold')
xlabel("Time [s]",'FontSize', 16, 'FontWeight', 'bold')
xlim([start_ end_]);
ylim([Ymin Ymax]);
set(gca, 'FontSize', 14, 'LineWidth', 2) 
grid on

% Set figure size (adjust values as needed)
fig.Units = 'inches';  
fig.Position = [0, 0, 6, 4]; % Example: 6x4 inches

% Adjust paper size to match figure size exactly
set(fig, 'PaperPositionMode', 'auto'); 
set(fig, 'PaperSize', fig.Position(3:4));

if save_fig
    % folder_name = fullfile('Data',date,'Plots');
    folder_name = fullfile('Data','Plots');
    if ~exist(folder_name, 'dir')
        mkdir(folder_name); % Create folder if it doesn't exist
    end
    plot_name = strcat(video,'.pdf');
    saveas(fig, fullfile(folder_name, plot_name));
end

if save_mat
    save(save_full_exp_path, 'slosh_times', 'world_points', 'h0_point');
    save(save_etas_exp_path, 'peaks_L','peaks_R');
end

%% REPROJECTION
% Check that the reprojected 3D points in the 2D cameras align with the
% expected results

% If tau is positive, the left video starts early wrt the right one
% If tau is negative, the right video starts early wrt the left one
if(tau>0)
    videoObj_L = VideoReader(left_video_path, 'CurrentTime', tau);
    videoObj_R = VideoReader(right_video_path);
else
    videoObj_L = VideoReader(left_video_path);
    videoObj_R = VideoReader(right_video_path, 'CurrentTime', abs(tau));
end
videoObj_L.CurrentTime = startTime_L;
videoObj_R.CurrentTime = startTime_R;

% Define the Aruco frame origin in homogeneous coordinates (in the Aruco frame)
aruco_origin = [0; 0; 0; 1]; % 4x1 vector
aruco_x = [15; 0; 0; 1];     % X-axis (scaled for visibility)
aruco_y = [0; 15; 0; 1];     % Y-axis
aruco_z = [0; 0; 15; 1];     % Z-axis

if show_reprojection
    k=0;
    while(hasFrame(videoObj_L) && hasFrame(videoObj_R) && ...
          videoObj_L.CurrentTime < endTime_L && videoObj_R.CurrentTime < endTime_R)
        k=k+1;
    
        % Step 1: Read frames from both videos
        if(k~=0)
            frame_L = readFrame(videoObj_L);
            frame_R = readFrame(videoObj_R);
        end
    
        % Step 2: Project world points to both cameras
        % Convert world points to homogeneous coordinates
        world_points_h = [world_points, ones(size(world_points, 1), 1)]'; % 4xN matrix
    
        % Project to left and right image planes
        proj_pts_L = P_L * world_points_h; % 3xN
        proj_pts_R = P_R * world_points_h; % 3xN
    
        % Normalize to get pixel coordinates (u, v)
        proj_pts_L = proj_pts_L(1:2, :) ./ proj_pts_L(3, :); % 2xN
        proj_pts_R = proj_pts_R(1:2, :) ./ proj_pts_R(3, :); % 2xN
    
        % get reprojection error
        error_L = norm(peaks_L_ud(k,:)' - proj_pts_L(:,k))
        error_R = norm(peaks_R_ud(k,:)' - proj_pts_R(:,k))
    
    
        % Step 3: Project the Aruco frame origin to both cameras
        aruco_proj_L = P_L * aruco_origin; % 3x1
        aruco_proj_R = P_R * aruco_origin; % 3x1
    
        % Normalize to get pixel coordinates for the Aruco origin
        aruco_proj_L = aruco_proj_L(1:2) / aruco_proj_L(3);
        aruco_proj_R = aruco_proj_R(1:2) / aruco_proj_R(3);
    
        % Project X, Y, Z axes
        aruco_x_L = P_L * aruco_x; aruco_x_R = P_R * aruco_x;
        aruco_y_L = P_L * aruco_y; aruco_y_R = P_R * aruco_y;
        aruco_z_L = P_L * aruco_z; aruco_z_R = P_R * aruco_z;
    
        aruco_x_L = aruco_x_L(1:2) / aruco_x_L(3);
        aruco_y_L = aruco_y_L(1:2) / aruco_y_L(3);
        aruco_z_L = aruco_z_L(1:2) / aruco_z_L(3);
    
        aruco_x_R = aruco_x_R(1:2) / aruco_x_R(3);
        aruco_y_R = aruco_y_R(1:2) / aruco_y_R(3);
        aruco_z_R = aruco_z_R(1:2) / aruco_z_R(3);
        
        % After n frames start displaying the points overlayed on the camera
        % frames
        if  k>50 && mod(k,1)==0
    
            % Overlay the projected points on the frames
            figure(11203);  imshow(frame_L); hold on;
            h_proj_L = plot(proj_pts_L(1, k), proj_pts_L(2, k), 'ro', 'MarkerSize', 5, 'LineWidth', 2);
    
            plot(aruco_proj_L(1), aruco_proj_L(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
            h_peak_L = plot(peaks_L_ud(k,1)', peaks_L_ud(k,2)', 'ro', 'MarkerSize', 5, 'LineWidth', 2, 'Color','yellow');
            % Draw arrows for the Aruco frame axes
            quiver(aruco_proj_L(1), aruco_proj_L(2), aruco_x_L(1) - aruco_proj_L(1), aruco_x_L(2) - aruco_proj_L(2), 'r', 'LineWidth', 2);
            quiver(aruco_proj_L(1), aruco_proj_L(2), aruco_y_L(1) - aruco_proj_L(1), aruco_y_L(2) - aruco_proj_L(2), 'g', 'LineWidth', 2);
            quiver(aruco_proj_L(1), aruco_proj_L(2), aruco_z_L(1) - aruco_proj_L(1), aruco_z_L(2) - aruco_proj_L(2), 'b', 'LineWidth', 2);
            
            legend([h_proj_L, h_peak_L], {'Projected World Point', 'Detected Peak'}, 'Location', 'northeast');
            title(['Left Camera - Frame ', num2str(k)]);
            hold off;
    
            figure(11231); imshow(frame_R); hold on;
            h_proj_R = plot(proj_pts_R(1, k), proj_pts_R(2, k), 'ro', 'MarkerSize', 5, 'LineWidth', 2);
            plot(aruco_proj_R(1), aruco_proj_R(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
            h_peak_R = plot(peaks_R_ud(k,1)', peaks_R_ud(k,2)', 'ro', 'MarkerSize', 5, 'LineWidth', 2, 'Color','yellow');
            quiver(aruco_proj_R(1), aruco_proj_R(2), aruco_x_R(1) - aruco_proj_R(1), aruco_x_R(2) - aruco_proj_R(2), 'r', 'LineWidth', 2);
            quiver(aruco_proj_R(1), aruco_proj_R(2), aruco_y_R(1) - aruco_proj_R(1), aruco_y_R(2) - aruco_proj_R(2), 'g', 'LineWidth', 2);
            quiver(aruco_proj_R(1), aruco_proj_R(2), aruco_z_R(1) - aruco_proj_R(1), aruco_z_R(2) - aruco_proj_R(2), 'b', 'LineWidth', 2);
            
            legend([h_proj_R, h_peak_R], {'Projected World Point', 'Detected Peak'}, 'Location', 'northeast');
            title(['Right Camera - Frame ', num2str(k)]);
            hold off;
    
            % Pause to view each frame (adjust speed if needed)
            pause(1.2);
        end
    end
end