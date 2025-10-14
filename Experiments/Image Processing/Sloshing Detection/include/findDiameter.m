function [det_ind1, det_ind2] = findDiameter(frame, D, h, inRange, varargin)
    % find_diameter: Finds left and right indices for diameter detection.
    %   Optionally uses HSV segmentation thresholds if 'inRange' is true.
    %
    %   Inputs:
    %     frame   - The input image (RGB or grayscale).
    %     D       - A diameter value (its purpose seems related to ratio calculations,
    %               but it's unused in the provided snippet's diameter calculation).
    %     h       - A height value (unused in the provided snippet).
    %     inRange - Logical flag: true for HSV segmentation, false for red channel binarization.
    %     varargin - Optional: A cell array containing HSV thresholds if inRange is true:
    %                {h_min, h_max, s_min, s_max, v_min, v_max}
    %
    %   Outputs:
    %     det_ind1 - The adjusted left index for diameter detection.
    %     det_ind2 - The adjusted right index for diameter detection.

    % Define default HSV values if not provided
    default_h_min = 88;
    default_h_max = 120;
    default_s_min = 130;
    default_s_max = 255;
    default_v_min = 0;
    default_v_max = 255;

    % Parse optional HSV arguments
    if inRange && ~isempty(varargin)
        if numel(varargin{1}) == 6
            h_min = varargin{1}(1);
            h_max = varargin{1}(2);
            s_min = varargin{1}(3);
            s_max = varargin{1}(4);
            v_min = varargin{1}(5);
            v_max = varargin{1}(6);
            disp('Using provided HSV thresholds.');
        else
            warning('Incorrect number of HSV threshold values provided. Using default HSV thresholds.');
            h_min = default_h_min;
            h_max = default_h_max;
            s_min = default_s_min;
            s_max = default_s_max;
            v_min = default_v_min;
            v_max = default_v_max;
        end
    elseif inRange % If inRange is true but no HSV values are provided
        disp('No HSV thresholds provided. Using default HSV thresholds.');
        h_min = default_h_min;
        h_max = default_h_max;
        s_min = default_s_min;
        s_max = default_s_max;
        v_min = default_v_min;
        v_max = default_v_max;
    end

    if inRange
        hsv_frame = rgb2hsv(frame);
        minval = [h_min/180 s_min/255 v_min/255]; %// Define three element vector here for each colour plane
        maxval = [h_max/180 s_max/255 v_max/255]; %// Define three element vector here for each colour plane
        
        % init logical image of size of frame
        segmented_frame = true(size(hsv_frame,1), size(hsv_frame,2));
        
        % if pixel is out of range set pixel in logical image to false
        for ii = 1 : 3
            % pixel remains true only if it satisfies the condition for all
            % channels
            segmented_frame = segmented_frame & (hsv_frame(:,:,ii) >= minval(ii) & hsv_frame(:,:,ii) <= maxval(ii));
        end
        segmented_frame = ~segmented_frame;
    else
        % Ensure frame is RGB for red channel extraction
        if size(frame, 3) < 3
            error('Input frame must be an RGB image for red channel binarization when inRange is false.');
        end
        red_channel = frame(:,:,1);
        level = 0.2; % Your fixed threshold level
        BW = imbinarize(red_channel, level);
        
        se_close= strel('disk', 3);
        closed = imclose(BW, se_close);
        se_open= strel('disk',1);
        segmented_frame = imopen(closed, se_open);
    end

    figure();
    imshowpair(frame,segmented_frame,'montage');
    [rows, columns] = size(segmented_frame);
    pxSpacing = 50;
    hold on;
    % Add grid lines to the original image side
    for row = 0:pxSpacing : rows
        line([0, columns], [row,row], 'Color', '#9A9A9A', 'LineWidth', 0.05);
        text(columns,row,num2str(row), 'HorizontalAlignment', 'left')
    end
    for col = 0 : pxSpacing : columns
        line([col, col],[0, rows],'Color','#9A9A9A','LineStyle','-','LineWidth',0.05);
        text(col,rows+pxSpacing/3,num2str(col),'HorizontalAlignment','center')
    end
    % Add grid lines to the segmented image side
    for row = 0:pxSpacing : rows
        line([columns, columns+columns], [row,row], 'Color', '#9A9A9A', 'LineWidth', 0.05);
        text(columns+columns,row,num2str(row), 'HorizontalAlignment', 'left')
    end
    for col = 0 : pxSpacing : columns
        line([col+columns, col+columns],[0, rows],'Color','#9A9A9A','LineStyle','-','LineWidth',0.05);
        text(col+columns,rows+pxSpacing/3,num2str(col),'HorizontalAlignment','center')
    end

    prompt = {'Enter height [in pixel] for Diameter detection:'};
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'600'};
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    
    if isempty(answer) % User cancelled the dialog
        det_ind1 = [];
        det_ind2 = [];
        disp('Diameter detection cancelled by user.');
        close(gcf); % Close the figure
        return;
    end

    h_det_px  = str2double(answer{1});
    
    %Diameter and Initial Height Detection
    
    % Find longest continuous columns of pixels at row h_det_px, get indeces
    % of first and last column. Assuming 0 (false/black) is the target.
    [D_ind1, D_ind2] = longest_interval(segmented_frame(h_det_px,:), 0);
    
    D_px = D_ind2 - D_ind1;     
    % ratio = D_px/D; % D is an input but 'ratio' is commented out
    det_dist_px = round(0.05*D_px);
    
    det_ind1 = D_ind1 - det_dist_px; 
    det_ind2 = D_ind2 + det_dist_px;

    % Ensure indices are within frame bounds
    det_ind1 = max(1, det_ind1);
    det_ind2 = min(columns, det_ind2);

    figure();
    imshow(segmented_frame);
    hold on;
    grid minor;
    
    % Highlight detected diameter region
    line([D_ind1 D_ind1],[h_det_px-20 h_det_px+20],'Color','green','LineWidth',1.5);
    line([D_ind2 D_ind2],[h_det_px-20 h_det_px+20],'Color','green','LineWidth',1.5);
    line([D_ind1 D_ind2],[h_det_px    h_det_px],'Color','green','LineWidth',1.5);
    text((D_ind1+D_ind2)/2,h_det_px-20,'D','Color','green','interpreter','latex','FontSize',18);
    
    % Add grid lines to the second figure as well
    [rows_seg, columns_seg] = size(segmented_frame);
    for row = 0 : pxSpacing : rows_seg
        line([1, columns_seg], [row, row], 'Color', '#9A9A9A', 'LineWidth', 0.05);
        text(columns_seg,row,num2str(row), 'HorizontalAlignment', 'left');
    end
    for col = 0 : pxSpacing : columns_seg
        line([col, col],[1, rows_seg],'Color','#9A9A9A','LineStyle','-','LineWidth',0.05);
        text(col,rows_seg+pxSpacing/3,num2str(col),'HorizontalAlignment','center');
    end
    hold off;
end

% Note: You need to have a 'longest_interval' function defined somewhere
% in your MATLAB path for this code to run.
% Example longest_interval function (if you don't have one):
% function [startIndex, endIndex] = longest_interval(rowVector, targetValue)
%     % Finds the start and end indices of the longest continuous interval
%     % of targetValue in rowVector.
%     % Assumes rowVector is a logical or binary (0/1) vector.
%     
%     if isempty(rowVector)
%         startIndex = [];
%         endIndex = [];
%         return;
%     end
% 
%     % Convert to binary if not already
%     binaryRow = (rowVector == targetValue);
% 
%     % Find start and end of all continuous segments
%     diffs = diff([0, binaryRow, 0]);
%     starts = find(diffs == 1); % Where a segment starts
%     ends = find(diffs == -1) - 1; % Where a segment ends
% 
%     if isempty(starts)
%         startIndex = [];
%         endIndex = [];
%         return;
%     end
% 
%     % Calculate lengths and find the longest
%     lengths = ends - starts + 1;
%     [~, maxIdx] = max(lengths);
% 
%     startIndex = starts(maxIdx);
%     endIndex = ends(maxIdx);
% end
