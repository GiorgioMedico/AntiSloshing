function [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container, mode, varargin)
% getContainerPlacement - Compute spatial placement of containers on a tray.
%
%   [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container)
%
%   This function computes the positions of multiple containers arranged on a tray,
%   assuming a square grid layout. It returns the 3D positions of each container's 
%   center of mass (COM), the base center of each container (assuming vertical orientation),
%   and the radial distance of each container's center from the tray center.
%
%   Inputs:
%       - num_containers : Integer number of containers to place
%       - tray           : Struct with tray properties:
%                             .length   - tray length (m)
%                             .width    - tray width (m)
%       - container      : Struct with container properties:
%                             .cyl_radius  - radius of the cylinder (m)
%                             .fill_level  - height of the liquid (m)
%                             .hG          - vertical coordinate of the COM (m)
%       - mode           : 'auto', 'diag', 'inline' or 'circle' (placement configuration)
%       - varargin       : (Optional) diag_radius (used only in 'circle' and 'diag' mode)
%
%   Outputs:
%       - p_7G_all : [3 x N] array, each column is the position vector of the COM 
%                    of the k-th container in frame 7
%       - p_7b_all : [3 x N] array, each column is the position vector of the base 
%                    center of the k-th container in frame 7
%       - p_diag   : [1 x N] vector of radial distances of each container's position
%                    from the center of the tray (in X-Y plane)

    edge_dist = 0.05; % distance from edge of container to edge of tray

    tray_len = tray.length;
    tray_width = tray.width;
    cyl_radius = container.cyl_radius;
    cyl_height = container.fill_level;
    hG = container.hG;

    switch lower(mode)
        case 'auto'
            % containers placed on an evenly spaced square meshgrid 
            spacing = linspace(-min(tray_width, tray_len)/2 + cyl_radius + edge_dist, ...
                                min(tray_width, tray_len)/2 - cyl_radius - edge_dist, ...
                                ceil(sqrt(num_containers)));
            [x_positions, y_positions] = meshgrid(spacing, spacing);
            x_positions = x_positions(:);
            y_positions = y_positions(:);

        case 'inline'

            % Choose longest tray edge
            if tray_len >= tray_width
                axis = 'x';
                axis_len = tray_len;
            else
                axis = 'y';
                axis_len = tray_width;
            end
            disp(axis)
            disp(axis_len)
        
            % Define fixed spacing
            % spacing = 0.09;
            if isempty(varargin) || isempty(varargin{:})
                error('spacing must be specified for ''inline'' mode.');
            end
            spacing = varargin{1};
            total_span = (num_containers - 1) * spacing;
            required_space = total_span + 2 * (cyl_radius + edge_dist);
        
            if required_space > axis_len + 1e-3
                error('Not enough space to place containers in inline mode.');
            end
        
            % Compute positions centered around 0
            positions = linspace(-total_span/2, total_span/2, num_containers);
        
            % Assign coordinates
            switch axis
                case 'x'
                    x_positions = positions;
                    y_positions = zeros(size(positions));
                case 'y'
                    y_positions = positions;
                    x_positions = zeros(size(positions));
            end

        case {'circle', 'diag'}
            if isempty(varargin) || isempty(varargin{:})
                error('diag_radius must be specified for ''circle'' mode.');
            end
            diag_radius = varargin{1};

            if strcmp(mode, 'circle')
                % Compute evenly spaced angles on a circle
                angles = linspace(0, 2*pi, num_containers + 1);
                angles(end) = [];  % remove duplicate 2π point
    
                x_positions = diag_radius * cos(angles);
                y_positions = diag_radius * sin(angles);

            else % 'diag' mode
                if num_containers > 4
                    error('Diag mode supports up to 4 containers.');
                end

                % Dynamically computed diagonal unit vectors
                d1 = [tray_len; tray_width];
                d1 = d1 / norm(d1); % diagonal ↘↖
        
                d2 = [tray_len; -tray_width];
                d2 = d2 / norm(d2); % diagonal ↗↙

                positions = [];

                switch num_containers
                    case 1
                        positions = [0; 0];
                    case 2
                        positions = [-d1, d1] * diag_radius;
                    case 3
                        positions = [-d1, d1, d2] * diag_radius;
                    case 4
                        positions = [-d1, d1, -d2, d2] * diag_radius;
                end

                x_positions = positions(1, :).';
                y_positions = positions(2, :).';
            end
        
        case 'manual'
            % Expecting varargin{1} to be a [2 x num_containers] matrix of coordinates
            if isempty(varargin) || isempty(varargin{1})
                error('In ''manual'' mode, a [2 x N] matrix of positions must be provided.');
            end

            manual_positions = varargin{1};

            % Sanity check: correct size
            if size(manual_positions, 1) ~= 2 || size(manual_positions, 2) ~= num_containers
                error('Manual positions must be a [2 x num_containers] matrix.');
            end

            x_positions = manual_positions(1, :).';
            y_positions = manual_positions(2, :).';

        otherwise
            error('Invalid mode. Use ''auto'', ''circle'', ''diag'' or ''manual''.');
    end

    if num_containers == 1
        x_positions = 0;
        y_positions = 0;
    end

    % === Build outputs ===
    p_7G_all = zeros(3, num_containers);
    p_7b_all = zeros(3, num_containers);
    p_diag = zeros(1, num_containers);

    for k = 1:num_containers
        x = x_positions(k);
        y = y_positions(k);
        p_7G_all(:, k) = [x; y; hG];
        p_7b_all(:, k) = [x; y; hG - cyl_height/2];
        p_diag(k) = sqrt(x^2 + y^2);
    end

    % === Collision check ===
    for i = 1:num_containers
        for j = i+1:num_containers
            d = norm(p_7G_all(1:2, i) - p_7G_all(1:2, j));
            if d < 2 * cyl_radius - 1e-6
                error('Collision detected between containers %d and %d.', i, j);
            end
        end
    end

    % Final bounds check: ensure no container exceeds tray limits
    if any(abs(x_positions) + cyl_radius > tray_len / 2) || ...
       any(abs(y_positions) + cyl_radius > tray_width / 2)
        error('At least one container base exceeds tray boundaries.');
    end

end

