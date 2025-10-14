function [cpx, cpy, cpz, n_ctrl_pts] = getPath(traj_type, mode, varargin)
% getPath - Returns control points for a predefined trajectory
%
% Inputs:
%   traj_type - string specifying trajectory type:
%       "ark"      - Arc trajectory
%       "pp"       - Pick-and-place style
%       "test"     - Custom test path
%       "pp3d"     - 3D pick-and-place
%       "sl"       - Straight line
%       "sl_bf"    - Back-and-forth line
%       "sl3d"     - Straight line in 3D
%       "sl_short" - Short straight line
%       "circle"   - Vertical circular motion
%       "static"   - Stationary or jittering path
%       "arc"      - 3D semicircular arc
%       "B"        - Complex 3D trajectory
%
%   mode - "relative" or "absolute"
% 
%   varargin - optional p0 (1x3 vector) specifying starting point [x0, y0, z0]
%
% Outputs:
%   cpx, cpy, cpz - control point vectors
%   n_ctrl_pts    - number of control points

    if nargin < 2
        error("Mode must be defined. Choose either 'absolute' or 'relative'.");
    end

    if strcmpi(mode, "relative") 
        if ~isempty(varargin)
            p0 = varargin{1};
        else
            warning("Initial position not specified, default 0 position will be selected.");
            p0 = [0, 0, 0];
        end
    end


    switch lower(traj_type)
        case "ark"
            n_ctrl_pts = 5;
            d0 = [0.6; 0.7; 1.4];
            dx = [0.0, 0.25, 0.5, 0.25, 0.0];
            dy = [0.0, -0.2, -0.7, -1.2, -1.4];
            dz = zeros(1, 5);

        case "pp"
            n_ctrl_pts = 9;
            d0 = [0.9; 0.0; 1.4];
            dx = [0.0, -0.1, -0.2, -0.2, -0.2, -0.6, -0.7, -0.7, -0.7];
            dy = [0.0, 0.0, 0.0, 0.5, 0.6, 0.7, 0.7, 0.8, 0.9];
            dz = zeros(1, 9); 

        case "test"
            n_ctrl_pts = 9;
            d0 = [0.9; -0.3; 1.2];
            dx = [0.0, -0.1, -0.2, -0.2, -0.2, -0.3, -0.3, -0.1, 0.0];
            dy = [0.0, 0.3, 0.5, 0.6, 0.9, 0.6, 0.3, 0.2, 0.0];
            dz = zeros(1, 9);

        case "pp3d"
            n_ctrl_pts = 10;
            d0 = [0.9; 0.0; 1.2];
            dx = [0.0, -0.1, -0.2, -0.2, -0.2, -0.8, -0.8, -0.8, -0.8, -0.8];
            dy = [0.0, 0.0, 0.0, 0.5, 0.6, 0.6, 0.6, 0.7, 0.8, 0.9];
            dz = [0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.4, 0.4, 0.4];

        case "sl"
            n_ctrl_pts = 5;
            d0 = [0.6; 0.4; 1.2];
            dx = zeros(1, 5);
            dy = [0, -0.2, -0.4, -0.6, -0.8];
            dz = zeros(1, 5);

        case "sl_bf"
            n_ctrl_pts = 10;
            % d0 = [0.6; -0.4; 1.4];
            d0 = [0.6; -0.4; 1.5];
            dx = zeros(1, 10);
            dy = [0, 0.2, 0.4, 0.6, 0.8, 0.8, 0.6, 0.4, 0.2, 0];
            dz = zeros(1, 10);

            

        case "sl3d"
            n_ctrl_pts = 5;
            % d0 = [1.1; 0.6; 0.8];
            d0 = [1.1; 0.6; 1.1];
            dx = zeros(1, 5);
            dy = [0, -0.1, -0.6, -1.1, -1.2];
            dz = [0.0, 0.1, 0.2, 0.2, 0.2];

        case "sl_short"
            n_ctrl_pts = 5;
            d0 = [-0.5; 0.3; 0.2];
            dx = zeros(1, 5);
            dy = [0, -0.1, -0.3, -0.5, -0.6];
            dz = zeros(1, 5);

        case "circle"
            n_ctrl_pts = 5;
            d0 = [0.8; 0.5; 1.4];
            dx = zeros(1, 5);
            dy = [0, -0.5, -1.2, -0.5, 0];
            dz = [0, -0.4, 0, 0.6, 0];

        case "static"
            n_ctrl_pts = 5;
            d0 = [1.1; 0.6; 1.0];
            dx = zeros(1, 5);
            dy = [0.0, 0.0, 0.0, -0.1, -0.1];
            dz = zeros(1, 5);

        case "arc"
            n_ctrl_pts = 11;
            ang1 = linspace(0, pi, 1000);
            ang2 = pi / 6;
            radius = 0.6;
            % d0 = [0; 0; 0];
            d0 = [0.7508; 0; 1.2176];

            R_traj = Rz(-pi/2);
            trasl = [0;-0.6;0]; 

            x = radius * cos(ang1);
            y = radius * sin(ang1) * cos(ang2);
            z = radius * sin(ang1) * sin(ang2);

            % pos_circ = [x; y; z];
            pos_circ = R_traj*([x; y; z]+trasl);
            ctrl_pts = [pos_circ(:, 1), pos_circ(:, 100:100:900), pos_circ(:, end)];
            
            % circ_point = zeros(3,9);
            % for i=1:9
            %     j=i*100;
            %     circ_point(:,i)=pos_circ(:,j) + [0;0;0];   %con 0.7 si va oltre il terzo limite di posizione di giunto con 0.6 no, poi ci guardiamo
            % end
            
            P1 = pos_circ(:,1);
            P11 = pos_circ(:,end);

           
            %parametri per robot function
            % R_traj = Rz(-pi/2);
            % trasl = [0;-0.6;0];
            % 
            % ctrl_pts = [P1,circ_point,P11];
            % 
            % ctrl_pts_x = ctrl_pts(1,:);
            % ctrl_pts_y = ctrl_pts(2,:);
            % ctrl_pts_z = ctrl_pts(3,:);

            dx = ctrl_pts(1, :);
            dy = ctrl_pts(2, :);
            dz = ctrl_pts(3, :);

            

        case "b"
            n_ctrl_pts = 20;
            % d0 = [0.1516-0.5; -0.501; 1.1117];
            % d0 = [-0.6; -0.5; 0];
            d0 = [0.7508; 0; 1.2176];
            % d0 = [0.7508; 0; 1.1176];

            % d0 = [0.1524; -0.5; 1.2177];
            % d0 = [-0.1576; -0.5; 1.2177];

            % q_offset = [1.27692, -0.503636, -2.19692, -1.71663e-16, -1.69328, 1.86308]
            % 
            % %parametri per robot function
            R_traj = Rz(-pi/2);
            trasl = [0;-0.6;0];   %vado oltre il terzo limite di posizione di giunto
            
            % P0 = [0.5;0;0];
            % P1 = [0.5;0;0.2];
            % P2 = [0.5;0.3;0.3];
            % P3 = [0.2;0.3;0.2];
            % P4 = [0.2;0.2;0.2];
            % P5 = [-0.2;0.2;0.3]; 
            % P6 = [-0.2;0.5;0.4];
            % P7 = [-0.5;0.5;0.5];
            % P8 = [-0.5;0;0.3];
            % P9 = [-0.5;0;0];

            P0 = R_traj*([0.5;0;0]+trasl);
            P1 = R_traj*([0.5;0;0.2]+trasl);
            P2 = R_traj*([0.5;0.3;0.3]+trasl);
            P3 = R_traj*([0.2;0.3;0.2]+trasl);
            P4 = R_traj*([0.2;0.2;0.2]+trasl);
            P5 = R_traj*([-0.2;0.2;0.3]+trasl); 
            P6 = R_traj*([-0.2;0.5;0.4]+trasl);
            P7 = R_traj*([-0.5;0.5;0.5]+trasl);
            P8 = R_traj*([-0.5;0;0.3]+trasl);
            P9 = R_traj*([-0.5;0;0]+trasl);

            
            ctrl_pts = [P0, P0, P1, P1, P2, P2, P3, P3, P4, P4, P5, P5, P6, P6, P7, P7, P8, P8, P9, P9];
            
            dx = ctrl_pts(1,:);
            dy = ctrl_pts(2,:);
            dz = ctrl_pts(3,:);

        case "manual"
            d0 = [0.0; 0; 0.0];
            if strcmpi(mode, "relative")
                if length(varargin) < 2
                    error("For 'manual' with 'relative' mode, provide p0 and control points (3xN matrix).");
                end
                p0 = varargin{1};
                n_ctrl_pts = varargin{2};
                ctrl_pts_input = varargin{3};
            elseif strcmpi(mode, "absolute")
                if isempty(varargin)
                    error("For 'manual' with 'absolute' mode, provide control points (3xN matrix).");
                end
                n_ctrl_pts = varargin{1};
                ctrl_pts_input = varargin{2};
            end


            if ~ismatrix(ctrl_pts_input) || size(ctrl_pts_input, 1) ~= 3 || size(ctrl_pts_input, 2) ~= n_ctrl_pts 
                error("Manual control points must be a 3xN matrix.");
            end

            ctrl_pts = ctrl_pts_input;
            % n_ctrl_pts = size(ctrl_pts, 2);

            dx = ctrl_pts(1, :);
            dy = ctrl_pts(2, :);
            dz = ctrl_pts(3, :);

            % USAGE:
            % ctrl_pts = [0 0.1 0.2;
            % 0 0.2 0.4;
            % 1 1.1 1.2];
            % 
            % [cpx, cpy, cpz, n] = getPath("manual", "absolute", 3, ctrl_pts);


        otherwise
            error("Trajectory type '%s' not recognized.", traj_type);
    end

    % Apply offset
    switch lower(mode)
        case "relative"
            cpx = p0(1) + dx;
            cpy = p0(2) + dy;
            cpz = p0(3) + dz;

        case "absolute"
            cpx = d0(1) + dx;
            cpy = d0(2) + dy;
            cpz = d0(3) + dz;
    end
    
end