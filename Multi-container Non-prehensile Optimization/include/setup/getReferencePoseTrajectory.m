function [pos_start, pos_end, rot_z_start, rot_z_end, T_fixed, dt, N] = getReferencePoseTrajectory()
    % getReferencePoseTrajectory - Interactive GUI to define reference EE trajectory
    %
    % For OCP D-T formulation, the reference trajectory is the desired EE pose
    % (position + yaw orientation) that the optimization should track.
    %
    % Output:
    %   pos_start: [x, y, z] starting position (meters)
    %   pos_end: [x, y, z] ending position (meters)
    %   rot_z_start: starting yaw angle (radians)
    %   rot_z_end: ending yaw angle (radians)
    %   T_fixed: total time horizon (seconds)
    %   dt: time step for optimization (seconds)
    %   N: number of control intervals (computed)

    % Default values
    default_start = [0.5; 0.5; 1.0];   % Default start position
    default_end = [0.8; 0.5; 0.9];     % Default end position
    default_yaw_start = 0;              % Default start yaw (degrees)
    default_yaw_end = 0;                % Default end yaw (degrees)
    default_T = 5.0;                    % Default time horizon (seconds)
    default_dt = 0.1;                   % Default time step (seconds)

    % Create dialog
    prompt = {
        'Reference trajectory start X [m]:'
        'Reference trajectory start Y [m]:'
        'Reference trajectory start Z [m]:'
        'Reference trajectory start Yaw [deg]:'
        'Reference trajectory end X [m]:'
        'Reference trajectory end Y [m]:'
        'Reference trajectory end Z [m]:'
        'Reference trajectory end Yaw [deg]:'
        'Total time horizon T [s]:'
        'Time step dt [s]:'
    };

    dlgtitle = 'Reference End-Effector Trajectory and Time Parameters';
    dims = [1 35];
    definput = {
        num2str(default_start(1))
        num2str(default_start(2))
        num2str(default_start(3))
        num2str(default_yaw_start)
        num2str(default_end(1))
        num2str(default_end(2))
        num2str(default_end(3))
        num2str(default_yaw_end)
        num2str(default_T)
        num2str(default_dt)
    };

    answer = inputdlg(prompt, dlgtitle, dims, definput);

    if isempty(answer)
        % User cancelled - use defaults
        pos_start = default_start;
        pos_end = default_end;
        rot_z_start = deg2rad(default_yaw_start);
        rot_z_end = deg2rad(default_yaw_end);
        T_fixed = default_T;
        dt = default_dt;
    else
        pos_start = [
            str2double(answer{1});
            str2double(answer{2});
            str2double(answer{3})
        ];
        rot_z_start = deg2rad(str2double(answer{4}));  % Convert to radians
        pos_end = [
            str2double(answer{5});
            str2double(answer{6});
            str2double(answer{7})
        ];
        rot_z_end = deg2rad(str2double(answer{8}));    % Convert to radians
        T_fixed = str2double(answer{9});
        dt = str2double(answer{10});
    end

    % Validation

    if T_fixed <= 0
        warning('T_fixed must be positive. Using default: %.2f s', default_T);
        T_fixed = default_T;
    end

    if dt <= 0 || dt >= T_fixed
        warning('dt must be positive and less than T_fixed. Using default: %.4f s', default_dt);
        dt = default_dt;
    end

    % Compute N (number of control intervals)
    N_raw = T_fixed / dt;
    N = round(N_raw);

    % Ensure minimum number of intervals
    if N < 10
        warning('N must be at least 10. Adjusting dt to achieve N=10.');
        N = 10;
    end

    % Adjust dt for exact division
    dt = T_fixed / N;

    disp(' ')
    disp('Reference Trajectory Configuration:')
    disp(['  Start position: [' num2str(pos_start') '] m'])
    disp(['  Start yaw: ' num2str(rad2deg(rot_z_start)) ' deg'])
    disp(['  End position: [' num2str(pos_end') '] m'])
    disp(['  End yaw: ' num2str(rad2deg(rot_z_end)) ' deg'])
    disp(' ')
    disp('Optimization Time Parameters:')
    disp(['  Time horizon T: ' num2str(T_fixed) ' s'])
    disp(['  Time step dt: ' num2str(dt, '%.6f') ' s'])
    disp(['  Control intervals N: ' num2str(N)])
    disp(['  Reference trajectory points: ' num2str(N+1)])
    disp(' ')
end
