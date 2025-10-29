%--------------------------------------------------------------------------
% Prehensile_New_OCP.m
%
% Optimal Control Problem D-T (Direct Torque Control)
% Time-based trajectory optimization with direct joint torque control
%
% Solves: min sum_t L(x_t, u_t) + L_f(x_T)
%         s.t. x_{t+1} = f(x_t, u_t)
%              joint limits, torque limits, sloshing limits
%
% Formulation from: OCP.pdf (October 2025)
%
% Author: Giorgio Medico
% Date: October 2025
%--------------------------------------------------------------------------

%% Load libraries
clear all
close all
clear classes
clc

win = 0;
if win
    addpath(genpath('Data\'));
    addpath(genpath('include\'));
    addpath(genpath("casadi-3.6.5-windows64-matlab2018b"));
else
    addpath(genpath('Data/'));
    addpath(genpath('include/'));
    addpath(genpath("casadi-3.6.5-linux64-matlab2018b"));
end
import casadi.*

%% Configuration and Flags
quick_test = true;  % Set to true for batch testing with defaults (no GUI)

if quick_test
    % Quick test defaults (no GUI)
    save_fig = false;
    save_csv = false;
    save_mat = false;
    csv_joint_offset = 0;
    animations = false;
else
    % Interactive GUI mode
    [save_fig, save_csv, save_mat, csv_joint_offset, animations] = getSimulationParameters();
end
is_solved = false;

fig_name = "OCP_DT";  % Optimal Control Problem - Direct Torque
save_fig_folder = fullfile("Plots");
URDF = 'comau_smartsix5.urdf.xacro';

%% Setup: Container and Liquid Parameters
if quick_test
    % Quick test defaults
    cyl_radius = 0.035;      % 80mm radius
    fill_level = 0.049;      % 150mm fill level
    eta_lim = 0.015;        % 15mm sloshing limit
    cyl_height = 0.0991;       % 200mm container height
    total_mass = 0.197;       % 2kg total mass
    rho = 998.2;             % Water density
    visc = 0.001;           % Water viscosity
    hG = 0.0281;             % Center of mass height
    fprintf('Container parameters (defaults):\n')
    fprintf('  Radius: %.1f mm\n', cyl_radius*1000)
    fprintf('  Fill level: %.1f mm\n', fill_level*1000)
    fprintf('  Sloshing limit: %.1f mm\n', eta_lim*1000)
else
    [cyl_radius, fill_level, eta_lim, cyl_height, total_mass, rho, visc, hG] = getAndVisualizeContainerParameters();
end

% Liquid parameters
[g, m_fluid, Vol, csi11, zita, mn, ks, cs, as, ~, J, k, wn] = Parameters(cyl_radius, fill_level, rho, visc);
d_ = 2 * zita * wn;
gammaNL = (csi11^2 * fill_level * mn) / (rho * Vol * cyl_radius);

% Inertia matrix of container in G frame
Ixx = (1/12) * total_mass * (3 * cyl_radius^2 + fill_level^2);
Iyy = Ixx;
Izz = 0.5 * total_mass * cyl_radius^2;
IG_G = diag([Ixx, Iyy, Izz]);

container.cyl_radius = cyl_radius;
container.cyl_height = cyl_height;
container.fill_level = fill_level;
container.IG_G = IG_G;
container.total_mass = total_mass;
container.m_fluid = m_fluid;
container.hG = hG;

%% Setup: Container Placement and Tray
if quick_test
    % Quick test defaults: 4 containers equally spaced on a line
    num_containers = 4;
    tray_type = 'rectangle';
    tLen = 0.5;    % 500mm length
    tWid = 0.3;    % 300mm width
    tThick = 0.005; % 5mm thickness
    fCoeff = 0.3;  % Friction coefficient

    tray.length = tLen;
    tray.width = tWid;
    tray.thickness = tThick;
    tray.mu = fCoeff;

    % Container positions (6 containers equally spaced on a line along x-axis)
    x_positions = linspace(-0.15, 0.15, num_containers);
    p_7G_all = [x_positions;                % x positions
                zeros(1, num_containers);    % y positions (all on centerline)
                zeros(1, num_containers)];   % z positions (all on tray)
    p_7b_all = p_7G_all;
    p_diag = zeros(num_containers, 1);

    fprintf('  Number of containers: %d\n', num_containers)
    fprintf('  Container positions (x): ')
    fprintf('[%.3f ', p_7G_all(1,:))
    fprintf('] m\n')
else
    [num_containers, pMode, modeSpecificParam, tray_type, tLen, tWid, tThick, fCoeff] = getContainerPlacementParameters();

    tray = getTrayParameters(tray_type, tLen, tWid, tThick, fCoeff);

    if strcmpi(pMode, "manual")
        outputVectors = get2DVectorInput(num_containers);
        [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container, pMode, outputVectors);
    else
        [p_7G_all, p_7b_all, p_diag] = getContainerPlacement(num_containers, tray, container, pMode, modeSpecificParam);
    end

    % Display container positions
    disp('Displacement vectors for each container:');
    disp(p_7G_all);
end

R7G = eye(3);

%% Robot Model
robot = SmartSix();

% Tool offset (tray to end-effector)
if quick_test
    % Quick test defaults: no tool offset
    toolOffset = [0;0;0.056];
    toolRotationOffset = 0;
else
    [toolOffset, toolRotationOffset] = getToolOffsetParameters();
end
R67 = Rz(toolRotationOffset);
p_67_6 = [0; 0; tray.thickness] + toolOffset;
T67 = [R67 p_67_6; 
        0 0 0 1];

% Safety factor
safety = 1;

% Inertia in different frames
IG_7 = R7G * container.IG_G * R7G';
IG_6 = R67 * IG_7 * R67';

% Visualization
    robot_visu = visualizeSetup(URDF, tray_type, tray, container, toolOffset, toolRotationOffset, num_containers, p_7G_all);

%% Robot Limits
q_min = robot.q_min';
q_max = robot.q_max';
q_dot_min = safety * robot.qp_min';
q_dot_max = safety * robot.qp_max';
q_ddot_min = safety * robot.qpp_min';
q_ddot_max = safety * robot.qpp_max';

% Torque limits (Nm)
tau_min = -500 * ones(6, 1);  % Joint torque limits
tau_max = 500 * ones(6, 1);

%% Reference Trajectory Setup - TODO use B-spline k=4
% For OCP D-T, we use pose tracking instead of geometric path
% Define a reference end-effector trajectory (position + yaw orientation)

if quick_test
    % Quick test defaults: straight line trajectory with yaw rotation
    ref_pos_start = [0.5; 0.0; 1.0];   % Start position
    ref_pos_end = [0.8; 0.0; 0.9];     % End position
    ref_rot_z_start = 0;                % Start yaw (rad)
    ref_rot_z_end = deg2rad(45);        % End yaw (rad) - 45 deg rotation
    T_fixed = 5.0;                      % Total time horizon (seconds)
    dt = 0.1;                           % Time step (seconds)
    N = round(T_fixed / dt);            % Number of control intervals
    dt = T_fixed / N;                   % Adjusted dt for exact division

    fprintf('Reference trajectory:\n')
    fprintf('  Start: [%.2f, %.2f, %.2f] m, yaw: %.1f deg\n', ...
            ref_pos_start, rad2deg(ref_rot_z_start))
    fprintf('  End: [%.2f, %.2f, %.2f] m, yaw: %.1f deg\n', ...
            ref_pos_end, rad2deg(ref_rot_z_end))
    fprintf('Optimization time parameters:\n')
    fprintf('  Time horizon: %.2f s\n', T_fixed)
    fprintf('  Time step: %.4f s\n', dt)
    fprintf('  Control intervals: %d\n', N)
    fprintf('  Reference trajectory points: %d\n', N+1)
else
    % Interactive trajectory definition
    disp('Setting up reference end-effector trajectory...')
    [ref_pos_start, ref_pos_end, ref_rot_z_start, ref_rot_z_end, T_fixed, dt, N] = getReferencePoseTrajectory();
end

% Create reference trajectory (linear interpolation) directly at optimization resolution
t_ref = linspace(0, 1, N+1);
p_ref = [linspace(ref_pos_start(1), ref_pos_end(1), N+1);
         linspace(ref_pos_start(2), ref_pos_end(2), N+1);
         linspace(ref_pos_start(3), ref_pos_end(3), N+1)];
rot_z_ref = linspace(ref_rot_z_start, ref_rot_z_end, N+1);

% Store reference trajectory
traj_ref.t = t_ref;
traj_ref.p = p_ref;
traj_ref.rot_z = rot_z_ref;

%% State and Control Dimensions
x_dim = 6 + 6 + 4 * num_containers;  % q, q_dot, sloshing states (4 per container)
u_dim = 6;  % Joint torques

disp(['State dimension: ' num2str(x_dim)])
disp(['Control dimension: ' num2str(u_dim)])

%% Optimization Setup

% Reference trajectory is already at optimization resolution (N+1 points)
p_ref_opt = traj_ref.p;
rot_z_ref_opt = traj_ref.rot_z;

% Optimization object
opti = casadi.Opti();

% Decision variables with scaling
% Scale factors improve convergence
scaling_q = ones(6, 1);
scaling_qdot = ones(6, 1);
scaling_slosh = 0.01 * ones(4 * num_containers, 1);
scaling_x = [scaling_q; scaling_qdot; scaling_slosh];

X = repmat(scaling_x, 1, N+1) .* opti.variable(x_dim, N+1);  % State trajectory
U = repmat(10 * ones(u_dim, 1), 1, N) .* opti.variable(u_dim, N);  % Control inputs (torques) - scaled down

% Extract state components
q_traj = X(1:6, :);
q_dot_traj = X(7:12, :);
slosh_traj = X(13:end, :);

%% Create CasADi Dynamics Functions
disp('Creating CasADi dynamics functions...')

% Package sloshing parameters
dynamics_params = struct();
dynamics_params.wn = wn;
dynamics_params.zita = zita;
dynamics_params.mn = mn;
dynamics_params.cyl_radius = cyl_radius;
dynamics_params.g = g;
dynamics_params.as = as;
dynamics_params.csi11 = csi11;
dynamics_params.fill_level = fill_level;
dynamics_params.m_fluid = m_fluid;

% Create CasADi functions for dynamics
[dynamics_fcn, rk4_fcn, fk_position_fcn, fk_yaw_fcn] = createDynamicsCasadiFcn(robot, T67, p_7G_all, num_containers, dynamics_params);

%% Cost Function
% Stage cost: L(x_t, u_t) = 1/2 ||p_EE(q_t) - p_ref,t||^2_Qp
%                          + 1/2 ||yaw(q_t) - yaw_ref,t||^2_Qrot
%                          + 1/2 sum_i ||eta_i(x_t)||^2_Qeta
%                          + 1/2 ||u_t||^2_R
% Terminal cost: L_f(x_T) = 1/2 ||p_EE(q_T) - p_ref,T||^2_Pp
%                          + 1/2 ||yaw(q_T) - yaw_ref,T||^2_Prot
%                          + 1/2 sum_i ||eta_i(x_T)||^2_Peta
%                          + 1/2 ||q_dot_T||^2_Pqdot

% Weights
Qp = 100 * eye(3);       % End-effector position tracking weight
Qrot = 100;              % End-effector yaw tracking weight
Qeta = 100 * eye(num_containers);  % Sloshing suppression weight
R = 0.01 * eye(6);      % Control effort weight
Pp = 1000 * eye(3);      % Final position weight
Prot = 1000;             % Final yaw tracking weight
Peta = 1000 * eye(num_containers);  % Final sloshing weight
Pqdot = 100 * eye(6);   % Final velocity weight

% Sloshing height calculation: eta = (xi11^2 * h * m1 / (mf * R)) * sqrt(x^2 + y^2)
eta_scale = (csi11^2 * fill_level * mn) / (m_fluid * cyl_radius);

cost = 0;

% Stage costs
for t = 1:N
    % End-effector position tracking
    q_t = X(1:6, t);
    p_EE_t = fk_position_fcn(q_t);
    p_err_t = p_EE_t - p_ref_opt(:, t);
    cost = cost + 0.5 * (p_err_t' * Qp * p_err_t);

    % End-effector yaw tracking
    yaw_t = fk_yaw_fcn(q_t);
    yaw_err_t = yaw_t - rot_z_ref_opt(t);
    cost = cost + 0.5 * Qrot * yaw_err_t^2;

    % Sloshing heights
    for i = 1:num_containers
        x_idx = 12 + (i-1)*4 + 1;
        y_idx = 12 + (i-1)*4 + 2;
        eta_i = eta_scale * sqrt(X(x_idx, t)^2 + X(y_idx, t)^2);
        cost = cost + 0.5 * Qeta(i, i) * eta_i^2;
    end

    % Control effort
    cost = cost + 0.5 * (U(:, t)' * R * U(:, t));
end

% Terminal cost: position tracking, yaw tracking, suppress final sloshing and velocities
q_T = X(1:6, N+1);
p_EE_T = fk_position_fcn(q_T);
p_err_T = p_EE_T - p_ref_opt(:, N+1);
cost = cost + 0.5 * (p_err_T' * Pp * p_err_T);

% Terminal yaw tracking
yaw_T = fk_yaw_fcn(q_T);
yaw_err_T = yaw_T - rot_z_ref_opt(N+1);
cost = cost + 0.5 * Prot * yaw_err_T^2;

q_dot_T = X(7:12, N+1);
cost = cost + 0.5 * (q_dot_T' * Pqdot * q_dot_T);

for i = 1:num_containers
    x_idx = 12 + (i-1)*4 + 1;
    y_idx = 12 + (i-1)*4 + 2;
    eta_i_T = eta_scale * sqrt(X(x_idx, N+1)^2 + X(y_idx, N+1)^2);
    cost = cost + 0.5 * Peta(i, i) * eta_i_T^2;
end

opti.minimize(cost);

%% Dynamics and Constraints
disp('Setting up dynamics constraints...')

% Apply RK4 dynamics constraints for each time step
for k = 1:N
    % Get current state and control
    x_k = X(:, k);
    u_k = U(:, k);

    % Compute next state using RK4 integration (fully symbolic in CasADi)
    x_next = rk4_fcn(x_k, u_k, dt);

    % Add dynamics constraint: x_{k+1} = RK4(x_k, u_k, dt)
    opti.subject_to(X(:, k+1) == x_next);
end

disp(['Dynamics constraints added: ' num2str(N) ' RK4 integration steps'])

%% Joint Limits
opti.subject_to(q_min <= q_traj <= q_max);
opti.subject_to(q_dot_min <= q_dot_traj <= q_dot_max);

%% Torque Limits
opti.subject_to(tau_min <= U <= tau_max);

%% Sloshing Height Constraint
for i = 1:num_containers
    for t = 1:N+1
        x_idx = 12 + (i-1)*4 + 1;
        y_idx = 12 + (i-1)*4 + 2;
        eta_i = eta_scale * sqrt(X(x_idx, t)^2 + X(y_idx, t)^2);
        opti.subject_to(eta_i <= eta_lim);
    end
end

%% Initial and Final Conditions
% Initial: Use reference start position and yaw angle, compute IK
disp('Computing initial configuration via inverse kinematics...')

% Create initial transformation from reference start position and yaw
% Use reference yaw angle for z-rotation
R_0_6_init = Rz(ref_rot_z_start);  % Rotation around z-axis by reference yaw
T_0_6_init = [R_0_6_init, ref_pos_start; 0, 0, 0, 1];

% Compute inverse kinematics for initial configuration
try
    % Try different IK solutions (elbow configurations)
    q_0_found = false;
    for sol = 1:8  % Try all 8 IK solutions
        try
            q_0_candidate = robot.ik(T_0_6_init, sol);
            % Check if configuration is within joint limits
            if all(q_0_candidate >= robot.q_min') && all(q_0_candidate <= robot.q_max')
                q_0 = q_0_candidate;
                q_0_found = true;
                disp(['  Found valid IK solution (config ' num2str(sol) ')'])
                break;
            end
        catch
            % This IK solution doesn't exist, try next
            continue;
        end
    end

    if ~q_0_found
        warning('No valid IK solution found within joint limits. Using nominal configuration.')
        q_0 = [0; 0; 0; 0; 0; 0];
    end
catch ME
    warning(['IK computation failed: ' ME.message '. Using nominal configuration.'])
    q_0 = [0; 0; 0; 0; 0; 0];
end

x_init = [q_0; zeros(6, 1); zeros(4*num_containers, 1)];
opti.subject_to(X(:, 1) == x_init);

disp(['  Initial joint configuration: [' num2str(rad2deg(q_0'), '%.2f ') '] deg'])

% Final: Zero velocities and minimal sloshing (soft constraint via cost function)
% Don't strictly enforce final state, let optimizer minimize it

%% Initial Guess
opti.set_initial(X(:, 1), x_init);
for t = 2:N+1
    opti.set_initial(X(:, t), x_init);  % Start from initial state
end
opti.set_initial(U, zeros(u_dim, N));

%% Solver Setup
p_opts = struct();
    s_opts = struct('print_level', 5);
opti.solver('ipopt', p_opts, s_opts);

%% Solve
    disp('Starting optimization...')
    f_opti = figure();
    set(f_opti, 'Name', 'OCP Optimization Progress', 'NumberTitle', 'off', ...
        'Position', [200, 200, 1000, 480]);

tic
try
    sol = opti.solve();
    is_solved = true;
catch ME
        disp(['Optimization failed: ' ME.message])
        sol = opti.debug();
        sol.show_infeasibilities(1e-4);
end
t_calc = toc;

    disp(['Optimization time: ' num2str(t_calc) ' seconds'])

%% Extract Solution
if is_solved
    X_sol = sol.value(X);
    U_sol = sol.value(U);
    T_sol = T_fixed;  % Use fixed time horizon

    % Time grid
    t_vec = linspace(0, T_sol, N+1);

    % Extract trajectories
    q_sol = X_sol(1:6, :);
    q_dot_sol = X_sol(7:12, :);
    slosh_sol = X_sol(13:end, :);

        disp(' ')
        disp('--------------------------')
        disp(['Optimal time horizon: ' num2str(T_sol) ' seconds'])
        disp('--------------------------')
    else
        disp('Solution not available - optimization failed')
    return
end

%% Post-Processing: Joint Velocity Check
disp('Verifying joint limits...')

q_dot_max_actual = max(abs(q_dot_sol), [], 2);
if all(q_dot_max_actual <= robot.qp_max')
    disp('Joint velocities OK!')
else
    disp('Warning: Joint velocities exceed limits')
end

%% Post-Processing: Sloshing Heights
    disp('Computing final sloshing heights...')

eta_scale = (csi11^2 * fill_level * mn) / (m_fluid * cyl_radius);

fig_etas = figure();
set(fig_etas, 'Name', 'Sloshing Heights', 'NumberTitle', 'off', ...
    'Position', [200, 200, 800, 480]);
hold on
grid on
box on

for i = 1:num_containers
    x_idx = 12 + (i-1)*4 + 1;
    y_idx = 12 + (i-1)*4 + 2;
    eta_i = eta_scale * sqrt(slosh_sol(x_idx-12, :).^2 + slosh_sol(y_idx-12, :).^2);
    plot(t_vec, eta_i * 1000, 'LineWidth', 1.5);
    legend_entries{i} = sprintf('Container %d', i);
end

yline(eta_lim * 1000, '--k', 'LineWidth', 1.2)
xlim([0, T_sol])
title('Sloshing Heights', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('$\eta$ [mm]', 'Interpreter', 'latex')
legend(legend_entries, 'Location', 'best')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

if save_fig
    save_name = strcat(fig_name, '_etas.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(fig_etas, 'Renderer', 'painters')
    set(fig_etas, 'Units', 'Inches');
    fig_pos = get(fig_etas, 'Position');
    set(fig_etas, 'PaperUnits', 'Inches');
    set(fig_etas, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(fig_etas, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    print(fig_etas, '-dpdf', save_name)
end

%% Post-Processing: Joint Trajectories
fig_joints = figure();
set(fig_joints, 'Name', 'Joint Trajectories', 'NumberTitle', 'off', ...
    'Position', [200, 200, 1000, 600]);

for i = 1:6
    subplot(3, 2, i)
    hold on
    grid on
    plot(t_vec, rad2deg(q_sol(i, :)), 'LineWidth', 1.5)
    xlabel('Time [s]')
    ylabel(sprintf('Joint %d [deg]', i))
    title(sprintf('Joint Position %d', i))
end

if save_fig
    save_name = strcat(fig_name, '_joints.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(fig_joints, 'Renderer', 'painters')
    set(fig_joints, 'Units', 'Inches');
    fig_pos = get(fig_joints, 'Position');
    set(fig_joints, 'PaperUnits', 'Inches');
    set(fig_joints, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(fig_joints, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    print(fig_joints, '-dpdf', save_name)
end

%% Post-Processing: Control Inputs
fig_control = figure();
set(fig_control, 'Name', 'Control Inputs (Torques)', 'NumberTitle', 'off', ...
    'Position', [200, 200, 1000, 600]);

t_ctrl = linspace(0, T_sol, N);
for i = 1:6
    subplot(3, 2, i)
    hold on
    grid on
    plot(t_ctrl, U_sol(i, :), 'LineWidth', 1.5)
    xlabel('Time [s]')
    ylabel(sprintf('Torque %d [Nm]', i))
    title(sprintf('Joint Torque %d', i))
    yline(tau_max(i), '--r', 'LineWidth', 1)
    yline(tau_min(i), '--r', 'LineWidth', 1)
end

if save_fig
    save_name = strcat(fig_name, '_control.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(fig_control, 'Renderer', 'painters')
    set(fig_control, 'Units', 'Inches');
    fig_pos = get(fig_control, 'Position');
    set(fig_control, 'PaperUnits', 'Inches');
    set(fig_control, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(fig_control, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    print(fig_control, '-dpdf', save_name)
end

%% Post-Processing: Yaw Tracking
    disp('Computing yaw trajectory from solution...')

% Compute actual yaw angles from joint solution
yaw_sol = zeros(1, N+1);
for t = 1:N+1
    T_0_6_t = robot.fk(q_sol(:, t));
    R_0_6_t = T_0_6_t(1:3, 1:3);
    yaw_sol(t) = atan2(R_0_6_t(2, 1), R_0_6_t(1, 1));
end

fig_yaw = figure();
set(fig_yaw, 'Name', 'Yaw Tracking', 'NumberTitle', 'off', ...
    'Position', [200, 200, 800, 480]);
hold on
grid on
box on

plot(t_vec, rad2deg(yaw_sol), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual')
plot(t_vec, rad2deg(rot_z_ref_opt), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference')

xlim([0, T_sol])
title('End-Effector Yaw Tracking', 'Interpreter', 'latex')
xlabel('$t$ [s]', 'Interpreter', 'latex')
ylabel('Yaw [deg]', 'Interpreter', 'latex')
legend('Location', 'best')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12)

if save_fig
    save_name = strcat(fig_name, '_yaw.pdf');
    save_name = fullfile(save_fig_folder, save_name);
    set(fig_yaw, 'Renderer', 'painters')
    set(fig_yaw, 'Units', 'Inches');
    fig_pos = get(fig_yaw, 'Position');
    set(fig_yaw, 'PaperUnits', 'Inches');
    set(fig_yaw, 'PaperSize', [fig_pos(3), fig_pos(4)]);
    set(fig_yaw, 'PaperPosition', [0, 0, fig_pos(3), fig_pos(4)]);
    print(fig_yaw, '-dpdf', save_name)
end

    disp('Optimization complete!')
