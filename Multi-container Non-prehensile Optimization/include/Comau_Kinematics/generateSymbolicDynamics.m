%% generateSymbolicDynamics.m
% Generates exact symbolic dynamics functions for SmartSix robot from URDF
%
% This script derives closed-form symbolic expressions for:
%   - M(q): Mass/inertia matrix (6x6)
%   - C(q, q_dot): Coriolis + centrifugal vector (6x1)
%   - G(q): Gravity vector (6x1)
%
% The derived functions are compatible with CasADi MX variables and provide
% exact dynamics based on URDF inertial parameters.
%
% Output files (auto-generated):
%   - SmartSix_M_sym.m: Mass matrix function
%   - SmartSix_C_sym.m: Coriolis vector function
%   - SmartSix_G_sym.m: Gravity vector function
%
% Author: Giorgio Medico
% Date: October 2025

clear all
close all
clear classes
clc

%% Configuration
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║     SmartSix Symbolic Dynamics Generator from URDF            ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

% Output directory for generated functions
output_dir = fileparts(mfilename('fullpath'));
urdf_path = fullfile(output_dir, 'URDF', 'comau_smartsix5.urdf');

% Check if Symbolic Math Toolbox is available
if ~license('test', 'Symbolic_Toolbox')
    error('Symbolic Math Toolbox is required but not available.');
end

%% Load Robot Model
fprintf('→ Loading URDF from: %s\n', urdf_path);
if ~isfile(urdf_path)
    error('URDF file not found: %s', urdf_path);
end

% Load rigidBodyTree for parameter extraction
robot_rbt = importrobot(urdf_path);
robot_rbt.DataFormat = 'row';
fprintf('  ✓ Loaded %d links, %d joints\n', numel(robot_rbt.Bodies), numel(robot_rbt.Bodies));

%% Create SmartSix Robot Object for DH Parameters
robot = SmartSix();
fprintf('  ✓ SmartSix DH parameters loaded\n');

%% Extract Inertial Parameters from URDF
% The URDF has links: base_comau, axes_1, axes_2, axes_3, axes_4, axes_5, axes_6
% We need axes_1 through axes_6 (6 movable links)

link_names = {'axes_1', 'axes_2', 'axes_3', 'axes_4', 'axes_5', 'axes_6'};
n_links = length(link_names);

% Storage for inertial parameters
mass = zeros(n_links, 1);
com = zeros(n_links, 3);   % Center of mass in link frame
inertia = cell(n_links, 1); % 3x3 inertia tensor

fprintf('\n→ Extracting inertial parameters:\n');
for i = 1:n_links
    body = robot_rbt.getBody(link_names{i});
    mass(i) = body.Mass;
    com(i, :) = body.CenterOfMass;  % [x, y, z] in link frame
    inertia{i} = body.Inertia;       % [Ixx Iyy Izz Iyz Ixz Ixy]

    fprintf('  Link %d (%s): m=%.3f kg, COM=[%.4f %.4f %.4f]\n', ...
            i, link_names{i}, mass(i), com(i, 1), com(i, 2), com(i, 3));
end

%% Create Symbolic Variables
fprintf('\n→ Creating symbolic variables...\n');
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
syms g_sym real  % Gravity

q_sym = [q1; q2; q3; q4; q5; q6];
qd_sym = [qd1; qd2; qd3; qd4; qd5; qd6];

fprintf('  ✓ Created symbolic joint variables\n');

%% Derive Forward Kinematics Symbolically
fprintf('\n→ Deriving symbolic forward kinematics...\n');

% DH parameters from SmartSix
a = robot.a;
d = robot.d;
alpha = robot.alpha;

% Compute transformation matrices for each link
T = cell(n_links + 1, 1);
T{1} = eye(4);  % Base frame

for i = 1:n_links
    % DH transformation matrix
    T_i = dh_transform(a(i), alpha(i), d(i), q_sym(i));
    T{i+1} = T{i} * T_i;
end

fprintf('  ✓ Computed symbolic transforms T_0_i for i=1..6\n');

%% Compute Symbolic Jacobians for Each Link
fprintf('\n→ Computing geometric Jacobians...\n');

% Origins of each link frame in base frame
p = cell(n_links + 1, 1);
p{1} = [0; 0; 0];
for i = 1:n_links
    p{i+1} = T{i+1}(1:3, 4);
end

% Rotation matrices
R = cell(n_links + 1, 1);
R{1} = eye(3);
for i = 1:n_links
    R{i+1} = T{i+1}(1:3, 1:3);
end

% Joint axes (all revolute, z-axis in local frame)
z = cell(n_links + 1, 1);
z{1} = [0; 0; 1];
for i = 1:n_links
    z{i+1} = R{i+1}(:, 3);
end

fprintf('  ✓ Extracted joint axes and positions\n');

%% Derive Lagrangian Dynamics: M, C, G
fprintf('\n→ Deriving Lagrangian dynamics (this may take several minutes)...\n');

% Total kinetic energy
fprintf('  → Computing kinetic energy...\n');
KE = sym(0);

for i = 1:n_links
    % Position of COM in base frame
    p_com_local = [com(i, :)'; 1];  % Homogeneous coordinates
    p_com_global = T{i+1} * p_com_local;
    p_com = p_com_global(1:3);

    % Velocity of COM (using Jacobian approach)
    % For each joint j <= i, compute contribution to COM velocity
    v_com = sym([0; 0; 0]);
    omega_i = sym([0; 0; 0]);

    for j = 1:i
        % Joint j contributes to link i
        v_com = v_com + cross(z{j}, p_com - p{j}) * qd_sym(j);
        omega_i = omega_i + z{j} * qd_sym(j);
    end

    % Translational kinetic energy
    KE_trans = (1/2) * mass(i) * (v_com.' * v_com);

    % Rotational kinetic energy
    % Convert inertia from [Ixx Iyy Izz Iyz Ixz Ixy] to 3x3 matrix
    I_local = [inertia{i}(1), inertia{i}(6), inertia{i}(5);
               inertia{i}(6), inertia{i}(2), inertia{i}(4);
               inertia{i}(5), inertia{i}(4), inertia{i}(3)];

    % Transform inertia tensor to base frame
    I_global = R{i+1} * I_local * R{i+1}.';

    KE_rot = (1/2) * (omega_i.' * I_global * omega_i);

    KE = KE + KE_trans + KE_rot;
    fprintf('    Link %d/%d processed\n', i, n_links);
end

fprintf('  ✓ Kinetic energy derived\n');

% Total potential energy
fprintf('  → Computing potential energy...\n');
PE = sym(0);

for i = 1:n_links
    % Position of COM in base frame
    p_com_local = [com(i, :)'; 1];
    p_com_global = T{i+1} * p_com_local;

    % Height (z-coordinate) of COM
    h_com = p_com_global(3);

    PE = PE + mass(i) * g_sym * h_com;
end

fprintf('  ✓ Potential energy derived\n');

% Lagrangian
L = KE - PE;

%% Compute Mass Matrix M(q)
fprintf('\n→ Computing mass matrix M(q)...\n');
M_sym = sym(zeros(6, 6));

for i = 1:6
    for j = 1:6
        % M_ij = ∂²L/∂q̇_i∂q̇_j
        M_sym(i, j) = diff(diff(L, qd_sym(i)), qd_sym(j));
    end
    fprintf('  Row %d/%d completed\n', i, 6);
end

M_sym = simplify(M_sym);
fprintf('  ✓ Mass matrix M(q) derived [6x6]\n');

%% Compute Christoffel Symbols and Coriolis Matrix
fprintf('\n→ Computing Coriolis/centrifugal terms...\n');

% Christoffel symbols of the first kind
C_mat = sym(zeros(6, 6));  % Coriolis matrix such that C(q, q̇) = C_mat * q̇

for k = 1:6
    for j = 1:6
        c_kj = sym(0);
        for i = 1:6
            % c_kj = Σ_i (∂M_kj/∂q_i - 1/2 ∂M_ij/∂q_k) * q̇_i
            c_kj = c_kj + (diff(M_sym(k, j), q_sym(i)) - ...
                          (1/2) * diff(M_sym(i, j), q_sym(k))) * qd_sym(i);
        end
        C_mat(k, j) = c_kj;
    end
    fprintf('  Row %d/%d completed\n', k, 6);
end

C_sym = simplify(C_mat * qd_sym);
fprintf('  ✓ Coriolis vector C(q, q̇) derived [6x1]\n');

%% Compute Gravity Vector G(q)
fprintf('\n→ Computing gravity vector G(q)...\n');
G_sym = sym(zeros(6, 1));

for i = 1:6
    % G_i = ∂PE/∂q_i
    G_sym(i) = diff(PE, q_sym(i));
end

G_sym = simplify(G_sym);
fprintf('  ✓ Gravity vector G(q) derived [6x1]\n');

%% Generate MATLAB Functions
fprintf('\n→ Generating optimized MATLAB functions...\n');

% Mass matrix M(q)
fprintf('  → Generating SmartSix_M_sym.m...\n');
matlabFunction(M_sym, 'File', fullfile(output_dir, 'SmartSix_M_sym'), ...
               'Vars', {q_sym}, ...
               'Outputs', {'M'}, ...
               'Optimize', true, ...
               'Comments', 'Exact mass matrix from URDF inertial parameters');

% Coriolis vector C(q, q_dot)
fprintf('  → Generating SmartSix_C_sym.m...\n');
matlabFunction(C_sym, 'File', fullfile(output_dir, 'SmartSix_C_sym'), ...
               'Vars', {q_sym, qd_sym}, ...
               'Outputs', {'C'}, ...
               'Optimize', true, ...
               'Comments', 'Exact Coriolis/centrifugal vector from URDF');

% Gravity vector G(q)
fprintf('  → Generating SmartSix_G_sym.m...\n');
matlabFunction(G_sym, 'File', fullfile(output_dir, 'SmartSix_G_sym'), ...
               'Vars', {q_sym, g_sym}, ...
               'Outputs', {'G'}, ...
               'Optimize', true, ...
               'Comments', 'Exact gravity vector from URDF');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                  Generation Complete!                         ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('Generated files:\n');
fprintf('  ✓ %s\n', fullfile(output_dir, 'SmartSix_M_sym.m'));
fprintf('  ✓ %s\n', fullfile(output_dir, 'SmartSix_C_sym.m'));
fprintf('  ✓ %s\n', fullfile(output_dir, 'SmartSix_G_sym.m'));
fprintf('\nThese functions are compatible with CasADi MX symbolic variables.\n');

%% Helper Function: DH Transformation
function T = dh_transform(a, alpha, d, theta)
    % DH transformation matrix
    % Modified DH convention: T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),              cos(alpha),            d;
         0,           0,                        0,                     1];
end
