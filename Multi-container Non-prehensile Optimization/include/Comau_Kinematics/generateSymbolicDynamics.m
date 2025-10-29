%% generateSymbolicDynamics_fast.m
% Generates exact symbolic dynamics functions for SmartSix robot from URDF
%
% Faster pipeline:
%   - M(q) via Jacobians of link COMs (no Lagrangian double-diff)
%   - G(q) as gradient of potential energy
%   - C(q, qdot) via Christoffel symbols using precomputed dM/dq
%
% Output files (auto-generated in this folder):
%   - SmartSix_M_sym.m
%   - SmartSix_C_sym.m
%   - SmartSix_G_sym.m
%
% Author: Giorgio Medico
% Date: October 2025

function generateSymbolicDynamics()
clear all
close all
clear classes
clc

%% Configuration
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║   SmartSix Symbolic Dynamics (FAST) from URDF & DH            ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

% Output directory for generated functions
output_dir = fileparts(mfilename('fullpath'));
urdf_path  = fullfile(output_dir, 'URDF', 'comau_smartsix5.urdf');

% Check toolboxes
if ~license('test', 'Symbolic_Toolbox')
    error('Symbolic Math Toolbox is required but not available.');
end
if ~license('test', 'Robotics_System_Toolbox')
    warning('Robotics System Toolbox not detected by license(). If importrobot fails, check installation.');
end

%% Load Robot Model (URDF) for inertial parameters
fprintf('→ Loading URDF from: %s\n', urdf_path);
if ~isfile(urdf_path)
    error('URDF file not found: %s', urdf_path);
end

robot_rbt = importrobot(urdf_path);
robot_rbt.DataFormat = 'row';
fprintf('  ✓ Loaded %d links, %d joints\n', numel(robot_rbt.Bodies), numel(robot_rbt.Bodies));

%% Create SmartSix Robot Object for DH Parameters
robot = SmartSix(); % must be on MATLAB path
fprintf('  ✓ SmartSix DH parameters loaded\n');

%% Extract Inertial Parameters from URDF (axes_1..axes_6)
link_names = {'axes_1', 'axes_2', 'axes_3', 'axes_4', 'axes_5', 'axes_6'};
n_links    = length(link_names);

mass    = zeros(n_links, 1);
com     = zeros(n_links, 3);    % Center of mass in link frame
inertia = cell(n_links, 1);     % 6-vector [Ixx Iyy Izz Iyz Ixz Ixy] in link frame

fprintf('\n→ Extracting inertial parameters:\n');
for i = 1:n_links
    body = robot_rbt.getBody(link_names{i});
    mass(i)      = body.Mass;
    com(i, :)    = body.CenterOfMass;     % [x y z] in link frame
    inertia{i}   = body.Inertia(:).';     % [Ixx Ixy Ixz Iyy Iyz Izz]
    fprintf('  Link %d (%s): m=%.3f kg, COM=[%.4f %.4f %.4f]\n', ...
            i, link_names{i}, mass(i), com(i,1), com(i,2), com(i,3));
end

%% Symbolic Variables
fprintf('\n→ Creating symbolic variables...\n');
syms q1 q2 q3 q4 q5 q6 real
syms qd1 qd2 qd3 qd4 qd5 qd6 real
syms g_sym real  % Gravity acceleration (+Z)

q_sym  = [q1; q2; q3; q4; q5; q6];
qd_sym = [qd1; qd2; qd3; qd4; qd5; qd6];

assumeAlso(q_sym, 'real');
assumeAlso(qd_sym, 'real');
assumeAlso(g_sym, 'real');
fprintf('  ✓ Symbolic joint variables created\n');

%% Forward Kinematics via DH (Modified DH as in original script)
fprintf('\n→ Deriving symbolic forward kinematics (DH)...\n');

a     = robot.a(:);
d     = robot.d(:);
alpha = robot.alpha(:);

T = cell(n_links + 1, 1);
T{1} = sym(eye(4));               % Base frame (symbolic)

for i = 1:n_links
    fprintf('  Computing T_0_%d... ', i);
    T_i   = dh_transform(a(i), alpha(i), d(i), q_sym(i));
    % keep expressions relatively small during accumulation
    T{i+1} = simplify(T{i} * T_i, 'Steps', 3);
    fprintf('✓\n');
end

% Origins and rotations
p = cell(n_links + 1, 1);
R = cell(n_links + 1, 1);
z = cell(n_links + 1, 1);

p{1} = sym([0;0;0]);
R{1} = sym(eye(3));
z{1} = sym([0;0;1]);  % base z

for i = 1:n_links
    R{i+1} = T{i+1}(1:3,1:3);
    p{i+1} = T{i+1}(1:3,4);
    z{i+1} = R{i+1}(:,3);
end
fprintf('  ✓ Extracted R_0_i (rotation), p_0_i (position), z_0_i (joint axes) for all links\n');

%% Jacobians of COMs (geometric)
fprintf('\n→ Building COM Jacobians via symbolic differentiation...\n');

% COM positions in base
fprintf('  Computing COM positions in base frame...\n');
p_com = cell(n_links,1);
for i = 1:n_links
    fprintf('    Link %d COM... ', i);
    p_com_local  = [com(i, :)'; 1];
    p_com_global = T{i+1} * p_com_local;
    p_com{i}     = p_com_global(1:3);
    fprintf('✓\n');
end

% Linear Jacobians Jv{i} (3x6: 3 Cartesian velocities × 6 joints)
% For serial chain: columns j>i are automatically zero (joint j doesn't affect link i if j>i)
fprintf('  Computing linear Jacobians Jv (velocity propagation)...\n');
Jv = cell(n_links,1);
for i = 1:n_links
    fprintf('    Jv_%d... ', i);
    Jv{i} = jacobian(p_com{i}, q_sym);  % Already 3×6 (dp/dq for position w.r.t. joints)
    fprintf('✓\n');
end

% Angular Jacobians Jw{i} (3x6): for revolute joints, columns are z_j for j<=i
fprintf('  Computing angular Jacobians Jw (rotation propagation)...\n');
Jw = cell(n_links,1);
for i = 1:n_links
    fprintf('    Jw_%d... ', i);
    Jw_i = sym(zeros(3,6));
    for j = 1:i
        Jw_i(:, j) = z{j}; % joint axis j in base
    end
    Jw{i} = Jw_i;
    fprintf('✓\n');
end
fprintf('  ✓ All Jacobians (Jv and Jw) computed successfully\n');

%% Mass matrix M(q) via Jacobians
fprintf('\n→ Computing M(q) via Jacobian composition...\n');
M_sym = sym(zeros(6,6));
fprintf('  Adding link contributions to M(q)...\n');
for i = 1:n_links
    fprintf('    Link %d (m=%.3f kg): translational + rotational inertia... ', i, mass(i));
    % Inertia tensor in link frame (3x3) from URDF format [Ixx Ixy Ixz Iyy Iyz Izz]
    I_local = [inertia{i}(1), inertia{i}(2), inertia{i}(3);
               inertia{i}(2), inertia{i}(4), inertia{i}(5);
               inertia{i}(3), inertia{i}(5), inertia{i}(6)];
    % Rotate to base
    R_i     = R{i+1};
    I_global = R_i * I_local * R_i.';
    % Sum contributions
    M_sym = M_sym + (Jv{i}.' * (mass(i) * Jv{i})) + (Jw{i}.' * I_global * Jw{i});
    fprintf('✓\n');
end
fprintf('  ✓ M(q) [6x6] complete\n');

%% Potential energy & Gravity vector
fprintf('\n→ Computing potential energy and G(q)...\n');
PE = sym(0);
fprintf('  Computing potential energy contributions...\n');
for i = 1:n_links
    fprintf('    Link %d: m*g*h... ', i);
    PE = PE + mass(i) * g_sym * p_com{i}(3);  % z-height times mass*g
    fprintf('✓\n');
end
fprintf('  Computing gradient ∂PE/∂q to get G(q)...\n');
G_sym = jacobian(PE, q_sym).';
fprintf('  ✓ G(q) [6x1] complete\n');

%% Coriolis / Centrifugal vector via Christoffel (using dM/dq)
fprintf('\n→ Computing Coriolis/centrifugal vector using dM/dq...\n');

% Precompute dM/dq_k, k=1..6
fprintf('  Computing ∂M/∂q_k for all joints...\n');
dM_dq = cell(6,1);
for k = 1:6
    fprintf('    ∂M/∂q_%d... ', k);
    dM_dq{k} = diff(M_sym, q_sym(k));
    fprintf('✓\n');
end

% Build C(q, qdot) = C_mat(q, qdot) * qdot
% Christoffel symbols: Γ_ij^k = 0.5 * (∂M_kj/∂q_i + ∂M_ki/∂q_j - ∂M_ij/∂q_k)
% Coriolis matrix element: C_mat(i,j) = Σ_k Γ_ij^k * q̇_k
% Final Coriolis vector: C = C_mat * q̇
fprintf('  Computing Christoffel symbols and Coriolis matrix C_mat...\n');
C_mat = sym(zeros(6,6));
for i = 1:6
    fprintf('    Row %d/6... ', i);
    for j = 1:6
        s = sym(0);
        for k = 1:6
            % Christoffel symbol construction: (∂M/∂q_k)(i,j) + (∂M/∂q_j)(i,k) - (∂M/∂q_i)(k,j)
            s = s + 0.5 * ( dM_dq{k}(i,j) + dM_dq{j}(i,k) - dM_dq{i}(k,j) ) * qd_sym(k);
        end
        C_mat(i,j) = s;
    end
    fprintf('✓\n');
end
fprintf('  Computing C(q,q̇) = C_mat * q̇...\n');
C_sym = C_mat * qd_sym;
fprintf('  ✓ C(q, q̇) [6x1] complete\n');

%% Generate MATLAB functions
fprintf('\n→ Generating optimized MATLAB functions...\n');

% Mass matrix M(q)
fprintf('  → Generating SmartSix_M_sym.m [M(q): 6x6 configuration-dependent inertia]... ');
matlabFunction(M_sym, 'File', fullfile(output_dir, 'SmartSix_M_sym'), ...
               'Vars', {q_sym}, ...
               'Outputs', {'M'}, ...
               'Optimize', true, ...
               'Sparse', false);
fprintf('✓\n');

% Coriolis vector C(q, q_dot)
fprintf('  → Generating SmartSix_C_sym.m [C(q,q̇): 6x1 velocity-dependent forces]... ');
matlabFunction(C_sym, 'File', fullfile(output_dir, 'SmartSix_C_sym'), ...
               'Vars', {q_sym, qd_sym}, ...
               'Outputs', {'C'}, ...
               'Optimize', true, ...
               'Sparse', false);
fprintf('✓\n');

% Gravity vector G(q)
fprintf('  → Generating SmartSix_G_sym.m [G(q): 6x1 gravitational torques]... ');
matlabFunction(G_sym, 'File', fullfile(output_dir, 'SmartSix_G_sym'), ...
               'Vars', {q_sym, g_sym}, ...
               'Outputs', {'G'}, ...
               'Optimize', true, ...
               'Sparse', false);
fprintf('✓\n');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                     Generation Complete!                      ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('Generated files (saved in: %s):\n', output_dir);
fprintf('  ✓ SmartSix_M_sym.m  - Mass/inertia matrix M(q) [6x6]\n');
fprintf('  ✓ SmartSix_C_sym.m  - Coriolis/centrifugal vector C(q,q̇) [6x1]\n');
fprintf('  ✓ SmartSix_G_sym.m  - Gravity torque vector G(q,g) [6x1]\n');
fprintf('\nUsage:\n');
fprintf('  M = SmartSix_M_sym(q);          %% q: [6x1] joint angles\n');
fprintf('  C = SmartSix_C_sym(q, qd);      %% qd: [6x1] joint velocities\n');
fprintf('  G = SmartSix_G_sym(q, g);       %% g: scalar gravity (9.81 m/s²)\n');
fprintf('\nThese functions are compatible with CasADi MX symbolic variables.\n');

end % function main

%% Helper: Modified DH Transformation
function T = dh_transform(a, alpha, d, theta)
    % Modified DH: Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    T = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
end
