%% formulateAngularTrajectory.m
%--------------------------------------------------------------------------
% This section builds the symbolic and CasADi functions needed to compute:
% - Angle φ(s)
% - Angle derivative φ̇ (s, ṡ) = φ'(s)·ṡ
% - Angle second derivative φ̈ (s, ṡ, s̈) = φ''(s)·ṡ² + φ'(s)·s̈
%
% Prerequisite:
% The final angle value needs to be a Workspace variable
%
% Output:
% CasADi function handles: 
%   - get_thz_0:        φ(s)
%   - get_thz_dot_0:    φ̇ (s, ṡ)
%   - get_thz_ddot_0:   φ̈ (s, ṡ, s̈)
%
% Note:
% The angle changes linearly with s
%--------------------------------------------------------------------------


%% ------------------ Trajectory: φ(s) -----------------------------
s = casadi.MX.sym('s',1);
% phi = s*thz_end; % Linear 
phi = thz_0 + s*(thz_end - thz_0);
get_thz_0 = casadi.Function('get_thz_0',{s},{phi});

%% ------------------ First Derivative: φ'(s) -----------------------------
%-----------------Path first derivative phi'(s) = d/ds phi(s)-----------------
% Define the symbolic variable for the parameter s
s = casadi.MX.sym('s');

% Calculate the first derivative of the 3D path spline
phi_s = get_thz_0(s); % Evaluate the 3D path spline at s
phi_ds = jacobian(phi_s, s); % Compute the first derivative with respect to s

TrayRotzPath_ds = casadi.Function('TrayRotzPath_ds', {s}, {phi_ds}); % Define a CasADi function for the first derivative

%% ------------------ Second Derivative: φ''(s) ---------------------------
%------------------Path second derivative phi''(s) = d/ds phi'(s)--------------------
s = casadi.MX.sym('s');

% Calculate the second derivative of the 3D path spline
phi_ds = TrayRotzPath_ds(s); % Evaluate the first derivative at s
phi_dds = jacobian(phi_ds, s); % Compute the second derivative with respect to s

TrayRotzPath_dds = casadi.Function('TrayRotzPath_dds', {s}, {phi_dds}); % Define a CasADi function for the second derivative

%-------------------------------------------------------------------------

%% ------------------------ Velocity φ̇ (s, ṡ) -----------------------------
%------------------- Velocity phi_dot(s,s_dot) = phi'(s)*s_dot) -----------------------
s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('ds', 1);
phi_ds = TrayRotzPath_ds(s);
v = phi_ds * s_dot;

% in world reference frame
get_thz_dot_0 = casadi.Function('get_thz_dot_0', {s,s_dot},{v});

%% ---------------------- Acceleration φ̈ (s, ṡ, s̈) ------------------------
%--------Acceleration (phi_ddot(s,s_dot,s_ddot) = phi'(s)*s_ddot + phi''(s)*s_dot^2)--------
s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('s_dot', 1);
s_ddot = casadi.MX.sym('s_ddot', 1);
phi_ds = TrayRotzPath_ds(s);
phi_dds = TrayRotzPath_dds(s);
a = phi_dds*s_dot^2 + phi_ds*s_ddot;

% in world reference frame
get_thz_ddot_0 = casadi.Function('get_thz_ddot_0', {s,s_dot,s_ddot},{a});
