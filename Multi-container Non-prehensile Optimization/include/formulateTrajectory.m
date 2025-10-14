%% formulateTrajectory.m
%--------------------------------------------------------------------------
% This script defines symbolic and CasADi functions for computing the
% trajectoy, its derivatives and kinematic quantities  
% (velocity, acceleration) along a 3D path phi(s), where:
% - s is the path parameter (typically normalized in [0,1])
% - phi(s) is the Cartesian trajectory in 3D space
% This section builds the symbolic and CasADi functions needed to compute:
% - First spatial derivative φ'(s) for direction/velocity
% - Second spatial derivative φ''(s) for curvature/acceleration
% - Velocity φ̇ (s, ṡ) = φ'(s)·ṡ
% - Acceleration φ̈ (s, ṡ, s̈) = φ''(s)·ṡ² + φ'(s)·s̈
%
% Prerequisite:
% The control points cpx, cpy, cpz and their number n_ctrl_pts must be 
% already defined as Workspace variables
%
% Output:
% CasADi function handles: 
%   - get_p_07_0:     φ(s)
%   - TrayPath_ds:    φ'(s)
%   - TrayPath_dds:   φ''(s)
%   - get_v_07_0:     φ̇ (s, ṡ)
%   - get_a_07_0:     φ̈ (s, ṡ, s̈)
%--------------------------------------------------------------------------


%% ------------------ Trajectory: φ(s) -----------------------------

sctrl_pts = linspace(0,1,n_ctrl_pts); %Parametric values of the control points, evenly spaced between 0 and 1

ctrl_pts = [cpx; cpy; cpz];
dim_spline = 3;
degree = 4; % 4th-degree polynomial basis for the spline
n_knots = n_ctrl_pts+degree +1;

% Knot vector for the B-spline
knots = [zeros(1, degree) - 1e-3, linspace(0, 1, n_knots-2*degree), ones(1, degree) + 1e-3]; 
% Degree number of zero knots at the beginning, slightly offset by -1e-3 to avoid numerical issues.
% Linearly spaced knots in the middle.
% Degree number of one knots at the end, slightly offset by +1e-3

% -------------------- Trajectory ------------------------
% pathSpline3D spline trajectory as a function of a parametric variable
get_p_07_0 = casadi.Function.bspline('evaluateTrayPathSpline', {knots}, ctrl_pts(:), {degree}, dim_spline);


%% ------------------ First Derivative: φ'(s) -----------------------------
%-----------------Path first derivative phi'(s) = d/ds phi(s)--------------
% Define the symbolic variable for the parameter s
s = casadi.MX.sym('s');

% Calculate the first derivative of the 3D path spline
phi_s = get_p_07_0(s); % Evaluate the 3D path spline at s
phi_ds = jacobian(phi_s, s); % Compute the first derivative with respect to s

TrayPath_ds = casadi.Function('evaluateTrayPathDerivative', {s}, {phi_ds}); % Define a CasADi function for the first derivative

%% ------------------ Second Derivative: φ''(s) ---------------------------
%------------------Path second derivative phi''(s) = d/ds phi'(s)----------
s = casadi.MX.sym('s');

% Calculate the second derivative of the 3D path spline
phi_ds = TrayPath_ds(s); % Evaluate the first derivative at s
phi_dds = jacobian(phi_ds, s); % Compute the second derivative with respect to s

TrayPath_dds = casadi.Function('evaluateTrayPathSecondDerivative', {s}, {phi_dds}); % Define a CasADi function for the second derivative

%-------------------------------------------------------------------------

%% ------------------------ Velocity φ̇ (s, ṡ) -----------------------------
%------------------- Velocity phi_dot(s,s_dot) = phi'(s)*s_dot) -----------
s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('ds', 1);
phi_ds = TrayPath_ds(s);
v = phi_ds * s_dot;

% in world reference frame
get_v_07_0 = casadi.Function('getTrayVel', {s,s_dot},{v});

%% ---------------------- Acceleration φ̈ (s, ṡ, s̈) ------------------------
%--------Acceleration (phi_ddot(s,s_dot,s_ddot) = phi'(s)*s_ddot + phi''(s)*s_dot^2)--------
s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('s_dot', 1);
s_ddot = casadi.MX.sym('s_ddot', 1);
phi_ds = TrayPath_ds(s);
phi_dds = TrayPath_dds(s);
a = phi_dds*s_dot^2 + phi_ds*s_ddot;

% in world reference frame
get_a_07_0 = casadi.Function('getTrayAcc', {s,s_dot,s_ddot},{a});