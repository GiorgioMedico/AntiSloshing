%% formulateEndEffectorKinematicsTilt.m
% -------------------------------------------------------------------------
% This script defines the symbolic expressions and CasADi functions for:
% - Position of the end-effector (frame 6) in base frame (frame 0)
% - Velocity and angular velocity of the EE
% - Acceleration and angular acceleration of the EE
%
% The EE is computed starting from the tray frame (frame 7), assuming the
% transformation from frame 6 to 7 (T67) and vector from 6 to 7 (p_67_6) 
% are known in the workspace.
%
% Dependencies (must be in workspace):
% - get_R07(s,thx,thy), get_p_07_0(s), get_v_07_0(s,s_dot), get_a_07_0(s,s_dot,s_ddot)
% - get_w07_0(s,s_dot,thx,thy,thx_dot,thy_dot), get_w07_0_dot(s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot)
% - R67, T67, p_67_6
%
% Output:
% - CasADi functions: get_T07, get_R06, get_T06, get_p_06,
%                     get_v_06, get_w06, get_a_06, get_w, get_w_dot
% -------------------------------------------------------------------------

% Create symbolic variables
s       = casadi.MX.sym('s', 1);         % position
s_dot   = casadi.MX.sym('s_dot', 1);     % velocity
s_ddot  = casadi.MX.sym('s_ddot', 1);    % acceleration

thx = casadi.MX.sym('thx',1);
thy = casadi.MX.sym('thy', 1);
thz = casadi.MX.sym('thz', 1);

%% ------------------------------------------------------------------------
% TRANSFORMATIONS AND POSITION (Frame 0)

% Transformation from base (0) to tray (7)
R07 = get_R07(s,thx,thy);
R07_dot = get_R07_dot(s,s_dot, thx,thy,thx_dot,thy_dot);
R07_ddot = get_R07_ddot(s,s_dot,s_ddot, thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot);

p_07_0 = get_p_07_0(s);
T07 = [R07, p_07_0; 0 0 0 1];
get_T07 = casadi.Function('get_T07', {s,thx,thy}, {T07});

% Rotation from base to EE (6)
R06 = R07 * R67';
get_R06 = casadi.Function('get_R06', {s,thx,thy}, {R06});

% Full transform from base to EE
T06 = T07 * inv(T67);
get_T06 = casadi.Function('get_T06', {s,thx,thy}, {T06});

% Position of EE in frame 0
% p_06_0 = T06(1:3, 4);
p_06_0 = p_07_0 - R06*p_67_6;
get_p_06_0 = casadi.Function('get_p_06', {s,thx,thy}, {p_06_0});

%% ------------------------------------------------------------------------
% VELOCITY (Frame 0)

% Angular velocity of EE (same as tray's angular velocity)
w06_0 = get_w07_0(s,s_dot,thx,thy,thx_dot,thy_dot);
get_w06_0 = casadi.Function('get_w06', {s,s_dot,thx,thy,thx_dot,thy_dot}, {w06_0});

R06_dot = R07_dot * R67';
get_R06_dot = casadi.Function('get_R06_dot', {s, s_dot, thx,thy,thx_dot,thy_dot}, {R06_dot});

% Linear velocity of EE using tray velocity and rotational offset
v_07_0 = get_v_07_0(s, s_dot);
v_06_0 = v_07_0 - skew(w06_0) * (get_R06(s,thx,thy) * p_67_6);
% v_06_0 = v_07_0 - R06_dot*p_67_6;
get_v_06_0 = casadi.Function('get_v_06', {s,s_dot,thx,thy,thx_dot,thy_dot}, {v_06_0});

%% ------------------------------------------------------------------------
% ACCELERATION (Frame 0)

% Angular acceleration of EE
w06_0_dot = get_w07_0_dot(s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot);
get_w06_0_dot = casadi.Function('get_w06_dot', {s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {w06_0_dot});

R06_ddot = R07_ddot * R67';
get_R06_ddot = casadi.Function('get_R06_ddot', {s, s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {R06_ddot});

% Linear acceleration of EE
a_07_0 = get_a_07_0(s, s_dot, s_ddot);
p_offset = get_R06(s,thx,thy) * p_67_6;
a_06_0 = a_07_0 ...
         - skew(w06_0_dot) * p_offset ...
         - skew(w06_0) * (skew(w06_0) * p_offset);
% a_06_0 = a_07_0 - R06_ddot*p_67_6;
get_a_06_0 = casadi.Function('get_a_06', {s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {a_06_0});

%% ------------------------------------------------------------------------
% Optional duplicate naming for general angular quantities
get_w_0 = casadi.Function('get_w', {s,s_dot,thx,thy,thx_dot,thy_dot}, {w06_0});
get_w_dot_0 = casadi.Function('get_w_dot', {s,s_dot,s_ddot,thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot}, {w06_0_dot});
