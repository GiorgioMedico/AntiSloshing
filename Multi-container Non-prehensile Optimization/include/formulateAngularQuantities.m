%% formulateAngularQuantities.m
%--------------------------------------------------------------------------
% This section builds the symbolic and CasADi functions needed to compute:
% - Rotation matrix Rz(φ)
% - Angular velocity
% - Angular acceleration
%
% Prerequisite:
% The functions used to compute the angle and its derivatives need to be
% Workspace variables
%
% Output:
% CasADi function handles: 
%   - get_R07
%   - get_w_07_0
%   - get_w_07_0_dot
%
% Note:
% Additionally the derivatives of the rotation matrices are computed
%--------------------------------------------------------------------------

s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('s_dot', 1);
s_ddot = casadi.MX.sym('s_ddot', 1);

thz = get_thz_0(s); 
thz_dot = get_thz_dot_0(s,s_dot);
thz_ddot = get_thz_ddot_0(s,s_dot, s_ddot);

%% Rotation Matrix
R07 = Rz(thz);
get_R07 = casadi.Function('get_R07', {s},{R07});

%% Angular Velocity
w07 = [0; 0; thz_dot];
get_w07_0 = casadi.Function('get_w07', {s, s_dot},{w07});

% Rotation matrix derivative
R07_dot = skew(w07)*R07;    % from w = R_dot*R'
get_R07_dot = casadi.Function('get_R07_dot', {s, s_dot},{R07_dot});

%% Angular Acceleration
w07_dot = [0; 0; thz_ddot];
get_w07_0_dot = casadi.Function('get_w07_dot', {s, s_dot, s_ddot},{w07_dot});

% Rotation matrix second derivative
R07_ddot = skew(w07_dot)*R07 + skew(w07)*R07_dot;
get_R07_ddot = casadi.Function('get_R07_ddot', {s, s_dot, s_ddot},{R07_ddot});