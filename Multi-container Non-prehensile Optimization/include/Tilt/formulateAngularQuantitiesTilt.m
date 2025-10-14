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

thx = casadi.MX.sym('thx',1);
thy = casadi.MX.sym('thy',1);
thx_dot = casadi.MX.sym('thx_dot',1);
thy_dot = casadi.MX.sym('thy_dot',1);
thx_ddot = casadi.MX.sym('thx_ddot',1);
thy_ddot = casadi.MX.sym('thy_ddot',1);

%% Rotation Matrix
% R07 = Rx(thx)*Ry(thy)*Rz(thz);
R07 = Rz(thz)*Ry(thy)*Rx(thx);
get_R07 = casadi.Function('get_R07', {s,thx,thy},{R07});

%% Angular Velocity
% w1 = [thx_dot; 0; 0];
% w2 = Rx(thx)*[0; thy_dot; 0];
% w3 = Rx(thx)*Ry(thy)*[0; 0; thz_dot];
w1 = [0; 0; thz_dot];
w2 = Rz(thz)*[0; thy_dot; 0];
w3 = Rz(thz)*Ry(thy)*[thx_dot; 0; 0;];

w07 = w1 + w2 + w3;

get_w07_0 = casadi.Function('get_w07', {s, s_dot, thx,thy,thx_dot,thy_dot},{w07});

% Rotation matrix derivative
R07_dot = skew(w07)*R07;    % from w = R_dot*R'
get_R07_dot = casadi.Function('get_R07_dot', {s, s_dot, thx,thy,thx_dot,thy_dot},{R07_dot});


%% Angular Acceleration
% a1 = [thx_ddot; 0; 0];
% a2 = Rx(thx)*[0; thy_ddot; 0];
% a3 = Rx(thx)*Ry(thy)*[0; 0; thz_ddot];
a1 = [0; 0; thz_ddot;];
a2 = Rz(thz)*[0; thy_ddot; 0];
a3 = Rz(thz)*Ry(thy)*[thx_ddot; 0; 0;];

w07_dot = a1 + a2 + a3 + skew(w1)*w2 + skew(w1+w2)*w3;
get_w07_0_dot = casadi.Function('get_w07_dot', {s, s_dot, s_ddot, thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot},{w07_dot});

% Rotation matrix second derivative
R07_ddot = skew(w07_dot)*R07 + skew(w07)*R07_dot;
get_R07_ddot = casadi.Function('get_R07_ddot', {s, s_dot, s_ddot,  thx,thy,thx_dot,thy_dot,thx_ddot,thy_ddot},{R07_ddot});