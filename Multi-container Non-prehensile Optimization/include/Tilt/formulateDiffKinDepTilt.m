%% formulateDiffKinDepTilt.m
%--------------------------------------------------------------------------
% This section builds the symbolic and CasADi functions needed to compute:
% - Joint values q
% - Joint velocities q_dot
% - Geometric Jacobian Jg
%
% Output:
% CasADi function handles: 
%   - get_q
%   - get_q_dot
%   - get_Jg
%--------------------------------------------------------------------------

s = casadi.MX.sym('s',1);
s_dot = casadi.MX.sym('s_dot',1);
s_ddot = casadi.MX.sym('s_ddot',1);

thx = casadi.MX.sym('thx',1);
thy = casadi.MX.sym('thy',1);
thx_dot = casadi.MX.sym('thx_dot',1);
thy_dot = casadi.MX.sym('thy_dot',1);
thx_ddot = casadi.MX.sym('thx_ddot',1);
thy_ddot = casadi.MX.sym('thy_ddot',1);

T06 = get_T06(s,thx,thy);
pos = get_p_06_0(s,thx,thy);

v = get_v_06_0(s,s_dot,thx,thy,thx_dot,thy_dot);
w = get_w_0(s,s_dot,thx,thy,thx_dot,thy_dot);

%% q
% q = SmartSix_IK_sym(robot,T06,0);
q = robot.ik_sym(T06,0);
get_q = casadi.Function('get_q',{s,thx,thy},{q});
q_eval = get_q(s,thx,thy);

%% q_dot
% Jg = SmartSix_Jg_sym(q_eval);
Jg = robot.Jg_sym(q_eval);
get_Jg = casadi.Function('get_Jg',{s,thx,thy},{Jg});

alpha = 1e-3;
Jac_Matrix_Ps_Inv = (Jg'*Jg + alpha* eye(6))^(-1)*Jg' ;

q_dot = Jg\[v; w];
% q_dot = Jac_Matrix_Ps_Inv*[v; w];

get_q_dot = casadi.Function('get_q_dot', {s,s_dot,thx,thy,thx_dot,thy_dot},{q_dot});
