%% formulateDiffKinDep.m
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

T06 = get_T06(s);
pos = get_p_06_0(s);

v = get_v_06_0(s, s_dot);
w = get_w_0(s,s_dot);

%% q
% q = SmartSix_IK_sym(robot,T06,0);
q = robot.ik_sym(T06,0);

get_q = casadi.Function('get_q',{s},{q});
q_eval = get_q(s);

%% q_dot
% Jg = SmartSix_Jg_sym(q_eval);
Jg = robot.Jg_sym(q_eval);
get_Jg = casadi.Function('get_Jg',{s},{Jg});

alpha = 1e-3;
Jac_Matrix_Ps_Inv = (Jg'*Jg + alpha* eye(6))^(-1)*Jg' ;

q_dot = Jg\[v; w];
% q_dot = Jac_Matrix_Ps_Inv*[v; w];

get_q_dot = casadi.Function('get_q_dot', {s,s_dot},{q_dot});
