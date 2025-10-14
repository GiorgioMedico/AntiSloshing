%% mapCasadiFunctions.m
% This script maps all relevant CasADi functions over the time horizon (N+1),
% used for constraint and dynamics evaluations. 
% Each mapping is performed only if the corresponding function exists in the workspace.

% Time horizon assumed defined in the main script
assert(exist('N', 'var') == 1, 'Variable N must be defined before running this script.');

% End-effector
if exist('get_p_06_0', 'var')
    get_p_06_0_map = get_p_06_0.map(N+1);
    p06 = get_p_06_0_map(s_o);
end
if exist('get_R06', 'var')
    get_R06_map = get_R06.map(N+1);
    R06 = get_R06_map(s_o);
end
if exist('get_v_06_0', 'var')
    get_v_06_0_map = get_v_06_0.map(N+1);
    v06 = get_v_06_0_map(s_o, s_dot_o);
end
if exist('get_a_06_0', 'var')
    get_a_06_0_map = get_a_06_0.map(N+1);
    a06 = get_a_06_0_map(s_o, s_dot_o, s_ddot_o);
end
if exist('get_w06_0', 'var')
    get_w06_0_map = get_w06_0.map(N+1);
    w06_0 = get_w06_0_map(s_o, s_dot_o);
end
if exist('get_w06_0_dot', 'var')
    get_w06_0_dot_map = get_w06_0_dot.map(N+1);
    w06_0_dot = get_w06_0_dot_map(s_o, s_dot_o, s_ddot_o);
end

% Tray-related
if exist('get_p_07_0', 'var')
    get_p_07_0_map = get_p_07_0.map(N+1);
    p07 = get_p_07_0_map(s_o);
end
if exist('get_R07', 'var')
    get_R07_map = get_R07.map(N+1);
    R07 = get_R07_map(s_o);
end
if exist('get_v_07_0', 'var')
    get_v_07_0_map = get_v_07_0.map(N+1);
    v07 = get_v_07_0_map(s_o, s_dot_o);
end
if exist('get_a_07_0', 'var')
    get_a_07_0_map = get_a_07_0.map(N+1);
    a07 = get_a_07_0_map(s_o, s_dot_o, s_ddot_o);
end
if exist('get_w07_0', 'var')
    get_w07_0_map = get_w07_0.map(N+1);
    w07 = get_w07_0_map(s_o, s_dot_o);
end
if exist('get_w07_0_dot', 'var')
    get_w07_0_dot_map = get_w07_0_dot.map(N+1);
    w07_dot = get_w07_0_dot_map(s_o, s_dot_o, s_ddot_o);
end

% Orientation variables
if exist('get_thz_0', 'var')
    get_thz_0_map = get_thz_0.map(N+1);
    thz = get_thz_0_map(s_o);
end
if exist('get_thz_dot_0', 'var')
    get_thz_dot_0_map = get_thz_dot_0.map(N+1);
    thzd = get_thz_dot_0_map(s_o, s_dot_o);
end
if exist('get_thz_ddot_0', 'var')
    get_thz_ddot_0_map = get_thz_ddot_0.map(N+1);
    thzdd = get_thz_ddot_0_map(s_o, s_dot_o, s_ddot_o);
end

% Joint-related
if exist('get_q_dot', 'var')
    get_q_dot_map = get_q_dot.map(N+1);
    q_dot = get_q_dot_map(s_o, s_dot_o);
end


