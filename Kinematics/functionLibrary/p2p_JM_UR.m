% Creata da Claudio Mazzotti
% Modificata per UR da Roberto Di Leva

function qc = p2p_JM_UR(P1, P2, time, time_step, status)

q0 = P1.J;
qf = P2.J;

t = [0:time_step:time]';
% qc = jtraj(P1, P2, t);
[qc,qc_D,qc_DD] = mtraj(@lspb,q0,qf,t);