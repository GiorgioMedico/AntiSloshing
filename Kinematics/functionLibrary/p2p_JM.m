% Creata da Claudio Mazzotti

function qc = p2p_JM(P1, P2, time, time_step)


t = [0:time_step:time]';
% qc = jtraj(P1, P2, t);
[qc,qc_D,qc_DD] = mtraj(@lspb,P1,P2,t);
