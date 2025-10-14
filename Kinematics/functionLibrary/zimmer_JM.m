% Creata da Claudio Mazzotti

function qc = zimmer_JM(start_value, finish_value, time, time_step)


t = [0:time_step:time]';
% qc = value*ones(1,length(t));
qc = lspb(start_value,finish_value,t);