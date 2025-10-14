% Creata da Claudio Mazzotti
% Modificata per UR da Roberto Di Leva

% lineare Joint Motion.
% è un movimento lineare dalla posa T1 alla T2.

function [qc,motion_status,t] = lin_JM_UR(T1,T2,time,time_step,robot,status)

t = [0:time_step:time]';
TC = ctraj(T1, T2, length(t));
qc = [];
motion_status = [];

SOLTH = inverseKinematicUR(robot,T1,status);
qc_start = SOLTH;
qc = [qc;qc_start];
motion_status(1) = status;
status_richiesto = motion_status(1);

for i = 2:size(TC,3)
    rrs = inverseKinematicUR(robot,TC(:,:,i),status);
    find(status==status_richiesto);
    
%     if isempty(find(status==status_richiesto,1))
% %         disp(['moto non eseguibile'])
% %         qc = [qc;rrs(find(status==status_richiesto,1),:)];
%           qc = [qc;NaN*zeros(1,6);];
%         
%     else
        qc = [qc;rrs];
%         motion_status(i) = status(find(status==status_richiesto,1));
%     end
    
end
