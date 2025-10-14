% Creata da Claudio Mazzotti

% lineare Joint Motion.
% è un movimento lineare dalla posa T1 alla T2.

function [qc,motion_status] = lin_JM(T1,T2,time,time_step,robot,Redundancy,status_richiesto)

t = [0:time_step:time]';
TC = ctraj(T1, T2, length(t));
qc = [];
motion_status = [];

qc_start = inverseKinematicLBR(robot,TC(:,:,1),Redundancy,status_richiesto);
qc = [qc;qc_start];
motion_status(1) = status_richiesto;

for i = 2:size(TC,3)
    [rrs,status] = inverseKinematicLBR(robot,TC(:,:,i),Redundancy);
    find(status==status_richiesto);
    
    if isempty(find(status==status_richiesto,1))
%         disp(['moto non eseguibile'])
%         qc = [qc;rrs(find(status==status_richiesto,1),:)];
          qc = [qc;NaN*zeros(1,7);];
        
    else
        qc = [qc;rrs(find(status==status_richiesto,1),:)];
        motion_status(i) = status(find(status==status_richiesto,1));
    end
    
end

