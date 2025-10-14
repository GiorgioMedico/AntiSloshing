% Creata da Claudio Mazzotti

function [qc,motion_status] = circ_JM(T0,T1,T2, time,time_step,robot,Redundancy,status_richiesto)

t = [0:time_step:time]';
% TC = ctraj(T1, T2, length(t));
TC = ctraj_circular(T0, T1, T2, length(t));
qc = [];
motion_status = [];

qc_start = inverseKinematicLBR_J(robot,TC(:,:,1),Redundancy,status_richiesto);
qc = [qc;qc_start];
motion_status(1) = status_richiesto;

for i = 2:size(TC,3)
    [~,rrs,status] = inverseKinematicLBR(robot,TC(:,:,i),Redundancy);
    sol_distance = [];
    counter = 1;
    
    feasible_soluiton = find(isnan(status)==0);
    for j = find(isnan(status)==0)
        sol_distance(counter) = norm(qc(end,:)-rrs(j,:));
        counter = counter + 1;
    end
    
    [M,I] = min(sol_distance);
    
    if isnan(M)
        disp(['moto non eseguibile'])
        qc = [];
        
    else
        qc = [qc;rrs(feasible_soluiton(I),:)];
        motion_status(i) = status(feasible_soluiton(I));
    end

end

