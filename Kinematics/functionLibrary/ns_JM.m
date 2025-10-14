% Creata da Claudio Mazzotti

% lineare Joint Motion.
% è un movimento lineare del TCP dalla posa T1 alla T2.
% la triettoria generata è una interpolazione lineare da T1 a T2
% l'output della function è la traiettoria nello spazio dei giunti.

% function qc = lin_JM(T1,T2,time,time_step,robot,Redundancy,select_solution)
%
% t = [0:time_step:time]';
% TC = ctraj(T1, T2, length(t));
%
% qc = [];
% for i = 1:size(TC,3)
%     [~,matrix,status] = inverseKinematicLBR(robot,TC(:,:,i),Redundancy);
%     qc = [qc;matrix(select_solution,:)];
% end

function [qc,motion_status] = ns_JM(T,time,time_step,robot,status_start,redundancy_start,redundancy_stop)

t = [0:time_step:time]';
redundancy_motion_law = lspb(redundancy_start,redundancy_stop,length(t));
% TC = ctraj(T1, T2, length(t));
qc = [];
motion_status = [];

qc_start = inverseKinematicLBR_J(robot,T,redundancy_start,status_start);
qc = [qc;qc_start];
motion_status(1) = status_start;

for i = 2:length(redundancy_motion_law)
    [~,rrs,status] = inverseKinematicLBR(robot,T,redundancy_motion_law(i));
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

