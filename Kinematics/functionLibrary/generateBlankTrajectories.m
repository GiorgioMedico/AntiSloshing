function [qc_tot,zimmer_tot,time_tot,trajStatRed]=generateBlankTrajectories(punti,sequenzaMoto,trajStatRed,robot)


%% ANALISI INTORNO
%considerare anche un'intorno di posizioni del TCP (quindi dell'kmr)
Ntraj=size(trajStatRed,1);
Npose=size(punti,2);
qc_tot=[];
zimmer_tot=[];
time_tot=[]; last_time=0;
disp(['Evaluating ' num2str(Ntraj) ' trajectories'])
notAval=0;
for t=1:Ntraj
    for p=1:Npose
        punti{p}.ridondanza =trajStatRed(t,2);
        punti{p}.status =    trajStatRed(t,1);
        punti{p}.J = inverseKinematicLBR(robot,punti{p}.T,punti{p}.ridondanza,punti{p}.status); 
    end
    [qc{t},zimmer{t},time{t}] = trajectoryPlan(punti,robot,sequenzaMoto);

    if any(any(isnan(qc{t}))) || size(qc{t},1)~=size(time{t},1)
        trajStatRed(t,1)=NaN;
        trajStatRed(t,2)=NaN;
        notAval=notAval+1;
    else
        zimmer_tot=[zimmer_tot; zimmer{t};];
        qc_tot=[qc_tot; qc{t}];
        if isempty(time_tot)      
            time_tot=time{t};
            last_time=time_tot(end);
        else
            time_tot=[time_tot; time{t}+last_time+(time{t}(end)-time{t}(end-1))];
            last_time=time_tot(end);
        end  

    end
end
disp(['Generated ' num2str(Ntraj-notAval) ' trajectories'])