function [qc,zimmer,time]=automaticBlankTrajectory(punti,sequenzaMoto,robot)

Npose=size(punti,2);
feas=zeros(8,341,Npose);
for p=1:Npose
    feas(:,:,p)=feasibilityAnalysis(punti{p}.T ,robot);
end

[redundancyRange,mval]=redundancyRangeIndex(feas,robot);

%SCELTA AUTOMATICA
[~,statRow]=max([redundancyRange.maxValue]);
chosenStatus=statRow-1;
chosenRedundancy= mean(redundancyRange(statRow).maxDelimiters);
% chosenRange=mval;

plotFeasibilityIndex(feas,robot);
subplot(2,4,statRow)
hold on
plot(redundancyRange(statRow).maxDelimiters(1),1.2,'k+','LineWidth',3.5,'MarkerSize',15);
plot(redundancyRange(statRow).maxDelimiters(2),1.2,'k+','LineWidth',3.5,'MarkerSize',15);
plot(mean(redundancyRange(statRow).maxDelimiters),1.2,'k.','LineWidth',3,'MarkerSize',30);
text(mean(redundancyRange(statRow).maxDelimiters),1.15,num2str(redundancyRange(statRow).maxValue));

for p=1:Npose
    punti{p}.ridondanza =chosenRedundancy;
    punti{p}.status = chosenStatus;
    punti{p}.J = inverseKinematicLBR(robot,punti{p}.T,punti{p}.ridondanza,punti{p}.status);
end

[qc,zimmer,time] = trajectoryPlan(punti,robot,sequenzaMoto);