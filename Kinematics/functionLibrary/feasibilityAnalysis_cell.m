%modificata da Francesco Meoni (da: lbr_kinematic_capabilities by claudio
%mazzotti)
function sol = feasibilityAnalysis_cell(GoalPoint,robot)

% [sol]=feasibilityAnalysis(robot,GoalPoint)
%
% Compute feasibility of GoalPoint. sol is a cell arrays, 
% sol{i} contains the feasibility vector of status i, in the redundancy range
J3_min = robot.qlim(3,1)*180/pi;
J3_max = robot.qlim(3,2)*180/pi;
J3_spacing = J3_min:1:J3_max;
sol=cell(1,8);
for i = 1:1:length(J3_spacing)
    [~,status_poss]=inverseKinematicLBR(robot,GoalPoint,J3_spacing(i));
    solInt=zeros(1,8);
    solInt(status_poss+1)=1;
    
    for j=1:8
        sol{j}(i)=solInt(j);
    end
end

