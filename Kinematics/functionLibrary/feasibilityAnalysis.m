%modificata da Francesco Meoni (da: lbr_kinematic_capabilities by claudio
%mazzotti)
function sol = feasibilityAnalysis(GoalPoint,robot)

% [sol]=feasibilityAnalysis(robot,GoalPoint)
%
% Compute feasibility of GoalPoint. sol is a matrix, 
% sol(i,:)  contains the feasibility vector of status i, in the redundancy range

J3_min = robot.qlim(3,1)*180/pi;
J3_max = robot.qlim(3,2)*180/pi;
J3_spacing = J3_min:1:J3_max;
sol=zeros(8,length(J3_spacing));

for i = 1:1:length(J3_spacing)
    [~,status_poss]=inverseKinematicLBR(robot,GoalPoint,J3_spacing(i));
    sol(status_poss+1,i)=1;
end

